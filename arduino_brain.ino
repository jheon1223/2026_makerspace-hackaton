#include <Servo.h>

#define DEBUG 0
#if DEBUG
  #define DPRINT(x)   do { Serial.print("DBG "); Serial.print(x); } while(0)
  #define DPRINTLN(x) do { Serial.print("DBG "); Serial.println(x); } while(0)
#else
  #define DPRINT(x)   do {} while(0)
  #define DPRINTLN(x) do {} while(0)
#endif

#define MANUAL_HOME 1
const int JOG_SMALL = 5;
const int JOG_BIG   = 50;

// -------------------- Stepper Pin Map --------------------
const int PIN_DIR   = 2;
const int PIN_STEP  = 3;
const int PIN_EN    = 8;

// -------------------- Servo Pin Map --------------------
const int PIN_GATE_NORMAL  = 6;
const int PIN_ROLLER_A     = 9;
const int PIN_ROLLER_B     = 10;

Servo gateNormal;
Servo rollerA;
Servo rollerB;

// -------------------- Params --------------------
const int STOP_360   = 90;
const int FEED_SPEED = 8;

const int NOR_CLOSE = 0;
const int NOR_OPEN  = 70;

const unsigned long T_ROLL_MS        = 250;
const unsigned long T_FEED_SETTLE_MS = 200;

const int  N_SLOTS = 24;
const bool DIR_FORWARD = true;

const long STEPS_PER_REV    = 3200;
const int  STEPS_CELL_BASE  = STEPS_PER_REV / N_SLOTS; // 133
const int  STEPS_CELL_REM   = STEPS_PER_REV % N_SLOTS; // 8
int cell_err_acc = 0;

// -------------------- FSM --------------------
enum State {
  ST_HOME_WAIT,
  ST_INIT,
  ST_WAIT_PI_CMD,
  ST_DO_MOVE,
  ST_ERROR
};
State st = ST_INIT;

// gate를 DO_MOVE 끝에서 어떤 상태로 둘지
bool pendingGateOpen = false;

// ⭐ “진짜 첫 ZERO”인지 여부
bool firstZeroDone = false;

// =========================================================
// Helpers
// =========================================================
void stepperEnable(bool en) {
  digitalWrite(PIN_EN, en ? LOW : HIGH);
}

void stepPulse(int delayUs) {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(delayUs);
}

void moveMicrosteps(long steps, int pulseDelayUs = 1500) {
  for (long i = 0; i < steps; i++) stepPulse(pulseDelayUs);
}

void moveOneCell() {
  int steps = STEPS_CELL_BASE;
  cell_err_acc += STEPS_CELL_REM;
  if (cell_err_acc >= N_SLOTS) {
    steps += 1;
    cell_err_acc -= N_SLOTS;
  }
  digitalWrite(PIN_DIR, DIR_FORWARD ? HIGH : LOW);
  moveMicrosteps(steps);
}

void rollersStart(int speed) {
  speed = constrain(speed, 0, 40);
  rollerA.write(STOP_360 + speed);
  rollerB.write(STOP_360 - speed);
}
void rollersStop() {
  rollerA.write(STOP_360);
  rollerB.write(STOP_360);
}

bool readLine(String &out) {
  if (!Serial.available()) return false;
  out = Serial.readStringUntil('\n');
  out.trim();
  return out.length() > 0;
}

void sendMoveComplete() {
  Serial.print("move_complete\n");
}

// =========================================================
// Home commands (유지)
// =========================================================
void printHomeHelp() {
  DPRINTLN("=== HOME MODE (from RPi) ===");
  DPRINTLN("RPi cmd: HOME / ZERO / JOG a|d|A|D");
  DPRINTLN("Also: camera_done_open / camera_done_close");
  DPRINTLN("============================");
}

void handleHomeJog(const String &cmd) {
  int jog = 0;
  bool dir = DIR_FORWARD;

  if (cmd == "a") { dir = !DIR_FORWARD; jog = JOG_SMALL; }
  else if (cmd == "d") { dir = DIR_FORWARD; jog = JOG_SMALL; }
  else if (cmd == "A") { dir = !DIR_FORWARD; jog = JOG_BIG; }
  else if (cmd == "D") { dir = DIR_FORWARD; jog = JOG_BIG; }
  else return;

  digitalWrite(PIN_DIR, dir ? HIGH : LOW);
  moveMicrosteps(jog);
}

bool handlePiCommand(const String &line) {
  if (line == "HELP") {
    printHomeHelp();
    return true;
  }

  if (line == "HOME") {
    rollersStop();
    gateNormal.write(NOR_CLOSE);
    st = ST_HOME_WAIT;
    return true;
  }

  if (line == "ZERO") {
    if (st != ST_HOME_WAIT) return true;

    // microstep grid align (기존 유지)
    digitalWrite(PIN_DIR, DIR_FORWARD ? HIGH : LOW);
    moveMicrosteps(STEPS_CELL_BASE);
    digitalWrite(PIN_DIR, !DIR_FORWARD ? HIGH : LOW);
    moveMicrosteps(STEPS_CELL_BASE);

    // 누적 오차 리셋
    cell_err_acc = 0;

    // ZERO 이후에는 무조건: 게이트 닫고 1칸 이동 + move_complete
    gateNormal.write(NOR_CLOSE);

    moveOneCell();
    delay(30);
    sendMoveComplete();

    // 진짜 "첫 ZERO" 때만 추가로 1칸 더 이동 + move_complete
    if (!firstZeroDone) {
      moveOneCell();
      delay(30);
      sendMoveComplete();
      firstZeroDone = true;
    }

    // 이후는 라파 명령 대기 상태로
    st = ST_WAIT_PI_CMD;
    return true;
  }

  if (line.startsWith("JOG ")) {
    if (st != ST_HOME_WAIT) return true;
    if (line.length() < 5) return true;
    String c = line.substring(4);
    c.trim();
    handleHomeJog(c);
    return true;
  }

  return false;
}

// =========================================================
// Setup
// =========================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_EN, OUTPUT);

  stepperEnable(true);

  gateNormal.attach(PIN_GATE_NORMAL);
  rollerA.attach(PIN_ROLLER_A);
  rollerB.attach(PIN_ROLLER_B);

  gateNormal.write(NOR_CLOSE);
  rollersStop();

  cell_err_acc = 0;
  pendingGateOpen = false;
  firstZeroDone = false;

#if MANUAL_HOME
  st = ST_HOME_WAIT;
  printHomeHelp();
#else
  st = ST_INIT;
#endif
}

// =========================================================
// Main loop
// =========================================================
void loop() {
  // Serial 수신
  String line;
  if (readLine(line)) {
    if (handlePiCommand(line)) return;

    // camera_done_*는 대기 상태에서만 수용
    if (st == ST_WAIT_PI_CMD) {
      if (line == "camera_done_open") {
        pendingGateOpen = true;
        st = ST_DO_MOVE;
        return;
      }
      if (line == "camera_done_close") {
        pendingGateOpen = false;
        st = ST_DO_MOVE;
        return;
      }
    }
  }

  switch (st) {
    case ST_HOME_WAIT:
      break;

    case ST_INIT:
      // MANUAL_HOME=0일 때만 의미 있지만 유지
      st = ST_WAIT_PI_CMD;
      break;

    case ST_WAIT_PI_CMD:
      break;

    case ST_DO_MOVE:
      // 1) 게이트를 먼저 세팅 (open/close)
      gateNormal.write(pendingGateOpen ? NOR_OPEN : NOR_CLOSE);
      delay(30); // 서보가 각도에 도달할 시간(필요시 50~150ms로 조절)

      // 2) 롤러 회전
      rollersStart(FEED_SPEED);
      delay(T_ROLL_MS);
      rollersStop();
      delay(T_FEED_SETTLE_MS);

      // 3) 스텝모터 1칸 이동
      moveOneCell();

      // 4) 완료 알림
      sendMoveComplete();
      st = ST_WAIT_PI_CMD;
      break;


    case ST_ERROR:
      stepperEnable(false);
      rollersStop();
      gateNormal.write(NOR_CLOSE);
      break;
  }
}
