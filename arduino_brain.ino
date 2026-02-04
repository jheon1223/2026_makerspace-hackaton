#include <Servo.h>

// =========================================================
// DEBUG switch
//  - DEBUG=1 : print debug logs
//  - DEBUG=0 : no debug logs (protocol CAP/RES still works)
// =========================================================
#define DEBUG 0

#if DEBUG
  #define DPRINT(x)   do { Serial.print("DBG "); Serial.print(x); } while(0)
  #define DPRINTLN(x) do { Serial.print("DBG "); Serial.println(x); } while(0)
#else
  #define DPRINT(x)   do {} while(0)
  #define DPRINTLN(x) do {} while(0)
#endif

// =========================================================
// Manual Home via RPi (no Serial Monitor needed)
//  RPi sends:
//    HOME
//    ZERO
//    JOG a/d/A/D
// =========================================================
#define MANUAL_HOME 1
const int JOG_SMALL = 5;    // microsteps per tap
const int JOG_BIG   = 50;   // microsteps per tap

// -------------------- Stepper Pin Map --------------------
const int PIN_DIR   = 2;
const int PIN_STEP  = 3;
const int PIN_EN    = 8;

// -------------------- Servo Pin Map --------------------
const int PIN_GATE_NORMAL  = 6;   // 180deg: normal eject gate (5 o'clock)
const int PIN_ROLLER_A     = 9;   // 360deg roller A
const int PIN_ROLLER_B     = 10;  // 360deg roller B

Servo gateNormal;
Servo rollerA;
Servo rollerB;

// -------------------- 360 Servo params --------------------
const int STOP_360   = 90;
const int FEED_SPEED = 8;   // 0~40 권장(서보마다 다름)

// -------------------- Normal gate angles --------------------
const int NOR_CLOSE = 0;
const int NOR_OPEN  = 70;

// -------------------- Timing --------------------
const unsigned long T_ROLL_MS           = 250;   // 롤러로 원두 밀어넣는 시간
const unsigned long T_FEED_SETTLE_MS    = 200;    // stop 후 안정화
const unsigned long WAIT_RES_TIMEOUT_MS = 8000;  // 라파 응답 타임아웃

// -------------------- Slot / Pos --------------------
const int N_SLOTS = 24;
const int CAPTURE_POS       = 2; // 라파가 촬영하는 위치(물리 pos)
const int NORMAL_EJECT_POS  = 8;
const int DEFECT_EJECT_POS  = 11;

// -------------------- Direction --------------------
const bool DIR_FORWARD = true; // true면 정방향

// -------------------- Stepper Config --------------------
const long STEPS_PER_REV    = 3200; // 200*16 (1.8deg, 1/16)
const int  STEPS_CELL_BASE  = STEPS_PER_REV / N_SLOTS; // 133
const int  STEPS_CELL_REM   = STEPS_PER_REV % N_SLOTS; // 8
int cell_err_acc = 0;

// -------------------- Logical head --------------------
// head: "물리 pos=0"이 배열에서 어디를 가리키는가 (HOME/ZERO 정렬용)
int head = 0;

// -------------------- simple bean id for CAP only --------------------
int nextBeanId = 0;       // CAP 보낼 때만 증가
int waitingBeanId = -1;   // camera_done_* 기다리는 동안만 사용

// -------------------- FSM State --------------------
enum State {
  ST_HOME_WAIT,
  ST_INIT,

  // CAP 요청 -> 라파 응답 대기 -> 움직임 -> 완료 통지
  ST_CHECK_CAPTURE,
  ST_WAIT_RESULT,
  ST_DO_MOVE,

  ST_ERROR
};

State st = ST_INIT;
unsigned long t0 = 0;

// =========================================================
// Helpers
// =========================================================
void stepperEnable(bool en) {
  digitalWrite(PIN_EN, en ? LOW : HIGH); // DRV8825: EN low = enable
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

  DPRINT("[STEP] one cell microsteps=");
  DPRINTLN(steps);

  digitalWrite(PIN_DIR, DIR_FORWARD ? HIGH : LOW);
  moveMicrosteps(steps);

  // head 갱신
  if (DIR_FORWARD) head = (head + N_SLOTS - 1) % N_SLOTS;
  else             head = (head + 1) % N_SLOTS;
}

void rollersStart(int speed) {
  speed = constrain(speed, 0, 40);
  DPRINT("[FEED] rollers start speed=");
  DPRINTLN(speed);

  rollerA.write(STOP_360 + speed);
  rollerB.write(STOP_360 - speed);
}

void rollersStop() {
  DPRINTLN("[FEED] rollers stop");
  rollerA.write(STOP_360);
  rollerB.write(STOP_360);
}

bool readLine(String &out) {
  if (!Serial.available()) return false;
  out = Serial.readStringUntil('\n');
  out.trim();
  return out.length() > 0;
}

// =========================================================
// Protocol
//  RPi -> Arduino: "camera_done_open\n" or "camera_done_close\n"
//  Arduino -> RPi: "move_complete\n"
// =========================================================
void sendCaptureRequest(int bean_id, int pos) {
  Serial.print("CAP ");
  Serial.print(bean_id);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print("\n");
}

void sendMoveComplete() {
  Serial.print("move_complete\n");
}

// camera_done_* 파싱
// open  => gate open (NOR_OPEN)
// close => gate close (NOR_CLOSE)
bool parseCameraDone(const String &line, bool &gateOpen) {
  if (line == "camera_done_open")  { gateOpen = true;  return true; }
  if (line == "camera_done_close") { gateOpen = false; return true; }
  return false;
}

// =========================================================
// Home commands (from RPi)
// =========================================================
void printHomeHelp() {
  DPRINTLN("=== HOME MODE (from RPi) ===");
  DPRINTLN("RPi cmd: HOME / ZERO / JOG a|d|A|D");
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

  DPRINT("[HOME] jog ");
  DPRINT(dir == DIR_FORWARD ? "FWD " : "REV ");
  DPRINTLN(jog);
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
    DPRINTLN("[PI] HOME -> ST_HOME_WAIT");
    return true;
  }

  if (line == "ZERO") {
    if (st != ST_HOME_WAIT) {
      DPRINTLN("[PI] ZERO ignored (not in HOME_WAIT)");
      return true;
    }

    // microstep grid align (선택: 안정적으로 하기 위함) //이거 빼도 되는지 확인하기
    
    digitalWrite(PIN_DIR, DIR_FORWARD ? HIGH : LOW);
    moveMicrosteps(STEPS_CELL_BASE);
    digitalWrite(PIN_DIR, !DIR_FORWARD ? HIGH : LOW);
    moveMicrosteps(STEPS_CELL_BASE);

    // 논리 상태 초기화
    head = 0;
    cell_err_acc = 0;
    waitingBeanId = -1;

    DPRINTLN("[PI] ZERO -> aligned, start FSM");
    st = ST_INIT;
    return true;
  }

  if (line.startsWith("JOG ")) {
    if (st != ST_HOME_WAIT) return true; // 명령은 받았지만 무시
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

  head = 0;
  nextBeanId = 0;
  cell_err_acc = 0;
  waitingBeanId = -1;

#if MANUAL_HOME
  st = ST_HOME_WAIT;
  DPRINTLN("[BOOT] HOME_WAIT (use RPi commands)");
  printHomeHelp();
#else
  st = ST_INIT;
#endif
}

// =========================================================
// Main loop
// =========================================================
void loop() {
  // 1) Serial line 처리
  String line;
  if (readLine(line)) {
    // 라파 HOME/JOG/ZERO 먼저 처리 -> 처리되면 이번 loop는 FSM 진행하지 않음(안전)
    if (handlePiCommand(line)) return;

    // camera_done_* 처리
    bool gateOpen;
    if (parseCameraDone(line, gateOpen)) {
      if (st == ST_WAIT_RESULT && waitingBeanId != -1) {
        // 라파가 판독 끝났으니, 결과에 따라 "이번 동작 끝에서" 게이트를 열지/닫을지 결정
        // gateOpen: true면 열기, false면 닫기
        gateNormal.write(gateOpen ? NOR_OPEN : NOR_CLOSE);

        DPRINT("[RX] camera_done_");
        DPRINTLN(gateOpen ? "open" : "close");

        waitingBeanId = -1; // 응답 수신 완료
        st = ST_DO_MOVE;    // 이제 움직임 수행
      }
      return;
    }
  }

  // 2) FSM
  switch (st) {
    case ST_HOME_WAIT:
      // 라파에서 HOME/JOG/ZERO 올 때까지 대기
      break;

    case ST_INIT:
      st = ST_FEED_ROLL_START; //처음 두 칸은 돌아야됨.
      break;

    case ST_CHECK_CAPTURE: {
      // 여기서는 “촬영 위치에 원두가 왔다”라고 가정하고,
      // 라파가 사진 찍고 판독할 수 있도록 CAP을 보내고 멈춰서 기다림
      int bid = ++nextBeanId;
      waitingBeanId = bid;
      sendCaptureRequest(bid, CAPTURE_POS);
      t0 = millis();
      st = ST_WAIT_RESULT;
      break;
    }

    case ST_WAIT_RESULT:
      // camera_done_* 올 때까지 대기 (timeout 시 안전하게 close 처리 후 진행)
      if (waitingBeanId == -1) {
        // 이미 camera_done_*를 받아서 ST_DO_MOVE로 넘어가도록 위에서 처리됨
      } else if (millis() - t0 >= WAIT_RES_TIMEOUT_MS) {
        // 타임아웃이면 안전하게 close로 처리하고 움직임 진행
        gateNormal.write(NOR_CLOSE);
        waitingBeanId = -1;
        st = ST_DO_MOVE;
      }
      break;

    case ST_DO_MOVE:
      // 요구한 움직임: (롤러 2개 돌리기 -> 스텝모터 한 칸 -> 게이트는 (이미 open/close로 세팅됨))
      rollersStart(FEED_SPEED);
      delay(T_ROLL_MS);
      rollersStop();
      delay(T_FEED_SETTLE_MS);

      moveOneCell();
      sendMoveComplete(); // 모든 움직임 끝
      st = ST_CHECK_CAPTURE;
      break;


    case ST_ERROR:
      DPRINTLN("[ERROR] halted");
      stepperEnable(false);
      rollersStop();
      gateNormal.write(NOR_CLOSE);
      // 멈춰있음
      break;
  }
}
