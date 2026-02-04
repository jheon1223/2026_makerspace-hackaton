#include <Servo.h>

// =========================================================
// DEBUG switch
//  - DEBUG=1 : print debug logs
//  - DEBUG=0 : no debug logs (protocol CAP/RES still works)
// =========================================================
#define DEBUG 1

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
const unsigned long T_NOR_OPEN_MS       = 200;   // 정상 배출 게이트 열어두는 시간
const unsigned long WAIT_RES_TIMEOUT_MS = 8000;  // 라파 응답 타임아웃

// -------------------- Slot / Index --------------------
const int N_SLOTS = 24;

// "pos"는 물리적으로 고정된 위치(0~23), head로 배열 매핑
const int CAPTURE_POS       = 2;
const int NORMAL_EJECT_POS  = 8;
const int DEFECT_EJECT_POS  = 11;

const bool DIR_FORWARD = true; // true면 정방향

// -------------------- Stepper Config --------------------
const long STEPS_PER_REV    = 3200; // 200*16 (1.8deg, 1/16)
const int  STEPS_CELL_BASE  = STEPS_PER_REV / N_SLOTS; // 133
const int  STEPS_CELL_REM   = STEPS_PER_REV % N_SLOTS; // 8
int cell_err_acc = 0;

// -------------------- Bean tracking --------------------
// state: -1 empty, 3 entered, 2 capture requested(wait), 1 normal, 0 defect
int   beanId[N_SLOTS];
int8_t beanState[N_SLOTS];

// head: "물리 pos=0"이 배열에서 어디를 가리키는가
int head = 0;

// 증가하는 생두 고유 ID
int nextBeanId = 0;

// 캡처 대기 중인 bean_id
int waitingBeanId = -1;

// -------------------- FSM State --------------------
enum State {
  ST_HOME_WAIT,
  ST_INIT,
  ST_FEED_ROLL_START,
  ST_FEED_ROLL_WAIT,
  ST_STEP_ONE_CELL,
  ST_CHECK_CAPTURE,
  ST_WAIT_RESULT,
  ST_CHECK_EJECT,
  ST_NORMAL_EJECT_OPEN,
  ST_NORMAL_EJECT_CLOSE_WAIT,
  ST_ERROR
};

State st = ST_INIT;
unsigned long t0 = 0;

// =========================================================
// Helpers
// =========================================================
inline int slotIndexFromPos(int pos) {
  // pos: 물리 위치(0~23)
  // head: 물리 pos0가 배열에서 head를 가리킴
  int idx = head + pos;
  idx %= N_SLOTS;
  return idx;
}

void initSlots() {
  for (int i = 0; i < N_SLOTS; i++) {
    beanId[i] = -1;
    beanState[i] = -1;
  }
  waitingBeanId = -1;
}

void stepperEnable(bool en) {
  // DRV8825: EN low = enable
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

  DPRINT("[STEP] one cell microsteps=");
  DPRINTLN(steps);

  digitalWrite(PIN_DIR, DIR_FORWARD ? HIGH : LOW);
  moveMicrosteps(steps);

  // 한 칸 이동했으니 head만 전진
  if (DIR_FORWARD) {
    head = (head + N_SLOTS - 1) % N_SLOTS; // head--
  } else {
    head = (head + 1) % N_SLOTS;           // head++
  }
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

// =========================================================
// Protocol
//  Arduino -> RPi: "CAP <bean_id> <pos>\n"
//  RPi -> Arduino: "RES <bean_id> <cls>\n"
// =========================================================
void sendCaptureRequest(int bean_id, int pos) {
  // 프로토콜은 DEBUG와 무관하게 항상 출력
  Serial.print("CAP ");
  Serial.print(bean_id);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print("\n");

  DPRINT("[TX] CAP bean_id=");
  DPRINT(bean_id);
  DPRINT(" pos=");
  DPRINTLN(pos);
}

// RES 파싱: "RES <bean_id> <cls>"
bool parseResLine(const String &line, int &bean_id, int &cls) {
  if (!line.startsWith("RES ")) return false;

  int p1 = line.indexOf(' ');
  int p2 = line.indexOf(' ', p1 + 1);
  if (p1 < 0 || p2 < 0) return false;

  bean_id = line.substring(p1 + 1, p2).toInt();
  cls = line.substring(p2 + 1).toInt();

  if (bean_id <= 0) return false;
  if (!(cls == 0 || cls == 1)) return false;

  return true;
}

bool readLine(String &out) {
  if (!Serial.available()) return false;
  out = Serial.readStringUntil('\n');
  out.trim();
  return out.length() > 0;
}

// =========================================================
// Find bean_id in slots
// =========================================================
int findBeanIndexById(int bean_id) {
  for (int i = 0; i < N_SLOTS; i++) {
    if (beanId[i] == bean_id) return i;
  }
  return -1;
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

// 라파 명령 처리 (처리했으면 true)
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

    // microstep grid align (선택: 안정적으로 하기 위함)
    
    digitalWrite(PIN_DIR, DIR_FORWARD ? HIGH : LOW);
    moveMicrosteps(STEPS_CELL_BASE);
    digitalWrite(PIN_DIR, !DIR_FORWARD ? HIGH : LOW);
    moveMicrosteps(STEPS_CELL_BASE);

    // 논리 상태 초기화
    head = 0;
    cell_err_acc = 0;
    initSlots();
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

  initSlots();
  head = 0;
  nextBeanId = 0;
  cell_err_acc = 0;

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

    // RES 처리
    int rid, rcls;
    if (parseResLine(line, rid, rcls)) {
      int idx = findBeanIndexById(rid);
      if (idx >= 0) {
        beanState[idx] = (int8_t)rcls;
        DPRINT("[RX] RES bean_id=");
        DPRINT(rid);
        DPRINT(" cls=");
        DPRINTLN(rcls);

        // 기다리던 bean이면 waiting 해제
        if (waitingBeanId == rid) {
          waitingBeanId = -1;
        }
      } else {
        DPRINT("[RX] RES for unknown bean_id=");
        DPRINTLN(rid);
      }
    }
  }

  // 2) FSM
  switch (st) {
    case ST_HOME_WAIT:
      // 라파에서 HOME/JOG/ZERO 올 때까지 대기
      break;

    case ST_INIT:
      DPRINTLN("[STATE] INIT -> FEED");
      st = ST_FEED_ROLL_START;
      break;

    case ST_FEED_ROLL_START:
      rollersStart(FEED_SPEED);
      t0 = millis();
      st = ST_FEED_ROLL_WAIT;
      break;

    case ST_FEED_ROLL_WAIT:
      if (millis() - t0 >= T_ROLL_MS) {
        rollersStop();
        t0 = millis();

        // feed 완료 -> pos0에 새 원두 등록
        int i0 = slotIndexFromPos(0);
        if (beanState[i0] != -1) {
          // pos0이 비어있지 않으면 논리/기구 문제
          DPRINTLN("[ERR] pos0 not empty when feeding!");
          st = ST_ERROR;
          break;
        }

        int newId = ++nextBeanId;
        beanId[i0] = newId;
        beanState[i0] = 3; // entered
        DPRINT("[FEED] new bean_id=");
        DPRINTLN(newId);

        st = ST_STEP_ONE_CELL;
      }
      break;

    case ST_STEP_ONE_CELL:
      if (millis() - t0 >= T_FEED_SETTLE_MS) {
        moveOneCell();
        // 이동했으니 바로 캡처 체크
        st = ST_CHECK_CAPTURE;
      }
      break;

    case ST_CHECK_CAPTURE: {
      int ic = slotIndexFromPos(CAPTURE_POS);

      if (beanState[ic] == 3 && beanId[ic] > 0) {
        int bid = beanId[ic];

    #if DEBUG
        // 🔥 DEBUG 모드: 바로 정상(또는 원하는 값)으로 처리
        beanState[ic] = 1;   // 1 = normal (0으로 하면 defect)
        waitingBeanId = -1;
        DPRINTLN("[DEBUG] skip CAP/RES, auto NORMAL");
        st = ST_CHECK_EJECT;
    #else
        beanState[ic] = 2;   // capture requested
        waitingBeanId = bid;
        sendCaptureRequest(bid, CAPTURE_POS);
        t0 = millis();
        st = ST_WAIT_RESULT;
    #endif

      } else {
        st = ST_CHECK_EJECT;
      }
      break;
    }


    case ST_WAIT_RESULT: {
      // waitingBeanId가 -1이 되면(RES 받음) 종료
      if (waitingBeanId == -1) {
        DPRINTLN("[STATE] RES received -> CHECK_EJECT");
        st = ST_CHECK_EJECT;
      } else if (millis() - t0 >= WAIT_RES_TIMEOUT_MS) {
        // 타임아웃이면 해당 bean_id를 defect(0)로 처리
        int idx = findBeanIndexById(waitingBeanId);
        if (idx >= 0) {
          beanState[idx] = 0; // defect
          DPRINT("[WARN] RES timeout -> set DEFECT bean_id=");
          DPRINTLN(waitingBeanId);
        }
        waitingBeanId = -1;
        st = ST_CHECK_EJECT;
      }
      break;
    }

    case ST_CHECK_EJECT: {
      int in = slotIndexFromPos(NORMAL_EJECT_POS);
      int idn = beanId[in];
      int8_t sn = beanState[in];

      // 정상 배출 위치: state==1이면 gate 열기
      if (sn == 1 && idn > 0) {
        DPRINT("[EJECT] NORMAL bean_id=");
        DPRINTLN(idn);
        st = ST_NORMAL_EJECT_OPEN;
        break;
      } else {
        // 정상 아니면 gate 닫아두기
        gateNormal.write(NOR_CLOSE);
      }

      // 결점 배출 위치: 뭔가 있으면(0 포함, 혹은 정상도 실수로 오면) 드랍 처리
      int id = slotIndexFromPos(DEFECT_EJECT_POS);
      int bid = beanId[id];
      int8_t sd = beanState[id];

      if (sd != -1 && bid > 0) {
        // 여기선 servo 없이 그냥 떨어지는 구조라고 가정 -> 상태 clear
        if (sd == 1) {
          DPRINT("[WARN] NORMAL reached DEFECT eject! bean_id=");
          DPRINTLN(bid);
        } else {
          DPRINT("[EJECT] DEFECT drop bean_id=");
          DPRINTLN(bid);
        }

        beanId[id] = -1;
        beanState[id] = -1;
      }

      // 다음 사이클로
      st = ST_FEED_ROLL_START;
      break;
    }

    case ST_NORMAL_EJECT_OPEN: {
      gateNormal.write(NOR_OPEN);
      t0 = millis();
      st = ST_NORMAL_EJECT_CLOSE_WAIT;
      break;
    }

    case ST_NORMAL_EJECT_CLOSE_WAIT: {
      if (millis() - t0 >= T_NOR_OPEN_MS) {
        gateNormal.write(NOR_CLOSE);

        // 정상 배출 위치 슬롯 비우기
        int in = slotIndexFromPos(NORMAL_EJECT_POS);
        DPRINT("[EJECT] NORMAL cleared pos=");
        DPRINTLN(NORMAL_EJECT_POS);

        beanId[in] = -1;
        beanState[in] = -1;

        st = ST_FEED_ROLL_START;
      }
      break;
    }

    case ST_ERROR:
      DPRINTLN("[ERROR] halted");
      stepperEnable(false);
      rollersStop();
      gateNormal.write(NOR_CLOSE);
      // 멈춰있음
      break;
  }
}
