import serial
import time
import threading
import cv2
from pathlib import Path

from picamera2 import Picamera2
from libcamera import controls

# =========================
# Serial
# =========================
PORT = "/dev/ttyUSB0"
BAUD = 115200
ser = serial.Serial(PORT, BAUD, timeout=0.2)
time.sleep(2)

send_lock = threading.Lock()
running = True

def send_line(s: str):
    # thread-safe write
    with send_lock:
        ser.write((s.strip() + "\n").encode())

# =========================
# Save directory
# =========================
BASE_DIR = Path.home() / "Documents" / "bean_images"
BASE_DIR.mkdir(parents=True, exist_ok=True)

# =========================
# Camera init + restart
# =========================
def start_camera_still(width=2304, height=1296):
    picam2 = Picamera2()
    cfg = picam2.create_still_configuration(
        main={"size": (width, height), "format": "BGR888"}
    )
    picam2.configure(cfg)

    picam2.start()
    time.sleep(0.3)
    return picam2

picam2 = start_camera_still()

def restart_camera():
    global picam2
    try:
        picam2.stop()
        picam2.close()
    except Exception:
        pass
    time.sleep(0.3)
    picam2 = start_camera_still()

def capture_image(bean_id: int, pos: int) -> str:
    """
    bean_id: 아두이노가 부여한 생두 고유 ID
    pos: 현재 촬영 위치(예: 6)
    """
    ts = int(time.time() * 1000)
    img_path = BASE_DIR / f"bean_{bean_id:06d}_pos{pos}_{ts}.jpg"
    try:
        # AF 원샷(오토)
        picam2.set_controls({"AfMode": controls.AfModeEnum.Auto})
        time.sleep(0.25)

        frame = picam2.capture_array()  # (설정상) BGR888

        # ✅ 핵심: 빨강/파랑 스왑 (R<->B)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        ok = cv2.imwrite(str(img_path), frame)
        if not ok:
            print("[ERROR] cv2.imwrite failed")
            return ""
        return str(img_path)

    except Exception as e:
        print(f"[WARN] capture failed: {e} -> restarting camera")
        restart_camera()
        return ""

# =========================
# Dummy classifier: 0/1 alternating
# =========================
toggle = 0
def classify_alternating(_img_path: str) -> int:
    global toggle
    res = toggle
    toggle = 1 - toggle
    return res

# =========================
# Arduino line handler
#  Expect: "CAP <bean_id> <pos>"
# =========================
def handle_line(line: str):
    if not line.startswith("CAP "):
        return

    parts = line.split()
    if len(parts) != 3:
        print(f"[RPi] WARN: bad CAP format: {line}")
        return

    try:
        bean_id = int(parts[1])
        pos = int(parts[2])
    except:
        print(f"[RPi] WARN: parse fail: {line}")
        return

    print(f"[RPi] CAP received: bean_id={bean_id}, pos={pos}")

    img = capture_image(bean_id, pos)
    if not img:
        cls = 1  # fail-safe
        print(f"[RPi] capture FAILED -> RES {bean_id} {cls}")
    else:
        cls = classify_alternating(img)
        print(f"[RPi] saved {img} -> RES {bean_id} {cls}")

    send_line(f"RES {bean_id} {cls}")

# =========================
# Keyboard thread for homing
# =========================
HELP = """
[RPi Command]
  home        -> send HOME
  zero        -> send ZERO
  a/d/A/D     -> send JOG <key>
  help        -> show help
  q           -> quit
"""

def keyboard_loop():
    global running
    print(HELP)
    while running:
        try:
            cmd = input("cmd> ").strip()
        except (EOFError, KeyboardInterrupt):
            running = False
            break

        if cmd == "q":
            running = False
            break
        elif cmd == "help":
            print(HELP)
        elif cmd == "home":
            send_line("HOME")
        elif cmd == "zero":
            send_line("ZERO")
        elif cmd in ["a", "d", "A", "D"]:
            send_line(f"JOG {cmd}")
        else:
            print("unknown. type 'help'")

th = threading.Thread(target=keyboard_loop, daemon=True)
th.start()

print("RPi: ready, waiting for CAP <bean_id> <pos> ...")

try:
    while running:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode(errors="ignore").strip()
        if not line:
            continue

        # 아두이노 로그 보고 싶으면 주석 해제
        # print("ARD:", line)

        handle_line(line)

finally:
    running = False
    try:
        ser.close()
    except:
        pass
    try:
        picam2.stop()
        picam2.close()
    except:
        pass
    print("RPi: stopped")
