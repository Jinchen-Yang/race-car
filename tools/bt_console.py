#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bt_console.py —— CSV 文本遥测 + 有线/蓝牙双向调参台。

单键即发(无需回车), 按键绿色回显, 调参后自动刷新状态并黄色高亮变化的数值。

依赖: pip install pyserial
用法:
    python3 tools/bt_console.py                      # 自动找 XDS110 有线口
    python3 tools/bt_console.py /dev/tty.usbmodemXXX # 指定口
"""
import re
import sys
import threading
import time
import tty
import termios

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("缺 pyserial: python3 -m pip install pyserial")

# 26 列 CSV 列名
CSV_COLUMNS = [
    "ms", "state", "gray_err", "lost", "fresh", "ok_delta", "fail_delta",
    "raw_or_neg1", "gray_hits", "line_ok", "gray_min_hits", "aim_stop_us",
    "trim", "arc_kp_x100", "kd_x100", "arc_scale_x1000", "base",
    "line_kp_x100", "dy", "seg", "turn_diff", "left_duty", "right_duty",
    "nodata", "protect_ms", "startprot_ms",
]

# 可直接敲的固件单字符
RAW_CHARS = set("1234sxrRpPdDvVhHtToOcCjJkKgGyY?")

# ANSI 颜色
C_GREEN  = "\033[1;32m"   # 按键回显
C_YELLOW = "\033[1;33m"   # 变化的数值
C_RED    = "\033[1;31m"   # 退出
C_CYAN   = "\033[1;36m"   # 固件回显标签
C_RESET  = "\033[0m"

# ---- 共享状态 ----
_lock = threading.Lock()
_latest = None            # 最近一帧 CSV dict
_frames = 0
_servo_line = ""
_frame_event = threading.Event()  # 新 CSV 帧到达时 set


def autodetect():
    prefer_wired = ("usbmodem", "usbserial")
    prefer_bt = ("racecar", "hc-05", "hc05")
    ports = list(list_ports.comports())
    for p in ports:
        if any(k in p.device.lower() for k in prefer_wired):
            return p.device
    for p in ports:
        if any(k in p.device.lower() for k in prefer_bt):
            return p.device
    return None


def list_ports_and_exit():
    print("可用串口:")
    for p in list_ports.comports():
        print(f"  {p.device}   {p.description}")
    sys.exit("\n用法: python3 tools/bt_console.py <串口> [波特率=115200]")


# ---- 串口读取线程 ----

def reader(ser, stop):
    global _latest, _frames
    line = bytearray()
    while not stop.is_set():
        chunk = ser.read(4096)
        if not chunk:
            continue
        for b in chunk:
            if b in (10, 13):
                if line:
                    _process_line(line)
                    line.clear()
            elif 32 <= b <= 126:
                line.append(b)
                if len(line) > 500:
                    line.clear()
            else:
                line.clear()


def _process_line(raw: bytearray):
    global _latest, _frames, _servo_line
    text = raw.decode("utf-8", errors="ignore").strip()
    if not text:
        return
    if _looks_like_csv(text):
        parts = text.split(",")
        if len(parts) == len(CSV_COLUMNS):
            try:
                vals = [float(x) for x in parts]
                d = dict(zip(CSV_COLUMNS, vals))
                with _lock:
                    _latest = d
                    _frames += 1
                _frame_event.set()
            except ValueError:
                pass
        return
    if any(m in text for m in ("[CMD]", "[ERR]", "[DIAG]", "[TEST]", "[USER]",
                                "MSPM0 boot", "IMU ", "SERVO=", "cont stop")):
        with _lock:
            if "SERVO=" in text:
                _servo_line = text
        sys.stdout.write(f"\033[K{C_CYAN}{text}{C_RESET}\r\n")
        sys.stdout.flush()


def _looks_like_csv(text: str) -> bool:
    if text.count(",") < 10:
        return False
    return all(ch in set("0123456789,+-.eE ") for ch in text)


# ---- CSV 变化高亮 ----

def _snapshot_csv():
    """取当前 CSV 快照"""
    with _lock:
        return dict(_latest) if _latest else {}


def _fmt_csv_diff(prev: dict, cur: dict) -> str:
    """格式化 CSV 状态, 变化的数值黄色高亮"""
    if not cur:
        return "  (还没收到 CSV 数据)"
    st = int(cur["state"])
    seg = int(cur["seg"])
    stname = {0: "IDLE", 1: "RUN", 2: "AIM", 3: "STOP", 4: "ESTOP"}.get(st, "?")

    def _v(key, fmt="+7.0f"):
        val = cur[key]
        s = f"{val:{fmt}}"
        if prev and key in prev and prev[key] != val:
            return f"{C_YELLOW}{s}{C_RESET}"
        return s

    def _vi(key):
        val = int(cur[key])
        s = str(val)
        if prev and key in prev and int(prev[key]) != val:
            return f"{C_YELLOW}{s}{C_RESET}"
        return s

    lines = [
        f"  state={stname}  seg={seg}  t={_vi('ms')}ms  "
        f"arcKP={_v('arc_kp_x100', '.0f')}  KD={_v('kd_x100', '.0f')}  "
        f"V={_vi('base')}  trim={_vi('trim')}  lineKP={_v('line_kp_x100', '.0f')}",
        f"  gray={_v('gray_err', '+7.0f')}  hits={_vi('gray_hits')}  "
        f"dy={_v('dy', '+7.1f')}  turn={_v('turn_diff', '+6.0f')}  "
        f"L={_v('left_duty', '+6.0f')}  R={_v('right_duty', '+6.0f')}  "
        f"aimStop={_vi('aim_stop_us')}",
    ]
    return "\r\n".join(lines)


# ---- 面板 ----

def fmt_dashboard():
    with _lock:
        d = _latest
        frames = _frames
        servo = _servo_line
    if d is None:
        return "  (还没收到 CSV 数据)"
    st = int(d["state"])
    seg = int(d["seg"])
    stname = {0: "IDLE", 1: "RUN", 2: "AIM", 3: "STOP", 4: "ESTOP"}.get(st, "?")
    lines = [
        f"  帧#{frames}  state={stname}  seg={seg}  t={int(d['ms'])}ms",
        f"  [灰度] err={d['gray_err']:+7.0f}  hits={int(d['gray_hits'])}  "
        f"ok={int(d['line_ok'])}  raw={int(d['raw_or_neg1'])}  lost={int(d['lost'])}",
        f"  [循线] dy={d['dy']:+7.1f}  turn={d['turn_diff']:+6.0f}  "
        f"seg={seg}  nodata={int(d['nodata'])}",
        f"  [驱动] L={d['left_duty']:+6.0f}  R={d['right_duty']:+6.0f}  "
        f"base={int(d['base'])}  trim={int(d['trim'])}",
        f"  [PID]  arcKP={d['arc_kp_x100']/100:.2f}  KD={d['kd_x100']/100:.2f}  "
        f"scale={d['arc_scale_x1000']/1000:.3f}  lineKP={d['line_kp_x100']/100:.2f}",
        f"  [保护] prot={int(d['protect_ms'])}ms  startprot={int(d['startprot_ms'])}ms  "
        f"aimStop={int(d['aim_stop_us'])}us",
    ]
    if servo:
        s = servo.split("SERVO=", 1)[1] if "SERVO=" in servo else servo
        lines.append(f"  [云台] SERVO={s}")
    return "\r\n".join(lines)


HELP = (
    f"\r\n{C_CYAN}=== 单键命令(无需回车) ==={C_RESET}\r\n"
    f"  {C_GREEN}舵机{C_RESET}: O 上电  o 断电  C 归中  J/j 俯仰±1  K/k 水平±5  Y/y 停转±5us\r\n"
    f"  {C_GREEN}模式{C_RESET}: 1=F1  2=F2  3=F3  4=F4\r\n"
    f"  {C_GREEN}运控{C_RESET}: s 启动  x 急停  r 清障(ESTOP下连按3次)\r\n"
    f"  {C_GREEN}调参{C_RESET}: P/p 弧线KP  D/d KD  V/v 基速  H/h 直线KP  T/t 配平  G/g 灰度抗阴影\r\n"
    f"  {C_GREEN}显示{C_RESET}: m 实时面板(任意键退出)  ? 查状态  h 帮助  q 退出\r\n"
    f"  {C_YELLOW}变化的数值会黄色高亮{C_RESET}\r\n"
)


# ---- 主循环 ----

def main():
    args = sys.argv[1:]
    if args and args[0] in ("-h", "--help", "help"):
        list_ports_and_exit()
    port = args[0] if args else autodetect()
    baud = int(args[1]) if len(args) > 1 else 115200
    if not port:
        print("没找到 XDS110 有线串口。")
        list_ports_and_exit()

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"打不开 {port}: {e}")

    stop = threading.Event()
    t = threading.Thread(target=reader, args=(ser, stop), daemon=True)
    t.start()
    print(f"已连接 {port}@{baud}。单键即发, h=帮助, q=退出。")

    prev_vals = {}   # 上一次 [CMD] 的 key=val 快照

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1)
            if ch == '\x03':          # Ctrl-C
                break
            if ch == 'q':
                sys.stdout.write(f"{C_RED}q{C_RESET} quit\r\n")
                sys.stdout.flush()
                break

            # 回显按键(绿色)
            sys.stdout.write(f"{C_GREEN}{ch}{C_RESET}\r\n")
            sys.stdout.flush()

            if ch == 'h':
                sys.stdout.write(HELP)
                sys.stdout.flush()
                continue
            if ch == 'm':
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                try:
                    _monitor_raw()
                finally:
                    tty.setraw(fd)
                continue
            if ch == '?':
                prev = _snapshot_csv()
                ser.write(b"?\r\n")
                _frame_event.clear()
                _frame_event.wait(timeout=0.5)
                cur = _snapshot_csv()
                sys.stdout.write(_fmt_csv_diff(prev, cur) + "\r\n")
                sys.stdout.flush()
                continue

            # 发送给固件
            if ch in RAW_CHARS:
                prev = _snapshot_csv()
                ser.write(ch.encode("ascii") + b"\r\n")
                _frame_event.clear()
                _frame_event.wait(timeout=0.5)
                cur = _snapshot_csv()
                sys.stdout.write(_fmt_csv_diff(prev, cur) + "\r\n")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        stop.set()
        t.join(timeout=0.3)
        ser.close()
        print("\r\n已断开。")


def _monitor_raw():
    """实时面板(raw 模式版): 任意键退出。"""
    import select
    sys.stdout.write("实时面板(任意键退出)...\r\n")
    sys.stdout.flush()
    height = 0
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.read(1)
                break
            board = fmt_dashboard()
            n = board.count("\r\n") + 1
            if height:
                sys.stdout.write(f"\033[{height}A")
            sys.stdout.write("\033[J" + board + "\r\n")
            sys.stdout.flush()
            height = n
            time.sleep(0.1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


if __name__ == "__main__":
    main()