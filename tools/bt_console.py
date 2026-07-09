#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bt_console.py —— 蓝牙(HC-05)双向调参台: 一个串口同时【解 JustFloat 遥测】+【发命令】。

一个工具搞定"即输出 + 无线调参": 后台线程实时解 24 通道 JustFloat 波形并抓 [CMD] 文本回显,
前台用友好命令名或原始单字符发指令(小车 PID + 云台 bring-up)。

依赖: pip install pyserial      (录原始流仍用 log_serial.py, 离线转 CSV 用 parse_justfloat.py)

用法:
    python3 tools/bt_console.py                      # 自动找 racecar-bt / usbserial, 115200
    python3 tools/bt_console.py /dev/tty.racecar-bt   # 指定口
    python3 tools/bt_console.py /dev/tty.racecar-bt 115200

进去后:
    help            列出所有命令
    mon             实时面板(波形数值滚动刷新), 回车退出
    ?               发'?'查状态 + 打印最近一帧遥测
    pitch+ / pitch- 俯仰(竖直位置舵机)±1°   ← 能记住角度
    stop+ / stop-   水平连续舵机停转脉宽±5us(消蠕转)
    rail on/off, center, start, estop, f1..f4
    也可直接敲固件原始单字符: O o C J j K k G g P p D d V v H h T t 1 2 3 4 s x ?
    quit            退出

⚠ 命名陷阱(本车实测): 代码/命令里 PAN=竖直俯仰位置舵机, TILT=水平方位360°连续舵机(左右反了)。
   本工具的 pitch±=J/j(竖直), stop±=G/g(水平连续舵机停转), 已按物理轴取名, 照着用就行。
"""
import sys
import struct
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("缺 pyserial: python3 -m pip install pyserial")

TAIL = b"\x00\x00\x80\x7f"          # JustFloat 帧尾 = +Inf
NCH = 24
FRAME_DATA = NCH * 4                 # 96 字节数据 + 4 字节帧尾

# 通道名与 firmware empty.c task_vofa / parse_justfloat.py 一一对应(开环 RUN 态语义)
CH_NAMES = [
    "state", "encL", "encR", "k230_age", "delta_yaw", "vel_L", "seg", "vel_R",
    "duty_L", "duty_R", "gray_err", "lost", "yaw", "gray_ok", "gray_fail",
    "gray_byte", "mode", "imu_ok", "imu_fail", "hkp", "seg_tgt_yaw",
    "turn_diff", "online_ms", "trim",
]

# 友好命令名 -> 固件单字符。按物理轴取名(见文件头命名陷阱)。
ALIASES = {
    "rail on": "O", "railon": "O", "rail off": "o", "railoff": "o",
    "center": "C", "mid": "C",
    "pitch+": "J", "pitch-": "j", "elev+": "J", "elev-": "j",   # 竖直俯仰位置舵机 ±1°
    "stop+": "G", "stop-": "g",                                  # 水平连续舵机停转脉宽 ±5us
    "start": "s", "go": "s", "estop": "x", "halt": "x",
    "f1": "1", "f2": "2", "f3": "3", "f4": "4", "aim": "2",
    "kp+": "P", "kp-": "p", "kd+": "D", "kd-": "d",
    "v+": "V", "v-": "v", "hkp+": "H", "hkp-": "h", "trim+": "T", "trim-": "t",
}
RAW_CHARS = set("1234sxPpDdVvHhTtOoCJjKkGg?")   # 可直接敲的固件单字符

# ---- 共享状态(reader 线程写, 前台读) ----
_lock = threading.Lock()
_latest = None            # 最近一帧 24 float
_frames = 0
_servo_line = ""          # 最近一条含 SERVO= 的 [CMD] 回显


def autodetect():
    prefer = ("racecar", "hc-05", "hc05")
    ports = list(list_ports.comports())
    for p in ports:
        if any(k in p.device.lower() for k in prefer):
            return p.device
    for p in ports:
        if "usbserial" in p.device.lower() or "usbmodem" in p.device.lower():
            return p.device
    return None


def list_ports_and_exit():
    print("可用串口:")
    for p in list_ports.comports():
        print(f"  {p.device}   {p.description}")
    sys.exit("\n用法: python3 tools/bt_console.py <串口> [波特率=115200]")


def reader(ser, stop):
    """后台: 同一字节流喂两个独立解析器 —— JustFloat 帧 + [CMD] 文本行。"""
    global _latest, _frames, _servo_line
    fbuf = bytearray()        # 帧解析缓冲(帧尾对齐)
    tline = bytearray()       # 文本行缓冲(可打印 ASCII, \n 结束)
    while not stop.is_set():
        chunk = ser.read(512)
        if not chunk:
            continue
        # --- JustFloat 帧: 帧尾分帧, 帧尾前恰好 96 字节=一帧, 否则丢弃重同步 ---
        fbuf += chunk
        while True:
            idx = fbuf.find(TAIL)
            if idx < 0:
                if len(fbuf) > 3:
                    del fbuf[:-3]     # 只留可能被截断的半个帧尾
                break
            if idx == FRAME_DATA:
                vals = struct.unpack_from("<%df" % NCH, fbuf, 0)
                with _lock:
                    _latest = vals
                    _frames += 1
            del fbuf[:idx + 4]        # 消费掉这个帧尾, 缓冲回到帧边界
        # --- 文本行: 抓 [CMD]/boot 回显(二进制字节会打断半行, 自动丢弃) ---
        for b in chunk:
            if b in (10, 13):
                if tline:
                    _emit_text(tline.decode("utf-8", "ignore").strip())
                    tline.clear()
            elif 32 <= b <= 126:
                tline.append(b)
                if len(tline) > 200:
                    tline.clear()
            else:
                tline.clear()


def _emit_text(text):
    global _servo_line
    if not text:
        return
    if "[CMD]" in text or "boot" in text or "cont stop" in text:
        print(f"\r\033[K{text}\n> ", end="", flush=True)
        if "SERVO=" in text:
            with _lock:
                _servo_line = text


def fmt_dashboard():
    with _lock:
        v = _latest
        frames = _frames
        servo = _servo_line
    if v is None:
        return "  (还没解出遥测帧: 确认车在发波形 / HC-05 数据口 115200)"
    d = dict(zip(CH_NAMES, v))
    st = int(d["state"]); mode = int(d["mode"]); seg = int(d["seg"])
    stname = {0: "IDLE", 1: "RUN", 2: "AIM", 3: "STOP", 4: "ESTOP"}.get(st, "?")
    lines = [
        f"  帧#{frames}  state={stname}  mode=F{mode}  seg={seg}",
        f"  [掉头] dYaw={d['delta_yaw']:+7.1f}  turn_diff={d['turn_diff']:+6.0f}  "
        f"dutyL={d['duty_L']:+6.0f}  dutyR={d['duty_R']:+6.0f}",
        f"  [循迹] gray_err={d['gray_err']:+6.0f}  lost={int(d['lost'])}  yaw={d['yaw']:+7.1f}",
        f"  [编码] vel_L={d['vel_L']:+6.0f}  vel_R={d['vel_R']:+6.0f} mm/s   trim={int(d['trim'])}",
    ]
    if servo:
        s = servo.split("SERVO=", 1)[1]
        lines.append(f"  [云台] SERVO={s}")
    return "\n".join(lines)


def monitor(stop_key):
    """实时面板: ANSI 上移重绘, 回车退出。"""
    import select
    print("实时面板(回车退出)...\n")
    height = 0
    while True:
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            break
        board = fmt_dashboard()
        n = board.count("\n") + 1
        if height:
            sys.stdout.write(f"\033[{height}A")   # 上移到面板顶
        sys.stdout.write("\033[J" + board + "\n")  # 清到屏末再画
        sys.stdout.flush()
        height = n
        time.sleep(0.1)


HELP = """命令:
  友好名(推荐):
    rail on / rail off   舵机轨上/下电(先上电再调云台)
    center               云台 PAN/TILT 归中位(90°)
    pitch+ / pitch-      竖直俯仰位置舵机 ±1°(能记住角度; =固件 J/j)
    stop+  / stop-       水平连续舵机停转脉宽 ±5us(消蠕转; =固件 G/g)
    start / estop        启动 / 急停
    f1 f2 f3 f4          切模式(f2=定点瞄准)
    kp+/kp- kd+/kd- v+/v- hkp+/hkp- trim+/trim-   小车 PID 在线调
  原始单字符(直接敲): O o C J j K k G g  P p D d V v H h T t  1 2 3 4 s x ?
  mon    实时波形面板(回车退出)
  ?      查状态(含 SERVO/PAN/TILT) + 打印最近一帧
  help   本帮助      quit/exit  退出
⚠ 命名反了: PAN=竖直俯仰, TILT=水平方位连续舵机。pitch±/stop± 已按物理轴取名。"""


def main():
    args = sys.argv[1:]
    if args and args[0] in ("-h", "--help", "help"):
        list_ports_and_exit()
    port = args[0] if args else autodetect()
    baud = int(args[1]) if len(args) > 1 else 115200
    if not port:
        print("没找到 racecar-bt / usbserial 口。")
        list_ports_and_exit()

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"打不开 {port}: {e}\n(VOFA+ 或 log_serial.py 若开着同一个口, 先关掉——串口独占)")

    stop = threading.Event()
    t = threading.Thread(target=reader, args=(ser, stop), daemon=True)
    t.start()
    print(f"已连接 {port}@{baud}。输入 help 看命令, mon 看实时面板, quit 退出。")
    print("⚠ 真机验证前别发 f2/2 (云台修复未烧录前进 F2 仍会狂转)。")
    try:
        while True:
            try:
                cmd = input("> ").strip()
            except EOFError:
                break
            if not cmd:
                continue
            low = cmd.lower()
            if low in ("quit", "exit"):
                break
            if low == "help":
                print(HELP)
                continue
            if low == "mon":
                monitor(stop)
                continue
            if low == "?":
                ser.write(b"?\r\n")
                time.sleep(0.15)
                print(fmt_dashboard())
                continue
            ch = ALIASES.get(low)
            if ch is None and len(cmd) == 1 and cmd in RAW_CHARS:
                ch = cmd
            if ch is None:
                print(f"未知命令 '{cmd}' —— 输入 help 看可用命令")
                continue
            ser.write(ch.encode("ascii") + b"\r\n")   # 固件按单字符处理, \r\n 被忽略
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        t.join(timeout=0.3)
        ser.close()
        print("\n已断开。")


if __name__ == "__main__":
    main()
