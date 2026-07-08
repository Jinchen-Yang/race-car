#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""串口 PID 调参小控制台: 默认屏蔽连续 CSV, 专心发字符调参。

用法:
    python3 tools/serial_pid_console.py
    python3 tools/serial_pid_console.py /dev/tty.usbmodemXXX [115200]
    python3 tools/serial_pid_console.py /dev/tty.usbmodemXXX 115200 --show-csv

输入单字符命令后回车即可发送, 例如 ?, P, p, D, d, A, a, V, v, H, h, T, t, G, g, Y, y, O, o, r。
VOFA+ 必须关闭同一个串口, 因为 macOS/Windows 串口通常只能被一个程序占用。
"""
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("缺 pyserial: python3 -m pip install pyserial")


MARKERS = (
    "[CMD]",
    "[PID]",
    "[ERR]",
    "[DIAG]",
    "[TEST]",
    "[USER]",
    "MSPM0 boot",
    "IMU ",
)

CSV_COLUMNS = (
    "ms",
    "state",
    "gray_err",
    "lost",
    "fresh",
    "ok_delta",
    "fail_delta",
    "raw_or_neg1",
    "gray_hits",
    "line_ok",
    "gray_min_hits",
    "aim_stop_us",
    "trim",
    "arc_kp_x100",
    "kd_x100",
    "arc_scale_x1000",
    "base",
    "line_kp_x100",
    "dy",
    "seg",
    "turn_diff",
    "left_duty",
    "right_duty",
    "nodata",
    "protect_ms",
    "startprot_ms",
)
last_csv_values = None
show_csv_stream = False
csv_log_file = None
csv_log_path = None


def list_serial_ports() -> None:
    print("可用串口:")
    for p in list_ports.comports():
        print(f"  {p.device}   {p.description}")


def reader(ser: serial.Serial, stop: threading.Event) -> None:
    line = bytearray()
    last_byte_at = time.monotonic()

    while not stop.is_set():
        chunk = ser.read(256)
        now = time.monotonic()
        if not chunk:
            if line and now - last_byte_at > 0.25:
                line.clear()
            continue

        last_byte_at = now
        for b in chunk:
            if b in (10, 13):
                emit_line(line)
                line.clear()
            elif b == 9 or 32 <= b <= 126:
                line.append(b)
                if len(line) > 180:
                    line.clear()
            else:
                # JustFloat 二进制帧会大量落在这里; 丢掉即可。
                if line and now - last_byte_at > 0.25:
                    line.clear()


def emit_line(raw: bytearray) -> None:
    global last_csv_values
    if not raw:
        return
    text = raw.decode("utf-8", errors="ignore").strip()
    if not text:
        return
    if looks_like_csv_numbers(text):
        last_csv_values = text.split(",")
        write_csv_log(last_csv_values)
        if show_csv_stream:
            print(f"\n{text}\n> ", end="", flush=True)
    elif any(marker in text for marker in MARKERS):
        print(f"\n{text}\n> ", end="", flush=True)


def looks_like_csv_numbers(text: str) -> bool:
    if text.count(",") < 3:
        return False
    allowed = set("0123456789,+-. ")
    return all(ch in allowed for ch in text)


def main() -> None:
    global show_csv_stream, csv_log_file, csv_log_path
    if len(sys.argv) < 2:
        list_serial_ports()
        sys.exit("\n用法: python3 tools/serial_pid_console.py <串口> [波特率=115200] [--show-csv] [--log-csv [文件]]")

    port = sys.argv[1]
    baud = 115200
    args = sys.argv[2:]
    i = 0
    while i < len(args):
        arg = args[i]
        if arg == "--show-csv":
            show_csv_stream = True
        elif arg == "--log-csv":
            if i + 1 < len(args) and not args[i + 1].startswith("--"):
                csv_log_path = Path(args[i + 1])
                i += 1
            else:
                csv_log_path = default_log_path()
        else:
            baud = int(arg)
        i += 1

    if csv_log_path:
        csv_log_path.parent.mkdir(parents=True, exist_ok=True)
        csv_log_file = csv_log_path.open("w", encoding="utf-8", newline="")
        csv_log_file.write(",".join(CSV_COLUMNS) + "\n")
        csv_log_file.flush()

    with serial.Serial(port, baud, timeout=0.05) as ser:
        stop = threading.Event()
        t = threading.Thread(target=reader, args=(ser, stop), daemon=True)
        t.start()

        print(f"已连接 {port}@{baud}")
        print("默认已屏蔽连续 CSV 数据; 加 --show-csv 才显示原始数据流。")
        if csv_log_path:
            print(f"CSV 记录: {csv_log_path}")
        print("命令: ? 看最近状态; P/p 弯道KP; D/d KD; A/a灰度比例; V/v基速; H/h直线KP; T/t配平; G/g灰度抗阴影; Y/y水平舵机停转; O/o舵机电源; r恢复; x急停; s启动")
        print("退出: quit / exit / Ctrl-C")
        try:
            while True:
                cmd = input("> ").strip()
                if cmd in ("quit", "exit"):
                    break
                if not cmd:
                    continue
                if cmd == "?":
                    print_local_status()
                    continue
                before_ms = current_csv_ms()
                ser.write(cmd.encode("ascii", errors="ignore") + b"\r\n")
                wait_for_new_csv(before_ms)
                print_local_status(compact=True)
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
            t.join(timeout=0.2)
            if csv_log_file:
                csv_log_file.close()
                print(f"\nCSV 已保存: {csv_log_path}")


def print_local_status(compact: bool = False) -> None:
    if not last_csv_values:
        print("还没有收到 CSV 数据; 先确认端口有数字滚动。")
        return
    if compact:
        values = dict(zip(CSV_COLUMNS, last_csv_values))
        print(
            "当前: "
            f"raw={values.get('raw_or_neg1')} "
            f"hits={values.get('gray_hits')} "
            f"ok={values.get('line_ok')} "
            f"min={values.get('gray_min_hits')} "
            f"aimStop={values.get('aim_stop_us')} "
            f"fresh={values.get('fresh')} "
            f"fail={values.get('fail_delta')} "
            f"KP={values.get('arc_kp_x100')} "
            f"KD={values.get('kd_x100')} "
            f"scale={values.get('arc_scale_x1000')} "
            f"base={values.get('base')} "
            f"lineKP={values.get('line_kp_x100')} "
            f"trim={values.get('trim')} "
            f"dy={values.get('dy')} "
            f"seg={values.get('seg')} "
            f"turn={values.get('turn_diff')} "
            f"L={values.get('left_duty')} "
            f"R={values.get('right_duty')} "
            f"nodata={values.get('nodata')} "
            f"protect={values.get('protect_ms')} "
            f"startprot={values.get('startprot_ms')}"
        )
        return
    print("最近一帧:")
    for name, value in zip(CSV_COLUMNS, last_csv_values):
        print(f"  {name}: {value}")


def current_csv_ms():
    if not last_csv_values:
        return None
    return last_csv_values[0]


def wait_for_new_csv(before_ms, timeout_s: float = 0.4) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if last_csv_values and last_csv_values[0] != before_ms:
            return
        time.sleep(0.01)


def default_log_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("captures") / f"run_{stamp}.csv"


def write_csv_log(values) -> None:
    if not csv_log_file:
        return
    if len(values) != len(CSV_COLUMNS):
        return
    csv_log_file.write(",".join(values) + "\n")
    csv_log_file.flush()


if __name__ == "__main__":
    main()
