#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""串口原始流录制器：XDS110 / HC-05 蓝牙串口 → capture_*.bin，Ctrl-C 停止。

依赖: pip install pyserial
用法:
    python3 tools/log_serial.py                    # 列出可用串口
    python3 tools/log_serial.py /dev/tty.usbmodemXXX   # 开录(115200)

注意: 录制期间 VOFA+ 必须关掉该串口(独占)；录完用 parse_justfloat.py 转 CSV。
"""
import sys
import time
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("缺 pyserial: pip install pyserial")


def main():
    if len(sys.argv) < 2:
        print("可用串口:")
        for p in list_ports.comports():
            print(f"  {p.device}   {p.description}")
        sys.exit("\n用法: python3 tools/log_serial.py <串口> [波特率=115200]")

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    out = f"capture_{datetime.now():%Y%m%d_%H%M%S}.bin"

    with serial.Serial(port, baud, timeout=0.2) as ser, open(out, "wb") as f:
        print(f"录制中 {port}@{baud} → {out}   (Ctrl-C 停止)")
        total, t0 = 0, time.monotonic()
        try:
            while True:
                chunk = ser.read(4096)
                if chunk:
                    f.write(chunk)
                    total += len(chunk)
                if time.monotonic() - t0 > 2:
                    t0 = time.monotonic()
                    print(f"\r{total/1024:.1f} KB", end="", flush=True)
        except KeyboardInterrupt:
            pass
    # 24ch JustFloat = 100B/帧 @50Hz = 5KB/s；30s 一趟 ≈ 150KB
    print(f"\n完成: {out}  {total/1024:.1f} KB ≈ {total/100*0.02:.1f}s 数据")


if __name__ == "__main__":
    main()
