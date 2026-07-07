#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JustFloat 录制文件 → CSV 解析器（离线复盘用，纯标准库）。

用法:
    python3 tools/parse_justfloat.py capture.bin              # 默认 24 通道 → capture.csv
    python3 tools/parse_justfloat.py capture.bin --channels 25  # 固件加了 tick_ms 通道后用 25

输入 = 串口原始字节流(.bin)：来源可以是 tools/log_serial.py 录的、
OpenLog SD 卡上的 LOG 文件、或 VOFA+ "保存原始数据" 导出的文件。
帧格式 = N × float32(LE) + 帧尾 00 00 80 7F。
命令台 '?' 的文本回显会混在流里(README §4)，解析器靠帧尾定位自动跳过。

通道表与 empty.c task_vofa (r31) 一一对应，改通道后同步改 CH_NAMES。
"""
import argparse
import struct
import sys
from pathlib import Path

TAIL = b"\x00\x00\x80\x7f"          # JustFloat 帧尾 = float 的 +Inf，正常通道值不会撞上

# 通道名按 r31 RUN 态语义(app.c:548-556 重用了速度环时代的遥测变量;
# empty.c task_vofa 的行内注释仍是旧语义, 以此表和 README §4 为准)
CH_NAMES = [
    "state",           # ch0  状态机 0=IDLE 1=RUN 2=AIM 3=STOP 4=ESTOP
    "encL",            # ch1  左轮累计计数(QEI 4x)
    "encR",            # ch2  右轮累计计数(GPIO 1x)
    "k230_age_ms",     # ch3  距上帧 K230 数据毫秒数
    "delta_yaw_deg",   # ch4  DeltaYaw(°, 统一航向误差, 左正) —— RUN 态; IDLE=0
    "vel_meas_L",      # ch5  左轮实测速度 mm/s(低通后, 纯遥测)
    "seg",             # ch6  当前段号 0..3 —— RUN 态
    "vel_meas_R",      # ch7  右轮实测速度 mm/s(纯遥测)
    "duty_L",          # ch8  左轮实发占空(-1000..1000)
    "duty_R",          # ch9  右轮实发占空(含 trim)
    "gray_err",        # ch10 灰度加权质心偏差(±350, 左负右正, 仅遥测)
    "line_lost",       # ch11 丢线标志
    "yaw_deg",         # ch12 航向角(°, 连续累计, 左转+)
    "gray_i2c_ok",     # ch13 灰度 I2C 累计成功
    "gray_i2c_fail",   # ch14 灰度 I2C 累计失败 —— 独涨=电平/接触/供电问题
    "gray_byte",       # ch15 灰度最近原始字节(白地=255)
    "mode",            # ch16 模式 1=F1 2=F2 3=F3 4=四圈
    "imu_ok",          # ch17 IMU 累计成功(健康≈+100/s)
    "imu_fail",        # ch18 IMU 累计失败
    "heading_kp",      # ch19 盲走航向锁 KP
    "seg_target_yaw",  # ch20 当前段目标航向(°) —— RUN 态
    "turn_diff",       # ch21 差速(限幅后, 占空) —— RUN 态
    "online_ms",       # ch22 连续压线时长 ms
    "trim_R",          # ch23 右轮占空配平
    "tick_ms",         # ch24 (可选)固件 g_tick_ms —— 加上后可离线量化调度抖动
]


def parse(raw: bytes, n_ch: int):
    """扫描帧尾定位帧，长度不符的段按垃圾(命令台回显/半帧)丢弃。"""
    frame_len = 4 * n_ch
    frames, garbage = [], 0
    pos, prev_end = 0, 0
    while True:
        idx = raw.find(TAIL, pos)
        if idx < 0:
            garbage += len(raw) - prev_end
            break
        if idx - prev_end == frame_len:
            frames.append(struct.unpack_from(f"<{n_ch}f", raw, prev_end))
        else:
            garbage += max(0, idx - prev_end)
        prev_end = idx + 4
        pos = prev_end
    return frames, garbage


def main():
    ap = argparse.ArgumentParser(description="JustFloat .bin → .csv")
    ap.add_argument("input", type=Path)
    ap.add_argument("--channels", type=int, default=24)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    raw = args.input.read_bytes()
    frames, garbage = parse(raw, args.channels)
    if not frames:
        sys.exit(f"没解析出任何 {args.channels} 通道帧：检查 --channels 是否与固件一致、文件是否为原始字节流")

    names = (CH_NAMES + [f"ch{i}" for i in range(len(CH_NAMES), args.channels)])[: args.channels]
    out = args.out or args.input.with_suffix(".csv")
    with out.open("w", encoding="utf-8") as f:
        f.write("frame," + ",".join(names) + "\n")
        for i, fr in enumerate(frames):
            f.write(f"{i}," + ",".join(f"{v:.6g}" for v in fr) + "\n")

    # —— 复盘速览：跑完先看这几行，再决定把 CSV 丢给谁 ——
    col = {n: j for j, n in enumerate(names)}
    # 帧周期 = task_vofa 周期(r31=20ms, r32 起=50ms)；有 tick_ms 通道时用真实时间轴
    if "tick_ms" in col:
        span = (frames[-1][col["tick_ms"]] - frames[0][col["tick_ms"]]) / 1000.0
        print(f"帧数 {len(frames)}（tick 实测 ≈ {span:.1f}s）  垃圾字节 {garbage}  → {out}")
    else:
        print(f"帧数 {len(frames)}（时长=帧数×task_vofa 周期, r31=20ms/r32=50ms → "
              f"{len(frames)*0.02:.1f} 或 {len(frames)*0.05:.1f}s）  垃圾字节 {garbage}  → {out}")

    def delta(name):
        return frames[-1][col[name]] - frames[0][col[name]] if name in col else float("nan")

    if "gray_i2c_fail" in col:
        print(f"灰度 I2C: ok +{delta('gray_i2c_ok'):.0f} / fail +{delta('gray_i2c_fail'):.0f}"
              f"   IMU: ok +{delta('imu_ok'):.0f} / fail +{delta('imu_fail'):.0f}")
    if "yaw_deg" in col:
        print(f"yaw 净变化 {delta('yaw_deg'):+.1f}°（单圈应 ≈±360）")
    if "tick_ms" in col:  # 固件加了 ch24 才有：直接量化调度抖动/丢拍
        ticks = [fr[col["tick_ms"]] for fr in frames]
        dts = [b - a for a, b in zip(ticks, ticks[1:]) if 0 < b - a < 1000]
        if dts:
            nominal = sorted(dts)[len(dts) // 2]          # 名义周期取中位数(20/50ms 自适应)
            late = sum(1 for d in dts if d > nominal * 1.25)
            worst = sorted(dts)[-5:]
            print(f"帧间隔: 名义 {nominal:.0f}ms  均值 {sum(dts)/len(dts):.1f}ms  "
                  f"迟到(>1.25×名义) {late} 次  最差 5 拍 {worst}")


if __name__ == "__main__":
    main()
