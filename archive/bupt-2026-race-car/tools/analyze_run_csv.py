#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析小车 Serial Studio CSV 记录,定位突然右转/误切段/灰度掉帧。"""
import csv
import sys
from pathlib import Path


NUMERIC_FIELDS = {
    "ms", "state", "gray_err", "lost", "fresh", "ok_delta", "fail_delta",
    "raw_or_neg1", "gray_hits", "line_ok", "gray_min_hits", "aim_stop_us", "trim",
    "arc_kp_x100", "kd_x100", "arc_scale_x1000",
    "base", "line_kp_x100", "dy", "seg", "turn_diff", "left_duty",
    "right_duty", "nodata", "protect_ms", "startprot_ms",
}


def main() -> None:
    if len(sys.argv) < 2:
        sys.exit("用法: python3 tools/analyze_run_csv.py captures/run_YYYYmmdd_HHMMSS.csv")

    path = Path(sys.argv[1])
    rows = load_rows(path)
    if not rows:
        sys.exit("没有读到有效 CSV 行")

    print(f"文件: {path}")
    print(f"样本: {len(rows)} 行, 时长约 {(rows[-1]['ms'] - rows[0]['ms']) / 1000:.2f}s")

    summarize_io(rows)
    summarize_states(rows)
    summarize_segments(rows)
    summarize_turns(rows)
    diagnose(rows)


def load_rows(path: Path):
    out = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                parsed = {}
                for k, v in row.items():
                    if k in NUMERIC_FIELDS and v not in (None, ""):
                        parsed[k] = int(float(v))
                if "ms" in parsed:
                    out.append(parsed)
            except ValueError:
                continue
    return out


def summarize_io(rows):
    fails = sum(r.get("fail_delta", 0) for r in rows)
    fresh_bad = sum(1 for r in rows if r.get("fresh") == 0)
    raw_bad = sum(1 for r in rows if r.get("raw_or_neg1") == -1)
    print("\nI2C/灰度:")
    print(f"  fail_delta 总和: {fails}")
    print(f"  fresh=0 帧数: {fresh_bad}")
    print(f"  raw=-1 帧数: {raw_bad}")
    print(f"  raw 非 255 帧数: {sum(1 for r in rows if r.get('raw_or_neg1', 255) not in (255, -1))}")
    if any("gray_hits" in r for r in rows):
        line_ok = sum(1 for r in rows if r.get("line_ok") == 1)
        shadow_like = sum(
            1 for r in rows
            if r.get("raw_or_neg1", 255) not in (255, -1, 0) and r.get("line_ok") == 0
        )
        mins = sorted({r.get("gray_min_hits") for r in rows if "gray_min_hits" in r})
        print(f"  line_ok 帧数: {line_ok}")
        print(f"  raw变黑但未通过门控帧数: {shadow_like}")
        print(f"  gray_min_hits 出现值: {mins}")
    stops = sorted({r.get("aim_stop_us") for r in rows if "aim_stop_us" in r})
    if stops:
        print(f"  aim_stop_us 出现值: {stops}")


def summarize_states(rows):
    print("\n状态段:")
    prev = None
    start = None
    for r in rows:
        st = r.get("state")
        if st != prev:
            if prev is not None:
                print_span(prev, start, last)
            prev = st
            start = r
        last = r
    if prev is not None:
        print_span(prev, start, rows[-1])


def summarize_segments(rows):
    print("\nseg 跳变:")
    prev = rows[0].get("seg")
    for r in rows[1:]:
        seg = r.get("seg")
        if seg != prev:
            print(f"  t={r['ms']}ms seg {prev}->{seg} raw={r.get('raw_or_neg1')} hits={r.get('gray_hits')} ok={r.get('line_ok')} nodata={r.get('nodata')} turn={r.get('turn_diff')} L={r.get('left_duty')} R={r.get('right_duty')}")
            prev = seg


def summarize_turns(rows):
    right = [r for r in rows if r.get("state") == 1 and r.get("turn_diff", 0) < -20]
    left = [r for r in rows if r.get("state") == 1 and r.get("turn_diff", 0) > 20]
    print("\n转向:")
    print(f"  RUN 中明显右转(turn_diff<-20): {len(right)} 帧")
    print(f"  RUN 中明显左转(turn_diff>20): {len(left)} 帧")
    for label, group in (("右转", right[:5]), ("左转", left[:5])):
        if group:
            print(f"  {label}前几帧:")
            for r in group:
                print(f"    t={r['ms']} seg={r.get('seg')} raw={r.get('raw_or_neg1')} hits={r.get('gray_hits')} ok={r.get('line_ok')} gray={r.get('gray_err')} nodata={r.get('nodata')} turn={r.get('turn_diff')} L={r.get('left_duty')} R={r.get('right_duty')}")


def diagnose(rows):
    run = [r for r in rows if r.get("state") == 1]
    print("\n初步判断:")
    if not run:
        print("  没有 RUN 段,先确认是否发了 s 启动。")
        return
    if any(r.get("fresh") == 0 or r.get("fail_delta", 0) > 0 for r in run):
        print("  RUN 期间有 I2C 掉帧/失败,优先查灰度线束/供电/上拉。")
    if any(r.get("raw_or_neg1", 255) not in (255, -1, 0) and r.get("line_ok") == 0 for r in run):
        first = next(r for r in run if r.get("raw_or_neg1", 255) not in (255, -1, 0) and r.get("line_ok") == 0)
        print(f"  有疑似阴影/弱黑帧已被门控过滤: t={first['ms']} raw={first.get('raw_or_neg1')} hits={first.get('gray_hits')} min={first.get('gray_min_hits')}; 若车仍误转,把 G 再加一档。")
    if any(r.get("seg") == 1 and r.get("nodata", 255) <= 8 for r in run):
        first = next(r for r in run if r.get("seg") == 1 and r.get("nodata", 255) <= 8)
        print(f"  seg 已切到 1: t={first['ms']} raw={first.get('raw_or_neg1')} hits={first.get('gray_hits')} ok={first.get('line_ok')} nodata={first.get('nodata')}; 若此时车还没到弯,就是直线误认线导致突然右转。")
    if any(r.get("turn_diff", 0) < -20 for r in run):
        first = next(r for r in run if r.get("turn_diff", 0) < -20)
        print(f"  控制确实下发了右转: t={first['ms']} turn={first.get('turn_diff')} L={first.get('left_duty')} R={first.get('right_duty')}")
    elif any(r.get("raw_or_neg1", 255) not in (255, -1) for r in run):
        print("  灰度看到黑线,但 turn_diff 没明显变化,看 seg/弯道进入条件或增益。")
    else:
        print("  RUN 期间 raw 基本全 255,灰度没看到线;若车应在弯上,查高度/阈值/线宽。")


def print_span(state, start, end):
    print(f"  state={state}: {start['ms']}..{end['ms']}ms ({(end['ms'] - start['ms']) / 1000:.2f}s)")


if __name__ == "__main__":
    main()
