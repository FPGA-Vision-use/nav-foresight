"""453 帧验收分析（PC 侧）。

对比三样东西：
  1. 板子的 rollout 输出 vs planner 记录的被选候选路径（毫米）
  2. 板子的 43×5 margin vs 浮点参考（同一初值/参数下）
  3. 每列的最差偏差、超 20 mm 的帧数、以及候选排序一致率

判据（INTERFACE.md 第 6 节）：
  · 净空：不低于浮点版 0.02 m（20 mm）
  · 候选一致率：≥99%
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
COLS = ["boundary", "pillar", "timed", "sweep", "cloud"]


def main():
    ref = [json.loads(l) for l in open(HERE / "ref_frames.jsonl", encoding="utf-8")]
    got = [json.loads(l) for l in open(HERE / "replay453_out.jsonl", encoding="utf-8")]
    got = [g for g in got if g["f"] < len(ref)]
    by_frame = {g["f"]: g for g in got}

    devs = {c: [] for c in COLS}          # board - ref
    viol = {c: 0 for c in COLS}
    path_err = []
    argmin_same = {c: 0 for c in COLS}
    argmin_n = {c: 0 for c in COLS}
    frame_min_dev = {c: [] for c in COLS}
    per_frame_bad = []

    for fr in ref:
        g = by_frame.get(fr["frame"])
        if g is None:
            continue
        m = np.array(g["m"], dtype=np.int64).reshape(43, 5)
        ref_m = fr["margins_mm"]
        # ---- 1. 路径 ----
        p = np.array(g["p"], dtype=np.int64).reshape(400, 2)
        rp = np.array(fr["path_mm"][1:401], dtype=np.int64)
        e = np.abs(p - rp).max()
        path_err.append(int(e))
        # ---- 2. margin ----
        valid_cols = {0: True, 1: True,
                      2: bool(fr["actor_moving"]), 3: bool(fr["actor_moving"]),
                      4: bool(fr["cloud_cond"])}
        for c in range(5):
            if not valid_cols[c]:
                continue
            col_ref = np.array([np.nan if ref_m[k][c] is None else ref_m[k][c]
                                for k in range(43)], dtype=float)
            mask = np.isfinite(col_ref)
            if not mask.any():
                continue
            d = m[:, c][mask] - col_ref[mask]
            devs[COLS[c]].extend(d.tolist())
            viol[COLS[c]] += int((d < -20).sum())
            # 每列"该帧最紧的那个候选"的偏差（净空判据关心最小值）
            frame_min_dev[COLS[c]].append(float(m[:, c][mask].min() - col_ref[mask].min()))
            # 候选排序：板子与浮点各自的 argmin（最紧候选）
            if mask.sum() == 43:
                argmin_n[COLS[c]] += 1
                if int(np.argmin(m[:, c])) == int(np.argmin(col_ref)):
                    argmin_same[COLS[c]] += 1
        if e > 2 or any(v < -20 for v in [0]):
            per_frame_bad.append((fr["frame"], int(e)))

    print("=== 1. rollout 路径 vs planner 记录（被选候选，400 点）===")
    pe = np.array(path_err)
    print("  最大偏差 %d mm | p99 %d mm | 均值 %.3f mm | 完全一致 %d/%d 帧"
          % (pe.max(), np.percentile(pe, 99), pe.mean(), (pe == 0).sum(), pe.size))

    print("\n=== 2. margin：板子 - 浮点参考（同一初值/参数）===")
    print("  %-9s %7s %10s %10s %10s %10s %8s" %
          ("列", "样本", "最大(乐观)", "最小", "均值", "最大|差|", "<-20mm"))
    for c in COLS:
        d = np.array(devs[c])
        if d.size == 0:
            print("  %-9s （无有效样本）" % c)
            continue
        print("  %-9s %7d %10.1f %10.1f %10.3f %10.1f %8d"
              % (c, d.size, d.max(), d.min(), d.mean(), np.abs(d).max(), viol[c]))

    print("\n=== 3. 每帧最紧候选（净空判据关心这个）===")
    for c in COLS:
        v = np.array(frame_min_dev[c])
        if v.size == 0:
            continue
        print("  %-9s 帧数 %3d  最大(板子更乐观) %7.1f mm  平场 %.3f mm  超 20mm 的帧 %d"
              % (c, v.size, v.max(), v.mean(), int((v < -20).sum())))

    print("\n=== 4. 候选排序一致率（板子 argmin vs 浮点 argmin，逐列代理指标）===")
    for c in COLS:
        if argmin_n[c]:
            print("  %-9s %d/%d = %.1f%%"
                  % (c, argmin_same[c], argmin_n[c],
                     100.0 * argmin_same[c] / argmin_n[c]))

    print("\n=== 5. 总体 ===")
    all_dev = np.concatenate([np.array(devs[c]) for c in COLS if devs[c]])
    print("  全部样本 %d 个，最大|差| %.1f mm，超 20mm 的样本 %d 个（%.3f%%）"
          % (all_dev.size, np.abs(all_dev).max(),
             int((all_dev < -20).sum()),
             100.0 * (all_dev < -20).sum() / all_dev.size))
    print("  路径异常帧(>2mm):", per_frame_bad[:8], "共", len(per_frame_bad))

    ms1 = np.array([g["ms1"] for g in got])
    ms2 = np.array([g["ms2"] for g in got])
    print("\n=== 6. 耗时（453 帧）===")
    print("  rollout  中位 %.3f ms（min %.3f max %.3f）" % (np.median(ms1), ms1.min(), ms1.max()))
    print("  clearance 中位 %.3f ms（min %.3f max %.3f）" % (np.median(ms2), ms2.min(), ms2.max()))
    print("  PL 合计中位 %.3f ms → %.1f fps" % (np.median(ms1 + ms2), 1000 / np.median(ms1 + ms2)))


if __name__ == "__main__":
    main()
