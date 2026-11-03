"""PS 主循环原型（板侧）：一帧内完成 配置 → rollout → clearance → 决策 → 记录。

这是整条系统的软件骨架：PL 只算数值，PS 负责参数、握手、拦停门槛、打分、去抖。
决策逻辑与 `replay/ps_decision.py` 一致（含两道"核里表达不了"的退化模型门）。

用法（板子上）：
    python3 main_loop.py [起始帧] [帧数]
输出：
    /tmp/main_loop_log.jsonl   每帧一行：frame / chosen / stop / margins / ms
"""
import json
import math
import mmap

import numpy as np
import os
import struct
import sys
import time

sys.path.insert(0, "/tmp")
from clear_hw import DmaBuf  # noqa: E402

K, N = 43, 400
PTS_BYTES = K * N * 4
ROLL_CTRL = int(os.environ.get("ROLL_CTRL_BASE", "0x80020000"), 0)
ROLL_ADDR = int(os.environ.get("ROLL_ADDR_BASE", "0x80030000"), 0)
CLR_CTRL = int(os.environ.get("CLEAR_CTRL_BASE", "0x80000000"), 0)
CLR_ADDR = int(os.environ.get("CLEAR_ADDR_BASE", "0x80010000"), 0)
ROLL_P = ["x0", "y0", "ch0", "sh0", "v0", "k0"]
P_ORDER = ["xmin", "ymin", "xmax", "ymax", "pcx", "pcy", "psx", "psy",
           "acx", "acy", "avx", "avy", "abase", "ccx", "ccy", "cvx", "cvy",
           "crad", "margin_mm", "sw_guard"]
P_OFF = {n: 0x10 + 8 * i for i, n in enumerate(P_ORDER)}

# 决策参数（与 settings.json 对齐）
MARGIN_M = 0.23
PERSON_R, VEH_R, EXTRA = 0.25, 0.15, 0.08
CAP, HORIZON, STATIC_GUARD = 1.5, 1.0, 0.35
STRAIGHT_CLEAR, CPW = 0.5, 0.3
ONSET, HARD_MIN = 0, 2
GOAL = (1.0, 1.7)

_SCRATCH = bytearray(2 << 20)      # 2 MB > L2(1 MB)，足以把本进程的脏/旧行挤出去
_ZERO = bytes(2 << 20)


def sweep():
    _SCRATCH[:] = _ZERO


class Regs:
    def __init__(self, base):
        self.fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        self.m = mmap.mmap(self.fd, 0x1000, mmap.MAP_SHARED,
                           mmap.PROT_READ | mmap.PROT_WRITE, offset=base)

    def rd(self, off):
        return struct.unpack("<I", self.m[off:off + 4])[0]

    def wr(self, off, val):
        self.m[off:off + 4] = struct.pack("<I", val & 0xFFFFFFFF)


def run(regs, timeout_s=8.0):
    t0 = time.perf_counter()
    regs.wr(0, 0)
    regs.wr(0, 1)
    deadline = t0 + timeout_s
    while time.perf_counter() < deadline:
        if not (regs.rd(0) & 4):
            break
    else:
        return None
    while time.perf_counter() < deadline:
        if regs.rd(0) & 4:
            return (time.perf_counter() - t0) * 1e3
    return None


def controls_bank():
    o = [(0.0, 0.0)]
    for peak in [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5]:
        for style in ["bend", "lane"]:
            for sp in [0.15, 0.3]:
                o.append((peak, sp))
            o.append((peak, 0.3))
    return o


def main():
    start = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 453
    frames = [json.loads(l) for l in open("/tmp/ref_frames.jsonl", encoding="utf-8")][start:start + count]
    opts = controls_bank()
    peak = [o[0] for o in opts]
    tgt = [o[1] for o in opts]

    pts = DmaBuf(PTS_BYTES)
    mar = DmaBuf(64 * 4)
    pts.flush(); mar.flush()
    rc, ra = Regs(ROLL_CTRL), Regs(ROLL_ADDR)
    cc, ca = Regs(CLR_CTRL), Regs(CLR_ADDR)
    ra.wr(0x10, pts.phys & 0xFFFFFFFF); ra.wr(0x14, 0); ra.wr(0x1C, 0); ra.wr(0x20, 0)
    ca.wr(0x10, pts.phys & 0xFFFFFFFF); ca.wr(0x14, 0)
    ca.wr(0x1C, mar.phys & 0xFFFFFFFF); ca.wr(0x20, 0)
    print("pts=0x%08x mar=0x%08x  帧 %d..%d" % (pts.phys, mar.phys, start, start + len(frames) - 1),
          flush=True)

    log = open("/tmp/main_loop_log.jsonl", "w", encoding="utf-8")
    prev_trace = None
    had_raw = False
    stop_run = 0
    hard_run = 0
    t_all = time.perf_counter()
    for idx, fr in enumerate(frames):
        t0 = time.perf_counter()
        rp = fr["params"]
        x, y, h, v, curv = fr["init"]
        for i, val in enumerate((x, y, math.cos(h), math.sin(h), v, curv)):
            rc.wr(0x10 + 8 * i, struct.unpack("<I", struct.pack("<f", val))[0])
        for name in P_ORDER:
            cc.wr(P_OFF[name], int(rp[name]) & 0xFFFFFFFF)
        ms1 = run(rc)
        ms2 = run(cc)
        if ms1 is None or ms2 is None:
            continue
        t_sweep0 = time.perf_counter()
        sweep()
        t_sweep1 = time.perf_counter()
        mv = struct.unpack("<215i", mar.read(860))
        # 读回全部 43 路轨迹，算有效集与候选代价
        t_rd0 = time.perf_counter()
        raw = pts.mm[:PTS_BYTES]
        t_rd1 = time.perf_counter()
        w = np.frombuffer(raw, dtype=np.uint32).reshape(K, N)
        # rollout v1.7 起，43 个可行标志写在 pts 尾部（byte 68800，布局＝43 个 int32）
        fl = np.frombuffer(pts.mm[PTS_BYTES:PTS_BYTES + K * 4], dtype=np.int32)
        px = (((w & 0xFFFF).astype(np.int32) ^ 0x8000) - 0x8000).astype(np.float32) * 0.001
        py = ((((w >> 16) & 0xFFFF).astype(np.int32) ^ 0x8000) - 0x8000).astype(np.float32) * 0.001
        del w
        m = np.array(mv, dtype=float).reshape(K, 5)
        valid = (m > 0.0).all(axis=1) & (fl != 0)
        # ---- 退化模型门（核里表达不了的）由 PS 侧补 ----
        # (a) actor 无拟合速度：浮点用 base + cap*t 的膨胀模型
        if fr["actor_valid"] and not fr["actor_moving"]:
            apx, apy = rp["acx"] * 0.001, rp["acy"] * 0.001
            tg = np.arange(N, dtype=np.float32) * np.float32(0.02)
            rad = np.float32(rp["abase"] * 0.001) + np.float32(CAP) * tg
            d2 = (px - np.float32(apx)) ** 2 + (py - np.float32(apy)) ** 2
            valid &= (d2 - rad[None, :] ** 2).min(axis=1) > 0          # 平方比较，省 sqrt
        # (b) 云：条件式由核的第 5 列覆盖；静态残余要补 1.5 m 储备圆盘 + full-reach
        if fr["cloud_valid"]:
            rm = float(fr["radius_m"] or 0.0)
            age = fr["age_s"]
            stale = age is not None and age > 2.0
            if stale:
                rm = min(rm, 0.8)
            ccx, ccy = rp["ccx"] * 0.001, rp["ccy"] * 0.001
            d2c = (px - np.float32(ccx)) ** 2 + (py - np.float32(ccy)) ** 2
            dmin2 = d2c.min(axis=1)
            if fr["cloud_cond"]:
                unc = rp["abase"] * 0.001 - PERSON_R - MARGIN_M
                rad_fr = np.float32(unc + MARGIN_M + PERSON_R + 0.0051)
            else:
                reserve = 0.0 if stale else CAP * HORIZON
                rad_fr = np.float32(rm + MARGIN_M + 0.0051)
                rad_disc = np.float32(rm + reserve + MARGIN_M + 0.0051)
                valid &= dmin2 > rad_disc ** 2
            valid &= dmin2 > rad_fr ** 2
        # (c) unknown_at_start：无 actor 支持又不是真正空场景 → 全否
        empty_scene = ((not fr["actor_valid"]) and (not fr["cloud_valid"])
                       and fr["cell_n"] == 0 and (not had_raw))
        supported = bool(fr["motion_valid"]) and bool(fr["observed"])
        unknown = (((not fr["actor_valid"]) or (not supported)) and (not empty_scene))             or (not fr["observed"])
        if unknown:
            valid[:] = False
        # 直行优先
        straight = [k for k in range(1, K) if abs(peak[k]) < 1e-9 and tgt[k] > 0 and valid[k]]
        roomy = [k for k in straight if m[k].min() >= STRAIGHT_CLEAR * 1000.0]
        dist = np.sqrt((px[:, -1] - np.float32(GOAL[0])) ** 2 + (py[:, -1] - np.float32(GOAL[1])) ** 2)
        dxp = np.diff(px, axis=1); dyp = np.diff(py, axis=1)
        length = np.sqrt(dxp * dxp + dyp * dyp).sum(axis=1)
        score = dist + np.float32(0.03) * length + np.float32(CPW) * np.abs(np.array(peak, dtype=np.float32))
        if prev_trace is not None:
            prx, pry = prev_trace
            dc = np.sqrt((px - prx) ** 2 + (py - pry) ** 2).mean(axis=1)
            score = score + np.float32(0.5) * dc
        score = np.where(valid, score, np.inf)
        chosen = (max(roomy, key=lambda k: tgt[k]) if roomy
                  else (int(np.argmin(score)) if valid.any() else 0))
        had_raw = had_raw or bool(fr["cloud_valid"])
        raw_stop = not (valid.any() and tgt[chosen] > 0)
        stop_run = stop_run + 1 if raw_stop else 0
        pending = raw_stop and stop_run <= ONSET
        moving = bool(pending or (valid[chosen] and tgt[chosen] > 0))
        prev_trace = (px[chosen], py[chosen])
        t_np1 = time.perf_counter()
        ms_ps = (time.perf_counter() - t0) * 1e3
        log.write(json.dumps({"f": fr["frame"], "chosen": chosen, "stop": (not moving),
                              "m": [int(m[chosen][c]) for c in range(5)],
                              "valid_n": int(valid.sum()),
                              "ms1": round(ms1, 3), "ms2": round(ms2, 3),
                              "ms_total": round(ms_ps, 3),
                              "t_sweep": round((t_sweep1 - t_sweep0) * 1e3, 3),
                              "t_read": round((t_rd1 - t_rd0) * 1e3, 3),
                              "t_np": round((t_np1 - t_rd1) * 1e3, 3)}) + chr(10))
        if (idx + 1) % 60 == 0:
            print("  %d/%d  平均 %.1f ms/帧（含 PS 全部开销）"
                  % (idx + 1, len(frames), (time.perf_counter() - t_all) * 1e3 / (idx + 1)), flush=True)
    log.close()
    dt = (time.perf_counter() - t_all) * 1e3 / max(1, len(frames))
    print("完成 %d 帧：**端到端 %.1f ms/帧（%.1f fps，含 PS 配置/回读/决策）**"
          % (len(frames), dt, 1000.0 / dt), flush=True)


if __name__ == "__main__":
    main()
