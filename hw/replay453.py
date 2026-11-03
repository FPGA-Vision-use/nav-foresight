"""453 帧真实回放跑批（板侧）。

读 `/tmp/ref_frames.jsonl`（PC 生成的每帧参考），对每帧：
  1. 配置 rollout 的 5 个初值 → 跑 → 43×400 点写进 DDR
  2. 配置 clearance 的 20 个参数 → 跑 → 43×5 margin
  3. 把板子算出来的原始结果（215 个 margin + 被选候选的 400 点 + 43 个 viable）写一行 JSON

比较在 PC 侧做（那边有 numpy 和完整参考），板上只负责跑和记原始值，减少出错面。

用法：python3 replay453.py [起始帧] [帧数]
"""

import json
import mmap
import os
import struct
import sys
import time

sys.path.insert(0, "/tmp")
from clear_hw import DmaBuf  # noqa: E402

K, N = 43, 400

# 读 PL 输出前必须把本进程的缓存行挤掉，否则会读到上一次读留下的旧数据
_SCRATCH = bytearray(8 << 20)
_ZERO = bytes(8 << 20)


def sweep():
    _SCRATCH[:] = _ZERO
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


class Regs:
    def __init__(self, base):
        self.fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        self.m = mmap.mmap(self.fd, 0x1000, mmap.MAP_SHARED,
                           mmap.PROT_READ | mmap.PROT_WRITE, offset=base)
        self.pack = struct.pack
        self.unpack = struct.unpack

    def rd(self, off):
        return self.unpack("<I", self.m[off:off + 4])[0]

    def wr(self, off, val):
        self.m[off:off + 4] = self.pack("<I", val & 0xFFFFFFFF)


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


def main():
    start = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 453
    frames = [json.loads(l) for l in open("/tmp/ref_frames.jsonl", encoding="utf-8")]
    frames = frames[start:start + count]
    print("跑 %d 帧（%d..%d）" % (len(frames), start, start + len(frames) - 1), flush=True)

    pts = DmaBuf(PTS_BYTES)
    via = DmaBuf(64 * 4)
    mar = DmaBuf(64 * 4)
    pts.flush(); via.flush(); mar.flush()
    print("pts=0x%08x via=0x%08x mar=0x%08x" % (pts.phys, via.phys, mar.phys), flush=True)

    rc, ra = Regs(ROLL_CTRL), Regs(ROLL_ADDR)
    cc, ca = Regs(CLR_CTRL), Regs(CLR_ADDR)
    ra.wr(0x10, pts.phys & 0xFFFFFFFF); ra.wr(0x14, 0)
    ra.wr(0x1C, via.phys & 0xFFFFFFFF); ra.wr(0x20, 0)
    ca.wr(0x10, pts.phys & 0xFFFFFFFF); ca.wr(0x14, 0)
    ca.wr(0x1C, mar.phys & 0xFFFFFFFF); ca.wr(0x20, 0)

    out = open("/tmp/replay453_out.jsonl", "w", encoding="utf-8")
    ptsbin = open("/tmp/pts453.bin", "wb")
    bad = []
    t_all = time.perf_counter()
    for idx, fr in enumerate(frames):
        x, y, h, v, curv = fr["init"]
        for name, val in zip(ROLL_P, (x, y, __import__("math").cos(h),
                                      __import__("math").sin(h), v, curv)):
            rc.wr(0x10 + 8 * ROLL_P.index(name),
                  struct.unpack("<I", struct.pack("<f", val))[0])
        for name in P_ORDER:
            cc.wr(P_OFF[name], int(fr["params"][name]) & 0xFFFFFFFF)
        r1 = run(rc)
        r2 = run(cc)
        if r1 is None or r2 is None:
            bad.append((fr["frame"], "no_start/timeout"))
            continue
        sweep()                      # 挤掉缓存，保证下面读到的是 DDR 里的新数据
        mar.mm.seek(0)
        mv = struct.unpack("<215i", mar.read(860))
        ch = fr["chosen"]
        # 注意：DmaBuf.read() 内部会 seek(0)，不能先用 seek 定位——直接切 mmap
        allw = pts.mm[:PTS_BYTES]          # 全部 43 路，给渲染用
        ptsbin.write(allw)
        pw = struct.unpack("<400I", allw[ch * N * 4: ch * N * 4 + 1600])
        px = []
        for w in pw:
            px.append(((w & 0xFFFF) ^ 0x8000) - 0x8000)
            px.append((((w >> 16) & 0xFFFF) ^ 0x8000) - 0x8000)
        vv = struct.unpack("<43i", pts.mm[PTS_BYTES:PTS_BYTES + 172])
        out.write(json.dumps({"f": fr["frame"], "m": list(mv), "p": px,
                              "v": list(vv), "ms1": round(r1, 3), "ms2": round(r2, 3)}) + "\n")
        if (idx + 1) % 50 == 0:
            print("  %d/%d  %.2f ms/帧(均值)" % (idx + 1, len(frames),
                  (time.perf_counter() - t_all) * 1e3 / (idx + 1)), flush=True)
    out.close(); ptsbin.close()
    dt = (time.perf_counter() - t_all) * 1e3 / max(1, len(frames))
    print("完成 %d 帧，平均 %.1f ms/帧（含 PS 配置与回读），失败 %d"
          % (len(frames), dt, len(bad)), flush=True)
    if bad:
        print("失败帧:", bad[:10], flush=True)


if __name__ == "__main__":
    main()
