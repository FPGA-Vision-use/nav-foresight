"""一帧完整 PL 流水：rollout(R2) → clearance(R1)，共享同一块 DDR 缓冲。

在板子上跑（集成 bitstream）：
    python3 int_chain.py chain ref.txt        # 跑一帧，与串联参考逐位对拍
    python3 int_chain.py bench ref.txt 20     # 计时

集成 bitstream 的寄存器基址（build_bd_int.tcl）：
    rollout     控制 0x80000000 / 地址 0x80010000
    clearance   控制 0x80010000 / 地址 0x80020000

数据流：
    PS 只写寄存器（5 个初值 + 5 个参数 + 地址）
      → rollout 把 43×400 点写进 pts 缓冲（DDR）
      → clearance 就着同一块缓冲算 43×5 margin（DDR）
      → PS 读 215 个 margin
    两次 ap_idle 握手，中间没有任何 PS 参与数据搬运。

判完成只看 ap_idle（ap_done 是单周期脉冲，见 hw/README.md）。
"""

import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clear_hw import DmaBuf, P_OFF as CLR_P_OFF, DEFAULT_P as CLR_DEFAULT  # noqa: E402

# 基址默认值对应 build_bd_int.tcl 的 assign_bd_address 结果；host.py 会从 hwh 里读出真值再覆盖
ROLL_CTRL = int(os.environ.get("ROLL_CTRL_BASE", "0x80000000"), 0)
ROLL_ADDR = int(os.environ.get("ROLL_ADDR_BASE", "0x80010000"), 0)
CLR_CTRL = int(os.environ.get("CLEAR_CTRL_BASE", "0x80020000"), 0)
CLR_ADDR = int(os.environ.get("CLEAR_ADDR_BASE", "0x80030000"), 0)

ROLL_P = {"x0": 0x10, "y0": 0x18, "ch0": 0x20, "sh0": 0x28, "v0": 0x30, "k0": 0x38}
ROLL_INIT = {"x0": 1.106652, "y0": -1.672124,
             "ch0": -0.031611406, "sh0": 0.999500235, "v0": 0.0, "k0": 0.0}

K, N = 43, 400
PTS_BYTES = K * N * 4
CTRL = 0x00


class Regs:
    def __init__(self, base):
        import mmap
        self.fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        self.m = mmap.mmap(self.fd, 0x1000, mmap.MAP_SHARED,
                           mmap.PROT_READ | mmap.PROT_WRITE, offset=base)

    def rd(self, off):
        return struct.unpack("<I", self.m[off:off + 4])[0]

    def wr(self, off, val):
        self.m[off:off + 4] = struct.pack("<I", val & 0xFFFFFFFF)
        return self.rd(off)

    def close(self):
        self.m.close()
        os.close(self.fd)


def run_kernel(regs, ctrl_off=CTRL, timeout_s=8.0):
    """写 start，等 ap_idle 掉 0 再回 1。"""
    t0 = time.perf_counter()
    regs.wr(ctrl_off, 0)
    regs.wr(ctrl_off, 1)
    started = False
    deadline = t0 + timeout_s
    while time.perf_counter() < deadline:
        if not (regs.rd(ctrl_off) & 4):
            started = True
            break
    if not started:
        return {"ok": False, "why": "no_start", "ctrl": regs.rd(ctrl_off), "ms": 0.0}
    while time.perf_counter() < deadline:
        if regs.rd(ctrl_off) & 4:
            return {"ok": True, "ctrl": regs.rd(ctrl_off),
                    "ms": (time.perf_counter() - t0) * 1e3}
    return {"ok": False, "why": "timeout", "ctrl": regs.rd(ctrl_off),
            "ms": (time.perf_counter() - t0) * 1e3}


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "chain"
    ref = sys.argv[2] if len(sys.argv) > 2 else "chain_ref.txt"
    exp = [[int(x) for x in l.split()] for l in open(ref) if l.strip()]

    pts = DmaBuf(PTS_BYTES)
    via = DmaBuf(64 * 4)
    mar = DmaBuf(64 * 4)
    pts.flush(); via.flush(); mar.flush()     # 把 __init__ 触碰的脏行写回，别盖掉 PL 输出
    print("pts=0x%08x viable=0x%08x margin=0x%08x" % (pts.phys, via.phys, mar.phys))
    print("基址 roll ctrl=%#x addr=%#x | clr ctrl=%#x addr=%#x"
          % (ROLL_CTRL, ROLL_ADDR, CLR_CTRL, CLR_ADDR))

    rc, ra = Regs(ROLL_CTRL), Regs(ROLL_ADDR)
    cc, ca = Regs(CLR_CTRL), Regs(CLR_ADDR)

    # ---- rollout 配置 ----
    for name, off in ROLL_P.items():
        bits = struct.unpack("<I", struct.pack("<f", ROLL_INIT[name]))[0]
        back = rc.wr(off, bits)          # 参数在控制段，地址在地址段
        if back != bits:
            print("FAIL rollout 参数 %s@%02x 写 %08x 读回 %08x" % (name, off, bits, back)); return
    ra.wr(0x10, pts.phys & 0xFFFFFFFF); ra.wr(0x14, (pts.phys >> 32) & 0xFFFFFFFF)
    ra.wr(0x1C, via.phys & 0xFFFFFFFF); ra.wr(0x20, (via.phys >> 32) & 0xFFFFFFFF)

    # ---- clearance 配置 ----
    for name, off in CLR_P_OFF.items():
        v = int(CLR_DEFAULT[name])
        back = cc.wr(off, v)
        if back != (v & 0xFFFFFFFF):
            print("FAIL clearance 参数 %s@%02x 写 %08x 读回 %08x" % (name, off, v, back)); return
    ca.wr(0x10, pts.phys & 0xFFFFFFFF); ca.wr(0x14, (pts.phys >> 32) & 0xFFFFFFFF)
    ca.wr(0x1C, mar.phys & 0xFFFFFFFF); ca.wr(0x20, (mar.phys >> 32) & 0xFFFFFFFF)

    if cmd == "chain":
        r1 = run_kernel(rc)
        print("rollout :", r1)
        if not r1["ok"]:
            print("RESULT FAIL"); return
        r2 = run_kernel(cc)
        print("clearance:", r2)
        if not r2["ok"]:
            print("RESULT FAIL"); return
        mar.mm.seek(0)
        v = struct.unpack("<%di" % (K * 5), mar.read(K * 5 * 4))
        got = [list(v[i * 5:i * 5 + 5]) for i in range(K)]
        bad = 0
        worst = 0
        for i in range(K):
            for c in range(5):
                d = abs(got[i][c] - exp[i][c])
                if d:
                    bad += 1
                    worst = max(worst, d)
                    if bad <= 6:
                        print("  MISMATCH cand%d col%d got %d exp %d"
                              % (i, c, got[i][c], exp[i][c]))
        print("与串联参考不一致: %d/%d，最大偏差 %d mm" % (bad, K * 5, worst))
        print("cand0 got:", got[0], "exp:", exp[0])
        # rollout v1.7 起，可行标志写在 pts 尾部（byte 68800，43 个 int32）
        print("viable:", list(struct.unpack("<%di" % K, pts.mm[PTS_BYTES:PTS_BYTES + K * 4]))[:8], "...")
        print("一帧 PL: rollout %.3f ms + clearance %.3f ms = %.3f ms → %.1f fps"
              % (r1["ms"], r2["ms"], r1["ms"] + r2["ms"],
                 1000.0 / (r1["ms"] + r2["ms"])))
        print("RESULT", "PASS (逐位一致)" if bad == 0 else "FAIL")
        return

    if cmd == "bench":
        n = int(sys.argv[3]) if len(sys.argv) > 3 else 20
        ts = []
        for _ in range(n):
            r1 = run_kernel(rc)
            r2 = run_kernel(cc)
            if not (r1["ok"] and r2["ok"]):
                print("run failed", r1, r2); break
            ts.append((r1["ms"], r2["ms"]))
        if ts:
            tot = sorted(a + b for a, b in ts)
            print("runs=%d  一帧 PL: min=%.3f med=%.3f max=%.3f ms  → %.1f fps"
                  % (len(ts), tot[0], tot[len(tot) // 2], tot[-1],
                     1000.0 / tot[len(tot) // 2]))
            print("  rollout 中位 %.3f ms / clearance 中位 %.3f ms"
                  % (sorted(a for a, _ in ts)[len(ts) // 2],
                     sorted(b for _, b in ts)[len(ts) // 2]))
            print("RESULT PASS")
        return

    print("unknown cmd", cmd)


if __name__ == "__main__":
    main()
