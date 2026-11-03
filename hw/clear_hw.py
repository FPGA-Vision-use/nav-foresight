"""净空核 clearance v1 板级驱动（不依赖 PYNQ / XRT，只用 /dev/mem + dma_heap）。

在板子上跑：
    python3 clear_hw.py regs                           # 全寄存器回归（19 参数 + 4 地址，逐字验证）
    python3 clear_hw.py selftest  params.json          # 零输入冒烟：43 路结果必须完全一致
    python3 clear_hw.py accept pts_u32.bin params.json ref.txt   # 与 csim 参考逐位对拍
    python3 clear_hw.py bench  pts_u32.bin params.json 20        # 计时

寄存器事实（hwh + 生成 RTL 双重确认）：
    s_axi_control   @0x80000000  CTRL@0x00 GIER@0x04 IER@0x08 ISR@0x0C
                                19 个参数从 0x10 起、间隔 8 字节
    s_axi_control_r @0x80010000  pts_1@0x10 pts_2@0x14 pts_ctrl@0x18
                                out_r_1@0x1C out_r_2@0x20 out_r_ctrl@0x24
    CTRL: bit0=ap_start(写1启动) bit1=ap_done bit2=ap_idle bit3=ap_ready
    基址可用环境变量 CLEAR_CTRL_BASE / CLEAR_ADDR_BASE 覆盖。

握手要点（v1 的驱动就栽在这两条）：
    1. ap_done 在 RTL 里只是把单周期脉冲 ap_done 打一拍
       （clearance_control_s_axi.v: `int_ap_done <= ap_done`），10 ms 级轮询必然打不中，
       **不能靠它判完成**。
    2. 跑完 FSM 回 state1、ap_start 自清 → CTRL 又变回 0x4，
       所以"读回 0x4"既可能是没启动、也可能是已经跑完了。
    判完成只认 ap_idle：写 start 后先等 idle 掉 0（确认真的起来了），再等 idle 回 1。

bitstream 前提：必须是 v2 BD（build_bd_v2.tcl）生成的。v1 的板级设计里 PS 的
M_AXI_HPM0_LPD 是 128 位而 IP 的 AXI-Lite 是 32 位，中间没插位宽转换器，导致
(offset & 0xF) != 0 的寄存器全都写不进（19 个参数只有 10 个可写，输出地址寄存器
out_r_1 永远写不进）。`regs` 子命令就是为了抓这类问题。
"""

import ctypes
import fcntl
import json
import mmap
import os
import struct
import sys
import time

CTRL_BASE = int(os.environ.get("CLEAR_CTRL_BASE", "0x80000000"), 0)
ADDR_BASE = int(os.environ.get("CLEAR_ADDR_BASE", "0x80010000"), 0)

CTRL = 0x00
GIER = 0x04
IER = 0x08
ISR = 0x0C

P_OFF = {
    "xmin": 0x10, "ymin": 0x18, "xmax": 0x20, "ymax": 0x28,
    "pcx": 0x30, "pcy": 0x38, "psx": 0x40, "psy": 0x48,
    "acx": 0x50, "acy": 0x58, "avx": 0x60, "avy": 0x68, "abase": 0x70,
    "ccx": 0x78, "ccy": 0x80, "cvx": 0x88, "cvy": 0x90, "crad": 0x98,
    "margin_mm": 0xA0, "sw_guard": 0xA8,
}
IN_LO, IN_HI = 0x10, 0x14
OUT_LO, OUT_HI = 0x1C, 0x20
ADDR_REGS = [(IN_LO, "pts_1"), (IN_HI, "pts_2"),
             (OUT_LO, "out_r_1"), (OUT_HI, "out_r_2")]

K, N = 43, 400
N_OUT = K * 5
IN_BYTES = K * N * 4
DMA_HEAP = "/dev/dma_heap/reserved"
DMA_HEAP_IOCTL_ALLOC = 0xC0184800

_SWEEP = None
_SWEEP_SRC = None

DEFAULT_P = {"xmin": 0, "ymin": -2600, "xmax": 2600, "ymax": 2100,
             "pcx": -620, "pcy": 800, "psx": 600, "psy": 600,
             "acx": 1500, "acy": 400, "avx": -900, "avy": 300, "abase": 520,
             "ccx": 1500, "ccy": 400, "cvx": -900, "cvy": 300, "crad": 400,
             "margin_mm": 230, "sw_guard": 350}


class DmaBuf:
    """从 dma_heap 分配一块 PL 能直接访问的物理连续缓冲。"""

    def __init__(self, size):
        self.size = (size + 4095) & ~4095
        hfd = os.open(DMA_HEAP, os.O_RDWR)
        try:
            arg = ctypes.create_string_buffer(
                struct.pack("<QIIQ", self.size, 0, os.O_RDWR, 0), 24)
            fcntl.ioctl(hfd, DMA_HEAP_IOCTL_ALLOC, arg, True)
            sz, dfd = struct.unpack("<QI", arg.raw[:12])
        finally:
            os.close(hfd)
        self.mm = mmap.mmap(dfd, sz, mmap.MAP_SHARED,
                            mmap.PROT_READ | mmap.PROT_WRITE)
        os.close(dfd)
        self.mm[0:4] = b"\x00\x00\x00\x00"      # 触碰一下，页才进 pagemap
        va = ctypes.addressof(ctypes.c_char.from_buffer(self.mm))
        with open("/proc/self/pagemap", "rb") as pm:
            pm.seek((va // 4096) * 8)
            e = struct.unpack("<Q", pm.read(8))[0]
        if not (e >> 63) & 1:
            raise RuntimeError("pagemap: page not present")
        pfn = e & ((1 << 55) - 1)
        if pfn == 0:
            raise RuntimeError("pagemap: pfn=0（拿不到物理地址，禁止启动核）")
        self.phys = pfn * 4096 + va % 4096

    def write(self, data):
        assert len(data) <= self.size
        self.mm[0:len(data)] = data

    def read(self, n):
        self.mm.seek(0)
        return self.mm.read(n)


    def flush(self):
        """把本进程写进这块缓冲的脏缓存行挤出去（触发写回 DRAM）。

        这块 dma_heap 映射是 cacheable 的，CPU 写完必须把脏行挤出去 PL 才看得到；
        板上的 dma-buf 不支持 DMA_BUF_IOCTL_SYNC，/dev/mem 也被 strict devmem 挡住，
        所以用"顺序扫一遍比 L2 大的内存"来强制写回。L2 ~1.5MB，扫 8MB 足够。
        """
        global _SWEEP, _SWEEP_SRC
        if _SWEEP is None:
            _SWEEP = bytearray(8 << 20)
            _SWEEP_SRC = bytes(8 << 20)
        _SWEEP[:] = _SWEEP_SRC

    def close(self):
        self.mm.close()


class ClearKernel:
    def __init__(self):
        self.fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        self.mc = mmap.mmap(self.fd, 0x1000, mmap.MAP_SHARED,
                            mmap.PROT_READ | mmap.PROT_WRITE, offset=CTRL_BASE)
        self.mr = mmap.mmap(self.fd, 0x1000, mmap.MAP_SHARED,
                            mmap.PROT_READ | mmap.PROT_WRITE, offset=ADDR_BASE)
        self.in_buf = DmaBuf(IN_BYTES)
        self.out_buf = DmaBuf(N_OUT * 4)

    def rc(self, off):
        return struct.unpack("<I", self.mc[off:off + 4])[0]

    def wc(self, off, val):
        self.mc[off:off + 4] = struct.pack("<I", val & 0xFFFFFFFF)
        return self.rc(off)

    def rr(self, off):
        return struct.unpack("<I", self.mr[off:off + 4])[0]

    def wr(self, off, val):
        self.mr[off:off + 4] = struct.pack("<I", val & 0xFFFFFFFF)
        return self.rr(off)

    def set_params(self, p):
        for name, off in P_OFF.items():
            if name not in p:
                raise KeyError("缺参数 " + name)
            back = self.wc(off, int(p[name]))
            if back != (int(p[name]) & 0xFFFFFFFF):
                raise RuntimeError(
                    "参数 %s@0x%02X 写 %d 读回 %d" % (name, off, p[name], back))

    def set_addrs(self):
        """写输入/输出物理地址，逐字验证。out_r_1 在 0x1C，不是 0x20。"""
        for off, val, name in (
                (IN_LO, self.in_buf.phys & 0xFFFFFFFF, "pts_1"),
                (IN_HI, (self.in_buf.phys >> 32) & 0xFFFFFFFF, "pts_2"),
                (OUT_LO, self.out_buf.phys & 0xFFFFFFFF, "out_r_1"),
                (OUT_HI, (self.out_buf.phys >> 32) & 0xFFFFFFFF, "out_r_2")):
            back = self.wr(off, val)
            if back != val:
                raise RuntimeError("%s@0x%02X 写 0x%08X 读回 0x%08X"
                                   % (name, off, val, back))

    def status(self):
        return self.rc(CTRL)

    def run(self, timeout_s=2.0):
        """启动并等 ap_idle 走一个 0→1 的完整周期。"""
        t0 = time.perf_counter()
        self.wc(CTRL, 0x0)
        if self.rc(CTRL) & 1:
            raise RuntimeError("CTRL 清不掉 start 位")
        self.wc(CTRL, 0x1)
        deadline = t0 + timeout_s
        started = False
        while time.perf_counter() < deadline:
            if not (self.rc(CTRL) & 4):
                started = True
                break
        if not started:
            return {"ok": False, "why": "no_start", "ctrl": self.rc(CTRL),
                    "elapsed_ms": 0.0}
        while time.perf_counter() < deadline:
            if self.rc(CTRL) & 4:
                return {"ok": True, "ctrl": self.rc(CTRL),
                        "elapsed_ms": (time.perf_counter() - t0) * 1e3}
        return {"ok": False, "why": "timeout", "ctrl": self.rc(CTRL),
                "elapsed_ms": (time.perf_counter() - t0) * 1e3}

    def margins(self):
        self.out_buf.mm.seek(0)
        v = struct.unpack("<%di" % N_OUT, self.out_buf.read(N_OUT * 4))
        return [list(v[i * 5:i * 5 + 5]) for i in range(K)]

    def close(self):
        self.in_buf.close()
        self.out_buf.close()
        self.mc.close()
        self.mr.close()
        os.close(self.fd)


def check_regs(k):
    """19 个参数 + 4 个地址寄存器逐字写读回。v1 的 bitstream 会在这里挂 9 项。"""
    bad = []
    for i, (name, off) in enumerate(sorted(P_OFF.items(), key=lambda kv: kv[1])):
        pat = (0x1000 + i) & 0xFFFFFFFF
        back = k.wc(off, pat)
        ok = back == pat
        if not ok:
            bad.append(name)
        print("  %-9s @0x%02X  写 %10d 读回 %10d  %s"
              % (name, off, pat, back, "OK" if ok else "FAIL"))
    for off, name in ADDR_REGS:
        pat = (0x5000 + off) & 0xFFFFFFFF
        back = k.wr(off, pat)
        ok = back == pat
        if not ok:
            bad.append(name)
        print("  %-9s @0x%02X  写 0x%08X 读回 0x%08X  %s"
              % (name, off, pat, back, "OK" if ok else "FAIL"))
    for off in P_OFF.values():
        k.wc(off, 0)
    for off, _ in ADDR_REGS:
        k.wr(off, 0)
    print("寄存器回归: %d/%d 通过" % (len(P_OFF) + len(ADDR_REGS) - len(bad),
                                len(P_OFF) + len(ADDR_REGS)))
    return not bad


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "selftest"
    k = ClearKernel()
    print("in_phys=0x%08x out_phys=0x%08x ctrl_base=0x%08x addr_base=0x%08x"
          % (k.in_buf.phys, k.out_buf.phys, CTRL_BASE, ADDR_BASE))
    print("CTRL 初值 0x%x" % k.status())

    if cmd == "regs":
        ok = check_regs(k)
        print("RESULT", "PASS" if ok else "FAIL")
        k.close()
        return

    if cmd == "selftest":
        p = json.load(open(sys.argv[2])) if len(sys.argv) > 2 else dict(DEFAULT_P)
        k.in_buf.write(bytes(IN_BYTES))       # 全 0 输入 → 43 路结果必须完全一致
        k.in_buf.flush()
        k.set_params(p)
        k.set_addrs()
        r = k.run()
        print("run:", r)
        if r["ok"]:
            m = k.margins()
            same = all(m[i] == m[0] for i in range(K))
            print("cand0:", m[0])
            print("43 路一致:", same)
            print("RESULT", "PASS" if (same and m[0][0] != 0) else "FAIL")
        else:
            print("RESULT FAIL")
        k.close()
        return

    pts_path = sys.argv[2] if len(sys.argv) > 2 else "/tmp/pts_u32.bin"
    p = json.load(open(sys.argv[3])) if len(sys.argv) > 3 else dict(DEFAULT_P)
    raw = open(pts_path, "rb").read()
    k.in_buf.write(raw)
    k.in_buf.flush()
    k.set_params(p)
    k.set_addrs()
    k.in_buf.mm.seek(0)
    got0 = struct.unpack("<I", k.in_buf.read(4))[0]
    exp0 = struct.unpack("<I", raw[:4])[0]
    print("输入首字回读 0x%08x (期望 0x%08x) %s"
          % (got0, exp0, "OK" if got0 == exp0 else "MISMATCH"))

    if cmd == "accept":
        r = k.run()
        print("run:", r)
        if not r["ok"]:
            print("RESULT FAIL")
            k.close()
            return
        got = k.margins()
        exp = [[int(x) for x in line.split()]
               for line in open(sys.argv[4]) if line.strip()]
        bad = 0
        worst = 0
        for i in range(min(K, len(exp))):
            for c in range(5):
                d = abs(got[i][c] - exp[i][c])
                if d:
                    bad += 1
                    worst = max(worst, d)
                    if bad <= 8:
                        print("  MISMATCH cand%d col%d got %d exp %d"
                              % (i, c, got[i][c], exp[i][c]))
        print("与 csim 不一致: %d/%d，最大偏差 %d mm" % (bad, K * 5, worst))
        print("cand0 got:", got[0])
        print("cand0 exp:", exp[0])
        print("RESULT", "PASS (比特级一致)" if bad == 0 else "FAIL")
        k.close()
        return

    if cmd == "bench":
        n = int(sys.argv[4]) if len(sys.argv) > 4 else 20
        ts = []
        for _ in range(n):
            r = k.run()
            if not r["ok"]:
                print("run failed:", r)
                break
            ts.append(r["elapsed_ms"])
        if ts:
            ts.sort()
            print("runs=%d min=%.3f med=%.3f max=%.3f ms"
                  % (len(ts), ts[0], ts[len(ts) // 2], ts[-1]))
            print("理论 %.3f ms (144518 拍 @ 50 MHz)" % (144518 / 50e3))
            print("净空核单帧 %.1f fps" % (1000.0 / ts[len(ts) // 2]))
            print("RESULT PASS")
        k.close()
        return

    print("unknown cmd", cmd)


if __name__ == "__main__":
    main()
