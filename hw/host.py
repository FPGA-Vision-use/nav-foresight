"""PC 侧编排：推文件到板子、加载 bitstream、跑 clear_hw.py、取回结果。

    python host.py check
    python host.py push            # 推 bd.bit + 驱动 + 数据到 /tmp
    python host.py load            # fpgautil 加载 bitstream
    python host.py selftest
    python host.py accept
    python host.py bench 20
    python host.py sh "<命令>"     # 任意命令（自动 sudo 密码）
"""
import hashlib
import os
import sys

import paramiko

HOST = os.environ.get("BOARD_HOST", "169.254.73.99")
USER = "ubuntu"
PWD = os.environ.get("BOARD_PWD", "")  # 板端密码，运行时用环境变量 BOARD_PWD 提供
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
BIT = os.path.join(ROOT, "board", "bd.bit")
BIT_ROLL = os.path.join(ROOT, "board", "bd_roll.bit")
BIT_INT = os.environ.get("BIT_INT_PATH") or os.path.join(ROOT, "board", "bd_int26.bit")
HWH = os.path.join(ROOT, "board", "bd.hwh")
# 集成版 BD 的 hwh：默认指向 2026.1 重建的那套；要回退 2023.2 就设 HWH_INT_PATH
HWH_INT = os.environ.get("HWH_INT_PATH") or os.path.join(ROOT, "board", "bd_int26.hwh")
FILES_FRAMES = [
    (os.path.join(HERE, "replay453.py"), "/tmp/replay453.py"),
    (os.path.join(ROOT, "replay", "ref_frames.jsonl"), "/tmp/ref_frames.jsonl"),
]
FILES_INT = [
    (os.path.join(HERE, "int_chain.py"), "/tmp/int_chain.py"),
    (os.path.join(ROOT, "margins_from_roll.txt"), "/tmp/chain_ref.txt"),
]
FILES_ROLL = [
    (os.path.join(HERE, "roll_hw.py"), "/tmp/roll_hw.py"),
    (os.path.join(ROOT, "rollout", "rollout_dump.txt"), "/tmp/roll_ref.txt"),
]
FILES = [
    (os.path.join(HERE, "clear_hw.py"), "/tmp/clear_hw.py"),
    (os.path.join(HERE, "params.json"), "/tmp/params.json"),
    (os.path.join(HERE, "pts_u32.bin"), "/tmp/pts_u32.bin"),
    (os.path.join(HERE, "ref.txt"), "/tmp/ref.txt"),
]


def parse_hwh_bases(path):
    """从 hwh 里按 IP 类型取 AXI-Lite 基址：{ 'rollout': (ctrl, ctrl_r), 'clearance': (...) }

    hwh 里 MODULE 是嵌套的，按"最近的、位置在前面的 MODULE 标签"归属才可靠。
    """
    import re
    s = open(path, encoding="utf-8", errors="replace").read()
    mods = [(m.start(), m.group(1))
            for m in re.finditer(r'<MODULE[^>]*VLNV="([^"]+)"', s)]
    out = {}
    for m in re.finditer(r'NAME="(C_S_AXI_CONTROL(_R)?_BASEADDR)" (?:BASE)?VALUE="([^"]+)"', s):
        pos, which, _, val = m.start(), m.group(1), m.group(2), m.group(3)
        owner = [v for p, v in mods if p < pos]
        if not owner:
            continue
        vlnv = owner[-1]
        kind = "rollout" if "rollout" in vlnv else ("clearance" if "clearance" in vlnv else None)
        if not kind:
            continue
        ctrl, cr = out.get(kind, (None, None))
        if which.endswith("_R_BASEADDR"):
            cr = val
        else:
            ctrl = val
        out[kind] = (ctrl, cr)
    return out


def connect():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PWD, timeout=25,
              banner_timeout=30, auth_timeout=30)
    return c


def sh(c, cmd, sudo=False, timeout=300, quiet=False):
    full = ("echo '%s' | sudo -S %s" % (PWD, cmd)) if sudo else cmd
    _, o, e = c.exec_command(full, timeout=timeout)
    out = o.read().decode("utf-8", "replace")
    err = e.read().decode("utf-8", "replace")
    if not quiet:
        if out.strip():
            print(out.rstrip())
        keep = [l for l in err.splitlines() if "password for" not in l]
        if "".join(keep).strip():
            print("[stderr]", "\n".join(keep).rstrip())
    return out, err


def sftp_put(c, local, remote):
    s = c.open_sftp()
    s.put(local, remote)
    st = s.stat(remote)
    s.close()
    return st.st_size


def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    c = connect()
    try:
        if cmd == "sh":
            sh(c, sys.argv[2], sudo=("--sudo" in sys.argv))
            return
        if cmd == "check":
            sh(c, "uname -a")
            sh(c, "cat /sys/class/fpga_manager/fpga0/state")
            sh(c, "ls -l /tmp/bd.bit /tmp/clear_hw.py 2>&1")
            sh(c, "md5sum /tmp/bd.bit 2>&1")
            print("[local]", md5(BIT), "bd.bit")
            sh(c, "free -m | head -3")
            sh(c, "ls /dev/dma_heap/")
            return
        if cmd == "push":
            print("bd.bit", sftp_put(c, BIT, "/tmp/bd.bit"), "B")
            for local, remote in FILES:
                print(remote, sftp_put(c, local, remote), "B")
            return
        if cmd == "load":
            print("远程 md5:", sh(c, "md5sum /tmp/bd.bit", quiet=True)[0].split()[0])
            print("本地 md5:", md5(BIT))
            sh(c, "fpgautil -b /tmp/bd.bit", sudo=True, timeout=120)
            sh(c, "cat /sys/class/fpga_manager/fpga0/state")
            return
        if cmd == "selftest":
            sh(c, "python3 /tmp/clear_hw.py selftest /tmp/params.json", sudo=True)
            return
        if cmd == "accept":
            sh(c, "python3 /tmp/clear_hw.py accept /tmp/pts_u32.bin "
                  "/tmp/params.json /tmp/ref.txt", sudo=True)
            return
        if cmd == "bench":
            n = sys.argv[2] if len(sys.argv) > 2 else "20"
            sh(c, "python3 /tmp/clear_hw.py bench /tmp/pts_u32.bin "
                  "/tmp/params.json %s" % n, sudo=True, timeout=600)
            return
        if cmd == "regs":
            sh(c, "python3 /tmp/clear_hw.py regs", sudo=True)
            return
        if cmd == "rollpush":
            print("bd_roll.bit", sftp_put(c, BIT_ROLL, "/tmp/bd_roll.bit"), "B")
            for local, remote in FILES_ROLL:
                print(remote, sftp_put(c, local, remote), "B")
            return
        if cmd == "rollload":
            print("远程 md5:", sh(c, "md5sum /tmp/bd_roll.bit", quiet=True)[0].split()[0])
            print("本地 md5:", md5(BIT_ROLL))
            sh(c, "fpgautil -b /tmp/bd_roll.bit", sudo=True, timeout=180)
            sh(c, "cat /sys/class/fpga_manager/fpga0/state")
            return
        if cmd == "intpush":
            print("bd_int.bit", sftp_put(c, BIT_INT, "/tmp/bd_int.bit"), "B")
            for local, remote in FILES_INT:
                print(remote, sftp_put(c, local, remote), "B")
            return
        if cmd == "intload":
            print("远程 md5:", sh(c, "md5sum /tmp/bd_int.bit", quiet=True)[0].split()[0])
            print("本地 md5:", md5(BIT_INT))
            sh(c, "fpgautil -b /tmp/bd_int.bit", sudo=True, timeout=180)
            sh(c, "cat /sys/class/fpga_manager/fpga0/state")
            return
        if cmd == "framespush":
            for local, remote in FILES_FRAMES:
                print(remote, sftp_put(c, local, remote), "B")
            return
        if cmd == "frames":
            b = parse_hwh_bases(HWH_INT)
            env = ""
            if b.get("rollout") and b["rollout"][0]:
                env += "ROLL_CTRL_BASE=%s ROLL_ADDR_BASE=%s " % b["rollout"]
            if b.get("clearance") and b["clearance"][0]:
                env += "CLEAR_CTRL_BASE=%s CLEAR_ADDR_BASE=%s " % b["clearance"]
            args = " ".join(sys.argv[2:])
            sh(c, "bash -c 'cd /tmp && %spython3 replay453.py %s'" % (env, args),
               sudo=True, timeout=1800)
            return
        if cmd == "ptsget":
            s2 = c.open_sftp()
            local = os.path.join(ROOT, "replay", "pts453.bin")
            s2.get("/tmp/pts453.bin", local)
            s2.close()
            print("已取回", local, os.path.getsize(local), "B")
            return
        if cmd == "livestreampush":
            s2 = c.open_sftp()
            for f in ("live_stream.py", "car_track.py", "udp_link.py", "person_track.py", "clear_hw.py",
                      "live_config.json", "car_track.json"):
                s2.put(os.path.join(HERE, f), "/tmp/" + f)
            s2.close()
            print("live_stream.py / car_track.py / live_config.json / car_track.json 已推")
            return
        if cmd == "livestream":
            b = parse_hwh_bases(HWH_INT)
            env = ""
            if b.get("rollout") and b["rollout"][0]:
                env += "ROLL_CTRL_BASE=%s ROLL_ADDR_BASE=%s " % b["rollout"]
            if b.get("clearance") and b["clearance"][0]:
                env += "CLEAR_CTRL_BASE=%s CLEAR_ADDR_BASE=%s " % b["clearance"]
            sh(c, "bash -c 'cd /tmp && %spython3 live_stream.py %s'"
               % (env, " ".join(sys.argv[2:])), sudo=True, timeout=7200)
            return
        if cmd == "get":
            remote = sys.argv[2]
            local = sys.argv[3] if len(sys.argv) > 3 else os.path.join(ROOT, os.path.basename(remote))
            s2 = c.open_sftp()
            s2.get(remote, local)
            s2.close()
            print("已取回", local, os.path.getsize(local), "B")
            return
        if cmd == "livepush":
            s2 = c.open_sftp()
            for f in ("live_demo.py",):
                s2.put(os.path.join(HERE, f), "/tmp/" + f)
            s2.put(os.path.join(ROOT, "replay", "live_ground.json"), "/tmp/live_ground.json")
            s2.close()
            print("live_demo.py + live_ground.json 已推")
            return
        if cmd == "live":
            b = parse_hwh_bases(HWH_INT)
            env = ""
            if b.get("rollout") and b["rollout"][0]:
                env += "ROLL_CTRL_BASE=%s ROLL_ADDR_BASE=%s " % b["rollout"]
            if b.get("clearance") and b["clearance"][0]:
                env += "CLEAR_CTRL_BASE=%s CLEAR_ADDR_BASE=%s " % b["clearance"]
            sh(c, "bash -c 'cd /tmp && %spython3 live_demo.py %s'"
               % (env, " ".join(sys.argv[2:])), sudo=True, timeout=1800)
            return
        if cmd == "mainlooppush":
            s2 = c.open_sftp()
            s2.put(os.path.join(HERE, "main_loop.py"), "/tmp/main_loop.py")
            s2.close()
            print("main_loop.py 已推")
            return
        if cmd == "mainloop":
            b = parse_hwh_bases(HWH_INT)
            env = ""
            if b.get("rollout") and b["rollout"][0]:
                env += "ROLL_CTRL_BASE=%s ROLL_ADDR_BASE=%s " % b["rollout"]
            if b.get("clearance") and b["clearance"][0]:
                env += "CLEAR_CTRL_BASE=%s CLEAR_ADDR_BASE=%s " % b["clearance"]
            sh(c, "bash -c 'cd /tmp && %spython3 main_loop.py %s'"
               % (env, " ".join(sys.argv[2:])), sudo=True, timeout=1800)
            return
        if cmd == "mainloopget":
            s2 = c.open_sftp()
            local = os.path.join(ROOT, "replay", "main_loop_log.jsonl")
            s2.get("/tmp/main_loop_log.jsonl", local)
            s2.close()
            print("已取回", local, os.path.getsize(local), "B")
            return
        if cmd == "framesget":
            s2 = c.open_sftp()
            local = os.path.join(ROOT, "replay", "replay453_out.jsonl")
            s2.get("/tmp/replay453_out.jsonl", local)
            s2.close()
            print("已取回", local, os.path.getsize(local), "B")
            return
        if cmd == "chain":
            b = parse_hwh_bases(HWH_INT)
            print("hwh 基址:", b)
            env = ""
            if b.get("rollout") and b["rollout"][0]:
                env += "ROLL_CTRL_BASE=%s ROLL_ADDR_BASE=%s " % (b["rollout"][0], b["rollout"][1])
            if b.get("clearance") and b["clearance"][0]:
                env += "CLEAR_CTRL_BASE=%s CLEAR_ADDR_BASE=%s " % (b["clearance"][0], b["clearance"][1])
            args = " ".join(a.replace("/tmp/", "") for a in sys.argv[2:])
            sh(c, "bash -c 'cd /tmp && %spython3 int_chain.py %s'" % (env, args),
               sudo=True, timeout=900)
            return
        if cmd == "roll":
            # 注意：Git Bash 会把以 /tmp/ 开头的独立参数改写成 Windows 临时目录，
            # 所以这里一律用相对路径（先 cd /tmp）
            args = " ".join(a.replace("/tmp/", "") for a in sys.argv[2:])
            sh(c, "bash -c 'cd /tmp && python3 roll_hw.py %s'" % args,
               sudo=True, timeout=900)
            return
        if cmd == "verify":
            # 一条命令走完全部验收，任何一步 FAIL 就停
            steps = [
                ("load", "fpgautil -b /tmp/bd.bit", True, 180),
                ("state", "cat /sys/class/fpga_manager/fpga0/state", False, 30),
                ("regs", "python3 /tmp/clear_hw.py regs", True, 120),
                ("selftest", "python3 /tmp/clear_hw.py selftest /tmp/params.json",
                 True, 180),
                ("accept", "python3 /tmp/clear_hw.py accept /tmp/pts_u32.bin "
                           "/tmp/params.json /tmp/ref.txt", True, 180),
                ("bench", "python3 /tmp/clear_hw.py bench /tmp/pts_u32.bin "
                          "/tmp/params.json 20", True, 600),
            ]
            failed = []
            for name, c2, sudo, to in steps:
                print("\n########## %s ##########" % name)
                out, _ = sh(c, c2, sudo=sudo, timeout=to)
                if "RESULT FAIL" in out or "Traceback" in out or (" MISM" in out and name == "accept"):
                    failed.append(name)
                    break
            print("\n===== 总结 =====")
            print("失败步骤:", failed if failed else "无（全部通过）")
            return
        print("unknown", cmd)
    finally:
        c.close()


if __name__ == "__main__":
    main()
