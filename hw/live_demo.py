"""实时演示：相机 → ArUco（车 + 行人）→ 地面坐标 → PL 两核 → 决策 → 录像。

板上跑（摄像头插上之后）：
    python3 live_demo.py probe                 # 列相机、抓一帧存 /tmp/live_probe.jpg
    python3 live_demo.py run [帧数] [输出.mp4]   # 默认 300 帧、/tmp/live_demo.mp4

感知方案（演示用的"标记替代检测器"）：
    车上贴 `markers/aruco_id5_vehicle.png`，行人（或手持板）贴 `aruco_id6_pedestrian.png`。
    取标记**下边中点**作为接触地面的点，经 `H_image_to_ground` 投到米制地面；
    车头朝向取标记**上边方向**的角度。位置/朝向都经过一阶低通，避免抖动。
    这样整条链（候选生成 → 五类净空 → 决策）与真实检测器版本完全一致，
    差别只在"人/车在画面里的位置"这一步。

注意：
  · 板上是 cv2 4.5.4，只有老 API（`DetectorParameters_create` / `detectMarkers`），
    没有 4.7+ 的 `ArucoDetector`；`polygonalApproxAccuracyRate=0.05` 必须显式设，
    否则远处小标记会被静默漏检。
  · 单应矩阵沿用录制时标定的 `ground_snapshot.json`（相机不动就一直有效）。
"""
import json
import math
import os
import struct
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, "/tmp")
import cv2  # noqa: E402
from clear_hw import DmaBuf  # noqa: E402

# ---------------- PL 接口 ----------------
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

# ---------------- 场景与决策参数（与 settings.json 对齐）----------------
BOUNDS = (0.0, -2.6, 2.6, 2.1)
PILLAR = {"c": (-0.62, 0.8), "s": (1.0, 1.0)}
MARGIN_M = 0.23
PERSON_R, VEH_R, EXTRA = 0.25, 0.15, 0.08
UNC = 0.22                     # actor_measurement_margin_m
CAP, HORIZON, STATIC_GUARD = 1.5, 1.0, 0.35
STRAIGHT_CLEAR, CPW = 0.5, 0.3
GOAL = (1.0, 1.7)
ID_VEH, ID_PED = 5, 6
# 标记离地高度（米）：底面贴地就给 0；离地 h 时要按相机高度做视差修正，
# 否则投影点会沿"远离相机"方向偏出约 h/H_cam 的相对量（1 m 高 ≈ 偏 1.2 m）。
H_VEH, H_PED = 0.0, 0.0
CAM_XY = np.array([2.2, -3.7], dtype=np.float64)   # camera_world_xyz_m 的 xy
CAM_H = 2.4

_SCRATCH = bytearray(2 << 20)
_ZERO = bytes(2 << 20)


def sweep():
    _SCRATCH[:] = _ZERO


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


def run_pl(regs, timeout_s=8.0):
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


class MarkerTracker:
    """两个标记 → 地面坐标（米）+ 朝向。带一阶低通。"""

    def __init__(self, H_img2ground):
        self.H = np.array(H_img2ground, dtype=np.float64)
        self.d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.p = cv2.aruco.DetectorParameters_create()
        self.p.polygonalApproxAccuracyRate = 0.05
        self.state = {}                      # id -> dict(x,y,h,t)

    def _ground(self, px, height=0.0):
        v = self.H @ np.array([px[0], px[1], 1.0])
        pos = np.array([v[0] / v[2], v[1] / v[2]])
        if height > 0.0:                       # 视差修正：同一像素在 z=h 处对应的地面点
            pos = CAM_XY + (pos - CAM_XY) * ((CAM_H - height) / CAM_H)
        return pos

    def update(self, img, now):
        corners, ids, _ = cv2.aruco.detectMarkers(img, self.d, parameters=self.p)
        out = {}
        if ids is None:
            return out
        for k, mid in enumerate(ids.flatten()):
            if mid not in (ID_VEH, ID_PED):
                continue
            c = corners[k].reshape(4, 2)
            # 下边（离图像下方更近的那条边的中点）＝接触地面的点
            e01 = (c[0] + c[1]) / 2
            e23 = (c[2] + c[3]) / 2
            foot = e01 if e01[1] > e23[1] else e23
            top = e23 if foot is e01 else e01
            pos = self._ground(foot, H_VEH if mid == ID_VEH else H_PED)   # 米
            h = math.atan2(-(top[0] - foot[0]), -(top[1] - foot[1]))   # 上边指向
            prev = self.state.get(int(mid))
            if prev is None:
                self.state[int(mid)] = dict(x=pos[0], y=pos[1], h=h, t=now, v=0.0)
            else:
                a = 0.35
                pv = prev["v"]
                dt = max(1e-3, now - prev["t"])
                v = math.hypot(pos[0] - prev["x"], pos[1] - prev["y"]) / dt
                self.state[int(mid)] = dict(
                    x=prev["x"] + a * (pos[0] - prev["x"]),
                    y=prev["y"] + a * (pos[1] - prev["y"]),
                    h=h, t=now,
                    v=pv + 0.35 * (min(v, CAP) - pv))
            out[int(mid)] = self.state[int(mid)]
        return out


def probe():
    found = False
    for i in range(4):
        dev = "/dev/video%d" % i
        if not os.path.exists(dev):
            continue
        found = True
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        ok, img = cap.read()
        print("%s: open=%s read=%s %s" % (dev, cap.isOpened(), ok,
              "" if not ok else "%dx%d" % (img.shape[1], img.shape[0])))
        if ok:
            cv2.imwrite("/tmp/live_probe.jpg", img)
            print("  已存 /tmp/live_probe.jpg")
        cap.release()
    if not found:
        print("没有 /dev/video*：摄像头还没插（或插在另一台机器上）")


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "probe"
    if cmd == "probe":
        probe()
        return

    nfr = int(sys.argv[2]) if len(sys.argv) > 2 else 300
    outp = sys.argv[3] if len(sys.argv) > 3 else "/tmp/live_demo.mp4"
    ground = json.load(open("/tmp/live_ground.json"))
    H = ground["mapping"]["H_image_to_ground"]
    trk = MarkerTracker(H)

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    if not cap.isOpened():
        print("打不开相机 0")
        return
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1920
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 1080
    vw = cv2.VideoWriter(outp, cv2.VideoWriter_fourcc(*"mp4v"), 25.0, (w, h))
    print("相机 %dx%d → %s，跑 %d 帧" % (w, h, outp, nfr), flush=True)

    pts = DmaBuf(PTS_BYTES + 256)
    mar = DmaBuf(64 * 4)
    pts.flush(); mar.flush()
    rc, ra = Regs(ROLL_CTRL), Regs(ROLL_ADDR)
    cc, ca = Regs(CLR_CTRL), Regs(CLR_ADDR)
    ra.wr(0x10, pts.phys & 0xFFFFFFFF); ra.wr(0x14, 0); ra.wr(0x1C, 0); ra.wr(0x20, 0)
    ca.wr(0x10, pts.phys & 0xFFFFFFFF); ca.wr(0x14, 0)
    ca.wr(0x1C, mar.phys & 0xFFFFFFFF); ca.wr(0x20, 0)
    opts = controls_bank()
    peak = np.array([o[0] for o in opts]); tgt = np.array([o[1] for o in opts])

    prev_trace = None
    t_all = time.perf_counter()
    n_ok = 0
    for i in range(nfr):
        ok, img = cap.read()
        if not ok:
            break
        t0 = time.perf_counter()
        now = t0 - t_all
        tracks = trk.update(img, now)
        veh = tracks.get(ID_VEH)
        ped = tracks.get(ID_PED)
        if veh is None:
            vw.write(img)
            continue
        # ---- 配 PL ----
        x, y, hd, v = veh["x"], veh["y"], veh["h"], veh["v"]
        for j, val in enumerate((x, y, math.cos(hd), math.sin(hd), min(v, 0.5), 0.0)):
            rc.wr(0x10 + 8 * j, struct.unpack("<I", struct.pack("<f", val))[0])
        p = {"xmin": int(BOUNDS[0] * 1000), "ymin": int(BOUNDS[1] * 1000),
             "xmax": int(BOUNDS[2] * 1000), "ymax": int(BOUNDS[3] * 1000),
             "pcx": int(PILLAR["c"][0] * 1000), "pcy": int(PILLAR["c"][1] * 1000),
             "psx": int(PILLAR["s"][0] * 1000), "psy": int(PILLAR["s"][1] * 1000),
             "margin_mm": int(MARGIN_M * 1000), "sw_guard": 350}
        if ped is not None:
            p.update(acx=int(ped["x"] * 1000), acy=int(ped["y"] * 1000),
                     avx=0, avy=0,                      # v1 演示：行人速度先给 0（走 PS 侧膨胀门）
                     abase=int((PERSON_R + MARGIN_M + UNC) * 1000),
                     ccx=int(ped["x"] * 1000), ccy=int(ped["y"] * 1000),
                     cvx=0, cvy=0, crad=int((PERSON_R + MARGIN_M + UNC) * 1000))
        else:
            p.update(acx=0, acy=0, avx=0, avy=0, abase=0,
                     ccx=0, ccy=0, cvx=0, cvy=0, crad=0)
        for name in P_ORDER:
            cc.wr(P_OFF[name], int(p[name]) & 0xFFFFFFFF)
        ms1 = run_pl(rc)
        ms2 = run_pl(cc)
        if ms1 is None or ms2 is None:
            vw.write(img); continue
        sweep()
        mv = np.array(struct.unpack("<215i", mar.read(860)), dtype=float).reshape(K, 5)
        raw = pts.mm[:PTS_BYTES]
        fl = np.frombuffer(pts.mm[PTS_BYTES:PTS_BYTES + K * 4], dtype=np.int32)
        w32 = np.frombuffer(raw, dtype=np.uint32).reshape(K, N)
        px = (((w32 & 0xFFFF).astype(np.int32) ^ 0x8000) - 0x8000).astype(np.float32) * 0.001
        py = ((((w32 >> 16) & 0xFFFF).astype(np.int32) ^ 0x8000) - 0x8000).astype(np.float32) * 0.001
        valid = (mv > 0).all(axis=1) & (fl != 0)
        if ped is not None:      # 行人无拟合速度 → PS 侧膨胀门
            apx, apy = np.float32(ped["x"]), np.float32(ped["y"])
            tg = np.arange(N, dtype=np.float32) * np.float32(0.02)
            rad = np.float32((PERSON_R + MARGIN_M + UNC)) + np.float32(CAP) * tg
            d2 = (px - apx) ** 2 + (py - apy) ** 2
            valid &= (d2 - rad[None, :] ** 2).min(axis=1) > 0
        straight = [k for k in range(1, K) if abs(peak[k]) < 1e-9 and tgt[k] > 0 and valid[k]]
        roomy = [k for k in straight if mv[k].min() >= STRAIGHT_CLEAR * 1000.0]
        dist = np.sqrt((px[:, -1] - np.float32(GOAL[0])) ** 2 + (py[:, -1] - np.float32(GOAL[1])) ** 2)
        dxp, dyp = np.diff(px, axis=1), np.diff(py, axis=1)
        length = np.sqrt(dxp * dxp + dyp * dyp).sum(axis=1)
        score = dist + np.float32(0.03) * length + np.float32(CPW) * np.abs(peak.astype(np.float32))
        if prev_trace is not None:
            score = score + np.float32(0.5) * np.sqrt((px - prev_trace[0]) ** 2
                                                      + (py - prev_trace[1]) ** 2).mean(axis=1)
        score = np.where(valid, score, np.inf)
        chosen = (max(roomy, key=lambda k: tgt[k]) if roomy
                  else (int(np.argmin(score)) if valid.any() else 0))
        prev_trace = (px[chosen], py[chosen])
        moving = bool(valid[chosen] and tgt[chosen] > 0)
        # ---- 叠加 ----
        Hinv = np.array(ground["mapping"]["H_ground_to_image"])
        def g2i(pts_m):
            a = np.asarray(pts_m, dtype=float)
            vv = np.concatenate([a, np.ones(a.shape[:-1] + (1,))], axis=-1) @ Hinv.T
            return (vv[..., :2] / np.maximum(vv[..., 2:3], 1e-9)).astype(np.int32)
        for k in range(K):
            tight = int(mv[k].min())
            col = (60, 60, 235) if tight <= 0 else ((0, 190, 255) if tight < 150 else (90, 190, 90))
            if k == chosen:
                col = (255, 255, 255) if tight > 0 else (60, 60, 255)
            cv2.polylines(img, [g2i(np.stack([px[k], py[k]], axis=1))], False, col,
                          4 if k == chosen else 1, cv2.LINE_AA)
        cv2.drawMarker(img, tuple(g2i(np.array([[x, y]]))[0]), (0, 255, 255),
                       cv2.MARKER_TRIANGLE_UP, 18, 3)
        txt = "cand %2d  %s   bnd %5d pil %5d timed %5d swept %5d cloud %5d   %.1f+%.1f ms" % (
            chosen, "PASS" if moving else "STOP", mv[chosen][0], mv[chosen][1],
            mv[chosen][2], mv[chosen][3], mv[chosen][4], ms1, ms2)
        cv2.putText(img, txt, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(img, txt, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (80, 240, 80) if moving else (80, 80, 255), 2, cv2.LINE_AA)
        vw.write(img)
        n_ok += 1
        if (i + 1) % 25 == 0:
            print("  %d/%d  平均 %.1f ms/帧" % (i + 1, nfr,
                  (time.perf_counter() - t_all) * 1e3 / (i + 1)), flush=True)
    cap.release(); vw.release()
    dt = (time.perf_counter() - t_all) * 1e3 / max(1, nfr)
    print("完成：有效 %d 帧，平均 %.1f ms/帧（%.1f fps），写 %s" % (n_ok, dt, 1000.0 / dt, outp))


if __name__ == "__main__":
    main()
