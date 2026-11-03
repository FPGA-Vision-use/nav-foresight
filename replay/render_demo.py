"""把板子的输出渲染成演示视频（真实回放帧 + 硅上算出来的轨迹）。

输入：
  replay/ref_frames.jsonl        每帧的初值/参数/planner 记录路径
  replay/replay453_out.jsonl     板子的 margin 与选择
  replay/pts453.bin              板子算出的 43×400 点（每帧 68800 B，int32 打包）
  media/pedestrian_avoidance_182616.mp4   源视频（453 帧，1920×1080，30fps）
  scene/ground_snapshot.json     地面→图像单应 H

输出：
  replay/demo_board.mp4          叠加了 43 路候选、被选路径、障碍与决策的视频

说明：视频里画的一切轨迹点都来自板子（pts453.bin），不是 Python 重算的。
"""
import json
import sys
from pathlib import Path

import numpy as np

EXP = Path(r"C:/Users/haoyu/Documents/ChatGPT/fpga/work/zhang_demo_20261026/native_repo/experiments")
sys.path.insert(0, str(EXP.parent / "runtime_cv411"))
import cv2  # noqa: E402

HERE = Path(__file__).resolve().parent
VIDEO = EXP / "pedestrian_avoidance_20261023/media/pedestrian_avoidance_182616.mp4"
GROUND = EXP / "geometry_fix_20261023/scene/ground_snapshot.json"
LAYOUT = EXP / "geometry_fix_20261023/scene/replay_config_snapshot.json"

K, N = 43, 400
COLS = ["boundary", "pillar", "timed", "sweep", "cloud"]


def proj(H, pts_m):
    """世界(m) → 图像(px)。pts_m: (...,2)"""
    p = np.asarray(pts_m, dtype=float)
    ones = np.ones(p.shape[:-1] + (1,))
    v = np.concatenate([p, ones], axis=-1) @ H.T
    return v[..., :2] / np.maximum(v[..., 2:3], 1e-9)


def main():
    ref = [json.loads(l) for l in open(HERE / "ref_frames.jsonl", encoding="utf-8")]
    got = {json.loads(l)["f"]: json.loads(l) for l in open(HERE / "replay453_out.jsonl", encoding="utf-8")}
    lay = json.loads(LAYOUT.read_text(encoding="utf-8"))
    bounds = lay["bounds_m"]
    pillar = lay["pillars"][0]
    pts_bin = (HERE / "pts453.bin").read_bytes()
    nf = len(pts_bin) // (K * N * 4)

    cap = cv2.VideoCapture(str(VIDEO))
    W, Hh = 1920, 1080
    out = cv2.VideoWriter(str(HERE / "demo_board.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (W, Hh))

    # 世界视图面板：米 → 像素（y 向上）
    PX0, PY0, PW, PH = 1000, 60, 890, 950
    SPAN_X, SPAN_Y = 5.2, 8.0          # 视野（米）
    SX = min(PW / SPAN_X, PH / SPAN_Y)
    CXW, CYW = 1.3, 0.3                # 视野中心（米）

    def w2p(xy):
        a = np.asarray(xy, dtype=float)
        px = PX0 + PW / 2 + (a[..., 0] - CXW) * SX
        py = PY0 + PH / 2 - (a[..., 1] - CYW) * SX
        return np.stack([px, py], axis=-1).astype(np.int32)

    for f in range(nf):
        ok, raw = cap.read()
        if not ok:
            break
        g = got.get(f)
        canvas = np.full((Hh, W, 3), 18, np.uint8)
        # 左：原始视频（含原有标注），缩放
        small = cv2.resize(raw, (960, 540), interpolation=cv2.INTER_AREA)
        canvas[80:80 + 540, 20:20 + 960] = small
        cv2.putText(canvas, "camera (recorded replay)", (24, 60), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (180, 180, 180), 2, cv2.LINE_AA)
        cv2.putText(canvas, "world view (computed on KV260)", (PX0, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2, cv2.LINE_AA)
        cv2.rectangle(canvas, (PX0, PY0), (PX0 + PW, PY0 + PH), (70, 70, 70), 2)
        if g is None:
            out.write(canvas); continue
        m = np.array(g["m"], dtype=np.int64).reshape(K, 5)
        words = np.frombuffer(pts_bin, dtype=np.uint32, count=K * N, offset=f * K * N * 4).reshape(K, N)
        sx = ((words & 0xFFFF).astype(np.int32) ^ 0x8000) - 0x8000
        sy = (((words >> 16) & 0xFFFF).astype(np.int32) ^ 0x8000) - 0x8000
        xy = np.stack([sx, sy], axis=2) / 1000.0
        chosen = ref[f]["chosen"]

        # 走廊 + 柱子 + 行人
        bx = np.array([[bounds[0], bounds[1]], [bounds[2], bounds[1]],
                       [bounds[2], bounds[3]], [bounds[0], bounds[3]]])
        cv2.polylines(canvas, [w2p(bx)], True, (120, 120, 120), 2)
        pc = np.array(pillar["center_m"]); ps = np.array(pillar["size_m"]) / 2
        pcorn = np.array([[pc[0] - ps[0], pc[1] - ps[1]], [pc[0] + ps[0], pc[1] - ps[1]],
                          [pc[0] + ps[0], pc[1] + ps[1]], [pc[0] - ps[0], pc[1] + ps[1]]])
        cv2.polylines(canvas, [w2p(pcorn)], True, (40, 170, 255), 3)
        prm = ref[f]["params"]
        if ref[f]["actor_valid"]:
            ap = np.array([prm["acx"], prm["acy"]]) / 1000.0
            rad = prm["abase"] / 1000.0
            cv2.circle(canvas, tuple(w2p(ap)), max(4, int(rad * SX)), (70, 70, 255), 2)
            cv2.circle(canvas, tuple(w2p(ap)), 6, (70, 70, 255), -1)
            av = np.array([prm["avx"], prm["avy"]]) / 1000.0
            if np.linalg.norm(av) > 1e-6:
                cv2.arrowedLine(canvas, tuple(w2p(ap)), tuple(w2p(ap + av * 0.8)),
                                (70, 70, 255), 3, tipLength=0.3)

        # 43 路候选
        for k in range(K):
            tight = int(m[k].min())
            if tight <= 0:
                col = (60, 60, 230)
            elif tight < 150:
                col = (0, 190, 255)
            else:
                col = (90, 190, 90)
            if k == chosen:
                col = (255, 255, 255) if tight > 0 else (60, 60, 255)
            cv2.polylines(canvas, [w2p(xy[k])], False, col, 4 if k == chosen else 1, cv2.LINE_AA)
        v0 = xy[chosen][0]
        cv2.drawMarker(canvas, tuple(w2p(v0)), (0, 255, 255), cv2.MARKER_TRIANGLE_UP, 18, 3)

        mv = ref[f]["actor_valid"] and ref[f]["cloud_cond"]
        mm = [int(m[chosen][c]) for c in range(5)]
        safe = all((x > 0) if (i < 2 or mv) else True for i, x in enumerate(mm))
        decision = "PASS" if (safe and chosen != 0) else "STOP"
        dcol = (80, 230, 80) if decision == "PASS" else (80, 80, 255)
        lines = [
            (("frame %3d / %d" % (f, nf)), (220, 220, 220)),
            (("cand %2d   %s" % (chosen, decision)), dcol),
            (("boundary %5d mm" % mm[0]), (200, 200, 200)),
            (("pillar   %5d mm" % mm[1]), (200, 200, 200)),
            (("timed    %5d mm" % mm[2]), (200, 200, 200)),
            (("swept    %5d mm" % mm[3]), (200, 200, 200)),
            (("cloud    %5d mm" % mm[4]), (200, 200, 200)),
            (("PL  %.1f + %.1f = %.1f ms" % (g["ms1"], g["ms2"], g["ms1"] + g["ms2"])), (150, 220, 255)),
        ]
        y = 700
        for txt, col in lines:
            cv2.putText(canvas, txt, (26, y), cv2.FONT_HERSHEY_SIMPLEX, 0.78, (0, 0, 0), 5, cv2.LINE_AA)
            cv2.putText(canvas, txt, (26, y), cv2.FONT_HERSHEY_SIMPLEX, 0.78, col, 2, cv2.LINE_AA)
            y += 46
        out.write(canvas)
        if f % 60 == 0:
            print("  渲染 %d/%d" % (f, nf), flush=True)
    cap.release(); out.release()
    print("写出", HERE / "demo_board.mp4")


if __name__ == "__main__":
    main()
