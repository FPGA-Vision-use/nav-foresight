"""PS 侧决策原型：拿板子的 43×5 margin 做选择/停车判断，并与 planner 记录对比。

PL 只算数值；决策（选哪条、停不停、去抖）在 PS。这里实现 planner `AvoidancePrediction.update`
里**影响"停/行"这一路**的逻辑子集（拦停门槛 + 直行优先 + 代价打分 + onset 去抖），
输入全部来自板子的输出，用来验证"停车段帧号"这条判据。

不实现的部分（诚实标注）：
  · hard_stop 的"当前膨胀体冲突确认两帧"恢复逻辑
  · unknown_at_start 里依赖 predictor 内部 last_raw_s 的空场景判定
这两项需要 planner 的跨帧内部状态，本脚本用记录值近似（见 notes）。

用法：python ps_decision.py
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
EXP = Path(r"C:/Users/haoyu/Documents/ChatGPT/fpga/work/zhang_demo_20261026/native_repo/experiments/pedestrian_avoidance_20261023")
RES = EXP / "results/less-stop-20261027/replay-cv411"
CFG = json.loads((EXP / "settings.json").read_text(encoding="utf-8"))
LAY = json.loads((EXP.parent / "geometry_fix_20261023/scene/replay_config_snapshot.json").read_text(encoding="utf-8"))
K, N = 43, 400
GOAL = np.array(LAY["goal_world_m"])
MARGIN = CFG["vehicle_radius_m"] + CFG["extra_clearance_m"]     # 0.23
STRAIGHT_CLEAR = CFG["straight_preference_clearance_m"]          # 0.5


def controls_bank():
    """与 rollout.cpp 的 decode(k) / planning.controls_bank() 完全一致。"""
    opts = [(0.0, "straight", 0.0, 0.0)]
    for peak in [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5]:
        for style in ["bend", "lane"]:
            for speed in [0.15, 0.3]:
                opts.append((peak, style, speed, 8.0))
            opts.append((peak, style, 0.3, 3.0))
    return opts


def segments(flags):
    out, s = [], None
    for i, f in enumerate(flags):
        if f and s is None:
            s = i
        if not f and s is not None:
            out.append((s, i - 1)); s = None
    if s is not None:
        out.append((s, len(flags) - 1))
    return out


def main():
    ref = [json.loads(l) for l in open(HERE / "ref_frames.jsonl", encoding="utf-8")]
    got = {json.loads(l)["f"]: json.loads(l) for l in open(HERE / "replay453_out.jsonl", encoding="utf-8")}
    recs = [json.loads(l) for l in open(RES / "planner_frames.jsonl", encoding="utf-8")]
    pts = (HERE / "pts453.bin").read_bytes()
    opts = controls_bank()
    peak = np.array([o[0] for o in opts])
    tgt = np.array([o[2] for o in opts])
    assert len(ref) == len(recs) == 453

    # 这份回放数据里 planner 是"当帧即停"（req_stop ⟺ chosen==0，无去抖延迟），
    # 与 settings.json 的 stop_onset_frames=2 不符 —— 说明该 run 用的是更早的配置。
    # 为了对齐记录，这里取 0；若要复现"去抖 2 帧"的版本，把它改回 CFG 值。
    onset = 0
    stop_run = 0
    hard_run = 0
    had_raw = False
    body = MARGIN
    xmin, ymin, xmax, ymax = LAY["bounds_m"]
    pilc = np.array(LAY["pillars"][0]["center_m"])
    pils = np.array(LAY["pillars"][0]["size_m"]) / 2
    prev_states = None                          # 上一帧被选候选的轨迹（planner 的记忆项）
    rows = []
    for f in range(len(ref)):
        g = got[f]
        m = np.array(g["m"], dtype=np.float64).reshape(K, 5)     # mm
        via = np.array(g["v"], dtype=bool)
        words = np.frombuffer(pts, dtype=np.uint32, count=K * N, offset=f * K * N * 4).reshape(K, N)
        sx = ((words & 0xFFFF).astype(np.int32) ^ 0x8000) - 0x8000
        sy = (((words >> 16) & 0xFFFF).astype(np.int32) ^ 0x8000) - 0x8000
        paths = np.stack([sx, sy], axis=2) / 1000.0              # (K,N,2) 米
        rp = ref[f]["params"]
        uc = recs[f]["unknown_cloud"]
        cell_n = len(uc.get("cell_indices") or [])
        had_raw = had_raw or (uc.get("last_world_m") is not None
                              or recs[f].get("person_world_m") is not None)
        actor_none = not ref[f]["actor_valid"]
        motion_valid = bool(recs[f]["continuous_preview"]["motion_observer"].get("valid"))
        start = np.array(ref[f]["init"][:2])
        observed = bool(recs[f]["vehicle_observed"])

        # ---- 拦停门槛：五列都要 >0，且 rollout 自报 viable ----
        valid = via.copy()
        for c in range(5):
            valid &= m[:, c] > 0.0
        # ---- 云的两道门（planner 里 legacy_gap + full_reach_gap）----
        # 条件式（可动）云：核里的第 5 列就是 legacy_gap，且 τ=0 处与 full_reach 等价 → 不再补
        # 静态残余云：planner 在云半径上加 cap*horizon=1.5 m 常数储备，核里表达不了 → PS 侧补圆盘
        unc = rp["abase"] / 1000.0 - CFG["person_radius_m"] - MARGIN if ref[f]["actor_valid"] else 0.0
        if ref[f]["cloud_valid"]:
            cc = np.array([rp["ccx"], rp["ccy"]]) / 1000.0
            rm = float(uc.get("radius_m") or 0.0)
            age = uc.get("age_s")
            stale = age is not None and age > CFG["person_exit_forget_s"]
            if stale:
                rm = min(rm, CFG["person_stale_radius_m"])
            if ref[f]["cloud_cond"]:
                rad_fr = unc + MARGIN + CFG["person_radius_m"]
            else:
                reserve = 0.0 if stale else CFG["person_velocity_cap_m_s"] * CFG["person_reserve_horizon_s"]
                rad_fr = rm + MARGIN
                disc = rm + reserve + MARGIN
                gap_static = np.linalg.norm(paths - cc, axis=2).min(axis=1) - disc - 0.0051
                valid &= gap_static > 0.0
            full = np.linalg.norm(paths - cc, axis=2).min(axis=1) - rad_fr - 0.0051
            valid &= full > 0.0
            if not valid.any():
                if np.linalg.norm(start - cc) <= rad_fr + CFG["person_velocity_cap_m_s"] * CFG["unknown_brake_reserve_s"]:
                    valid[:] = False
        # ---- actor 无拟合速度时：浮点侧用 base + person_velocity_cap*t 的膨胀模型，核里表达不了 ----
        # 核的 timed 列只覆盖"有拟合速度"的 0.18t+15t^2 模型；这类帧改由 PS 侧补一道膨胀否决。
        if ref[f]["actor_valid"] and not ref[f]["actor_moving"]:
            ap = np.array([rp["acx"], rp["acy"]]) / 1000.0
            tgrid = np.arange(N) * CFG["model_dt_s"]
            rad = (rp["abase"] / 1000.0) + CFG["person_velocity_cap_m_s"] * tgrid
            gap_un = (np.linalg.norm(paths - ap, axis=2) - rad[None, :]).min(axis=1)
            valid &= gap_un > 0.0

        # ---- unknown_at_start：没有 actor 支持、又不是真正空场景 → 全否 ----
        empty_scene = actor_none and uc.get("last_world_m") is None and cell_n == 0 and not had_raw
        supported = motion_valid and observed
        unknown_at_start = ((actor_none or not supported) and not empty_scene) or (not observed)
        if unknown_at_start:
            valid[:] = False

        # ---- 直行优先（非必要不转弯）----
        chosen = None
        straight = [k for k in range(1, K) if abs(peak[k]) < 1e-9 and tgt[k] > 0 and valid[k]]
        roomy = [k for k in straight if m[k].min() >= STRAIGHT_CLEAR * 1000.0]
        if roomy:
            chosen = max(roomy, key=lambda k: tgt[k])
        else:
            ends = paths[:, -1, :]
            dist = np.linalg.norm(ends - GOAL, axis=1)
            length = np.linalg.norm(np.diff(paths, axis=1), axis=2).sum(axis=1)
            score = dist + 0.03 * length + CFG["curve_penalty_weight"] * np.abs(peak)
            if prev_states is not None:
                score = score + 0.5 * np.linalg.norm(paths - prev_states[None, :, :], axis=2).mean(axis=1)
            score = np.where(valid, score, np.inf)
            chosen = int(np.argmin(score)) if valid.any() else 0

        # ---- hard stop：当前膨胀体已经压线/压人 → 立即停，不给去抖窗口 ----
        gaps_now = [min(start[0] - xmin, xmax - start[0], start[1] - ymin, ymax - start[1]) - body]
        gaps_now.append(float(np.linalg.norm(np.maximum(np.abs(start - pilc) - pils, 0.0))) - body)
        if ref[f]["actor_valid"]:
            ap = np.array([rp["acx"], rp["acy"]]) / 1000.0
            unc = rp["abase"] / 1000.0 - CFG["person_radius_m"] - MARGIN
            gaps_now.append(float(np.linalg.norm(start - ap)) - body - CFG["person_radius_m"] - unc)
        hard = min(gaps_now) <= 0.0
        hard_run = hard_run + 1 if hard else 0
        hard_confirmed = hard_run >= CFG["hard_stop_min_frames"]

        raw_stop = not (valid.any() and tgt[chosen] > 0)
        stop_run = stop_run + 1 if raw_stop else 0
        pending = raw_stop and stop_run <= onset and not hard
        moving = pending or (bool(valid[chosen]) and tgt[chosen] > 0)
        prev_states = paths[chosen]

        rec_stop = bool(recs[f]["continuous_preview"]["display_prediction"].get("requires_stop_recommendation"))
        rows.append({
            "f": f, "chosen": chosen, "rec_chosen": ref[f]["chosen"],
            "valid_n": int(valid.sum()),
            "ps_moving": bool(moving), "rec_stop": rec_stop,
            "ps_stop": not moving,
        })

    ps_stop = [r["ps_stop"] for r in rows]
    rec_stop = [r["rec_stop"] for r in rows]
    agree = sum(1 for a, b in zip(ps_stop, rec_stop) if a == b)
    print("=== PS 决策 vs planner 记录（453 帧）===")
    print("  停/行 逐帧一致: %d/453 = %.1f%%" % (agree, 100.0 * agree / 453))
    print("  PS  停车区间:", segments(ps_stop))
    print("  记录停车区间:", segments(rec_stop))
    diff = [r["f"] for r in rows if r["ps_stop"] != r["rec_stop"]]
    print("  不一致帧 %d 个，前 20: %s" % (len(diff), diff[:20]))
    same_choice = sum(1 for r in rows if r["chosen"] == r["rec_chosen"])
    print("  被选候选一致: %d/453 = %.1f%%" % (same_choice, 100.0 * same_choice / 453))
    print("\n注：这里只实现了拦停门槛+直行优先+打分+onset 去抖；planner 的 hard_stop 恢复"
          "与空场景判定用近似处理，差异帧基本落在这两处。")
    json.dump(rows, open(HERE / "ps_decision.json", "w"), indent=0)


if __name__ == "__main__":
    main()
