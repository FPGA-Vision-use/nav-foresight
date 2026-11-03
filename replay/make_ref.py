"""453 帧验收参考生成器（PC 侧）。

不需要复刻 planner 状态机：`planner_frames.jsonl` 每帧都记录了
  display_prediction.trace.states[0]   = [x, y, heading, v, a, curvature]（planner 内部初值）
  display_prediction.candidate_index   = 被选候选
  display_prediction.signed_margins_m  = 被选候选的浮点 margin（自检用）
  display_prediction.actor_prediction  = actor（center/velocity/uncertainty）
  unknown_cloud                        = 云的原始记录
于是可以：拿记录下来的初值跑 batch_rollout + obstacle_clearance + legacy_gap，
用"被选候选的 margin"验证重建是否逐项吻合，再输出全部 43 路的浮点 margin 给板子对拍。

用法：
    python make_ref.py check           # 只自检前 N 帧（默认 6）
    python make_ref.py all             # 生成 ref_frames.jsonl（453 帧）
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

EXP = Path(r"C:/Users/haoyu/Documents/ChatGPT/fpga/work/zhang_demo_20261026/native_repo/experiments/pedestrian_avoidance_20261023")
RES = EXP / "results/less-stop-20261027/replay-cv411"
LAYOUT = EXP.parent / "geometry_fix_20261023/scene/replay_config_snapshot.json"
OUT = Path(__file__).resolve().parent

sys.path.insert(0, str(EXP))
import planning  # noqa: E402

CFG = json.loads((EXP / "settings.json").read_text(encoding="utf-8"))
LAY = json.loads(LAYOUT.read_text(encoding="utf-8"))
BOUNDS = tuple(LAY["bounds_m"])
PILLAR = LAY["pillars"][0]
K, N = 43, 400
POS_GUARD_MM = 5.1      # kinematics.speed_m_s*model_dt_s/2+.0001


def frames():
    with open(RES / "planner_frames.jsonl", encoding="utf-8") as f:
        for line in f:
            yield json.loads(line)


def per_frame(d):
    """返回 (init, actor, guard_cloud, chosen, recorded_margins, status)。"""
    dp = d["continuous_preview"]["display_prediction"]
    init = np.array(dp["trace"]["states"][0], dtype=float)
    actor = dp["actor_prediction"]
    cloud = d["unknown_cloud"]
    guard = planning.reserve_cloud(cloud, actor)
    rec = dict(dp["signed_margins_m"])
    rec["legacy_reserve_gap_m"] = dp.get("legacy_reserve_gap_m")
    return init, actor, guard, int(dp["candidate_index"]), rec, d["route_status"]


def compute(init, actor, guard):
    states, cmds, viable, options = planning.batch_rollout(init, CFG)
    times = np.arange(len(states)) * CFG["model_dt_s"]
    m = planning.obstacle_clearance(states[:, :, :2], times, actor, BOUNDS, PILLAR, CFG)
    cg = planning.legacy_gap(states[:, :, :2], guard, CFG)
    return states, m, cg


def mm(x):
    return None if x is None or not np.isfinite(x) else float(x) * 1000.0


def cmd_check(nshow=6):
    print("bounds", BOUNDS, "pillar", PILLAR["center_m"], PILLAR["size_m"])
    worst = {"boundary": 0.0, "pillar": 0.0, "timed": 0.0, "sweep": 0.0, "cloud": 0.0}
    for i, d in enumerate(frames()):
        if i >= nshow:
            break
        init, actor, guard, chosen, rec, status = per_frame(d)
        states, m, cg = compute(init, actor, guard)
        got = {
            "boundary": mm(m[0][chosen]),
            "pillar": mm(m[1][chosen]),
            "timed": mm(m[2][chosen]),
            "sweep": mm(m[3][chosen]),
            "cloud": mm(cg[chosen]),
        }
        exp = {
            "boundary": mm(rec.get("boundary")),
            "pillar": mm(rec.get("pillar")),
            "timed": mm(rec.get("timed_person")),
            "sweep": mm(rec.get("current_and_swept_person")),
            "cloud": mm(rec.get("legacy_reserve_gap_m")),
        }
        print("帧 %-3d 候选 %-2d status %-40s" % (i, chosen, status))
        for k in got:
            gv, ev = got[k], exp[k]
            dif = None if (gv is None or ev is None) else gv - ev
            if dif is not None:
                worst[k] = max(worst[k], abs(dif))
            print("    %-8s 重建 %12s  记录 %12s  差 %s"
                  % (k, "None" if gv is None else "%.3f" % gv,
                     "None" if ev is None else "%.3f" % ev,
                     "None" if dif is None else "%.4f" % dif))
    print("\n最大偏差(mm):", {k: round(v, 4) for k, v in worst.items()})


def cmd_all():
    out = open(OUT / "ref_frames.jsonl", "w", encoding="utf-8")
    n_actor_v = n_cloud_moving = n_cloud_static = n_inf_timed = 0
    dev = []
    for i, d in enumerate(frames()):
        init, actor, guard, chosen, rec, status = per_frame(d)
        states, m, cg = compute(init, actor, guard)
        # 自检：被选候选的 5 列
        for got, exp in ((mm(m[0][chosen]), mm(rec.get("boundary"))),
                         (mm(m[1][chosen]), mm(rec.get("pillar"))),
                         (mm(m[2][chosen]), mm(rec.get("timed_person"))),
                         (mm(m[3][chosen]), mm(rec.get("current_and_swept_person"))),
                         (mm(cg[chosen]), mm(rec.get("legacy_reserve_gap_m")))):
            if got is not None and exp is not None:
                dev.append(abs(got - exp))
        if actor is not None and actor.get("velocity_m_s") is not None:
            n_actor_v += 1
        if guard.get("last_world_m") is not None:
            if guard.get("_conditional_actor_reserve"):
                n_cloud_moving += 1
            else:
                n_cloud_static += 1
        if not np.isfinite(m[2]).all():
            n_inf_timed += 1

        # 给板子的参数（毫米整数）
        margin_mm = int(round((CFG["vehicle_radius_m"] + CFG["extra_clearance_m"]) * 1000))
        if actor is not None:
            base = CFG["person_radius_m"] + CFG["vehicle_radius_m"] + CFG["extra_clearance_m"] + actor["uncertainty_m"]
            vel = actor["velocity_m_s"] or [0.0, 0.0]
            ac = [float(actor["center_m"][0]), float(actor["center_m"][1])]
            av = [float(vel[0]), float(vel[1])]
            abase = int(round(base * 1000))
            a_valid = 1
        else:
            ac, av, abase, a_valid = [0.0, 0.0], [0.0, 0.0], 0, 0
        if guard.get("last_world_m") is not None:
            cc = [float(guard["last_world_m"][0]), float(guard["last_world_m"][1])]
            cv = [float(x) for x in (guard.get("velocity_m_s") or [0.0, 0.0])]
            unc = guard["radius_m"] if guard.get("_conditional_actor_reserve") else 0.0
            crad = int(round((CFG["person_radius_m"] + CFG["vehicle_radius_m"]
                              + CFG["extra_clearance_m"] + unc) * 1000))
            c_valid = 1
        else:
            cc, cv, crad, c_valid = [0.0, 0.0], [0.0, 0.0], 0, 0

        rec_out = {
            "frame": i,
            "status": status,
            "chosen": chosen,
            "init": [float(v) for v in init[:4]] + [float(init[5])],   # x, y, heading, v, curvature
            "actor_valid": a_valid, "cloud_valid": c_valid,
            "actor_moving": int(actor is not None and actor.get("velocity_m_s") is not None),
            "cloud_cond": int(bool(guard.get("_conditional_actor_reserve"))),
            # PS 决策要用的观测/退化标志（这些是"核里表达不了、由 PS 补门"的输入）
            "observed": int(bool(d.get("vehicle_observed"))),
            "motion_valid": int(bool(d["continuous_preview"]["motion_observer"].get("valid"))),
            "cell_n": len((d["unknown_cloud"].get("cell_indices") or [])),
            "radius_m": (None if d["unknown_cloud"].get("radius_m") is None
                         else float(d["unknown_cloud"]["radius_m"])),
            "age_s": (None if d["unknown_cloud"].get("age_s") is None
                      else float(d["unknown_cloud"]["age_s"])),
            "params": {
                "xmin": 0, "ymin": int(round(BOUNDS[1] * 1000)),
                "xmax": int(round(BOUNDS[2] * 1000)), "ymax": int(round(BOUNDS[3] * 1000)),
                "pcx": int(round(PILLAR["center_m"][0] * 1000)),
                "pcy": int(round(PILLAR["center_m"][1] * 1000)),
                "psx": int(round(PILLAR["size_m"][0] * 1000)),
                "psy": int(round(PILLAR["size_m"][1] * 1000)),
                "acx": int(round(ac[0] * 1000)), "acy": int(round(ac[1] * 1000)),
                "avx": int(round(av[0] * 1000)), "avy": int(round(av[1] * 1000)),
                "abase": abase,
                "ccx": int(round(cc[0] * 1000)), "ccy": int(round(cc[1] * 1000)),
                "cvx": int(round(cv[0] * 1000)), "cvy": int(round(cv[1] * 1000)),
                "crad": crad, "margin_mm": margin_mm, "sw_guard": 350,
            },
            "margins_mm": [[None if not np.isfinite(x) else round(float(x) * 1000, 3)
                            for x in (m[0][k], m[1][k], m[2][k], m[3][k], cg[k])]
                           for k in range(K)],
            # planner 记录的被选候选路径（401 点，毫米整数），用来验证板子的 rollout
            "path_mm": [[int(round(pt[0] * 1000)), int(round(pt[1] * 1000))]
                        for pt in d["continuous_preview"]["display_prediction"]["world_m"]],
        }
        out.write(json.dumps(rec_out) + "\n")
    out.close()
    dev = np.array(dev)
    print("自检(被选候选 5 列与记录值之比): n=%d  max=%.4f mm  p99=%.4f mm  mean=%.4f mm"
          % (dev.size, dev.max(), np.percentile(dev, 99), dev.mean()))
    print("有速度的 actor 帧: %d / 453" % n_actor_v)
    print("云：条件式(可动) %d 帧，静态残余 %d 帧" % (n_cloud_moving, n_cloud_static))
    print("timed 全 inf 的帧: %d" % n_inf_timed)
    print("写出", OUT / "ref_frames.jsonl")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "all":
        cmd_all()
    else:
        cmd_check(int(sys.argv[2]) if len(sys.argv) > 2 else 6)
