"""
주행 궤적 시각화 — 시나리오를 오프라인으로 굴려 ego 경로/장애물/행동상태를 그림으로.
소켓 없이 제어스택을 직접 호출하는 결정론적 bicycle 시뮬. 결과 PNG 저장.
사용: python3 visualize.py [out.png] [scenario1.json scenario2.json ...]
"""
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "src"))
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from scenario import Scenario
from vtd_io import State, Obj, TL_RED
from control import PurePursuit, speed_profile_2pass, apply_speed_limits, LongPI
from behavior import Behavior
from route_frame import offset_path
from mock_vtd import init_actor_state, step_actor, spawned, tl_state_at

L = 2.95
STATE_COLOR = {
    "LANE_KEEP": "#2c7fb8", "AVOID": "#f16913", "FOLLOW": "#31a354",
    "YIELD_PED": "#d7301f", "OBSTACLE_STOP": "#d7301f",
    "RED_STOP": "#d7301f", "RED_SLOW": "#e34a33", "COLLISION": "#000000",
}


def nearest_index(route, x, y):
    return min(range(len(route)), key=lambda i: (route[i][0] - x) ** 2 + (route[i][1] - y) ** 2)


def simulate(sc):
    route = sc.route_points()
    base = sc.speed_limit

    def plan_limit(x):
        v = base
        for z in sc.zones:
            if z.x_start - 12.0 <= x <= z.x_end:
                v = min(v, z.limit * 0.85)
        return v

    v_prof = speed_profile_2pass(route, base)
    if sc.zones:
        v_prof = apply_speed_limits(route, v_prof, plan_limit, a_dec=1.2)

    pp, lon, beh, avoider = PurePursuit(), LongPI(), Behavior(base), None  # (실험용 lattice는 src/experimental 로 이동됨)
    ex, ey, eh, ev = sc.ego_start[0], sc.ego_start[1], sc.ego_start[2], 0.0
    dt, t, spawn_t = 0.02, 0.0, {}
    traj, actor_tracks, collided, astate = [], {}, False, {}

    while t < sc.duration:
        objs = []
        for a in sc.actors:
            if not spawned(a, t, ex, ey, spawn_t):
                continue
            if a.id not in astate:
                astate[a.id] = init_actor_state(a)
            st = astate[a.id]
            step_actor(a, st, ex, ey, dt)
            objs.append(Obj(a.id, st["x"], st["y"], 0, st["hd"], st["sp"], a.size[0], a.size[1], a.size[2]))
            actor_tracks.setdefault(a.id, {"type": a.type, "size": a.size, "pts": []})["pts"].append((st["x"], st["y"]))
        tl_state = 0
        ahead = [lt for lt in sc.lights if lt.stop_line_x > ex - 5]
        if ahead:
            lt = min(ahead, key=lambda l: l.stop_line_x)
            tl_state = tl_state_at(lt, t)
        s = State(x=ex, y=ey, heading=eh, speed=ev, objects=objs, tl_state=tl_state)

        bi = nearest_index(route, ex, ey)
        fwd = route[bi:] or route[-1:]
        vp = v_prof[bi] if bi < len(v_prof) else base
        off, blocked, turn_av = avoider.plan(s, objs)
        avoiding = (abs(off) > 0.1) and not blocked
        beh.speed_limit = plan_limit(ex)
        sl = sc.stop_line_ahead(ex)
        tl_stop = (sl - ex) if (sl is not None and tl_state == TL_RED) else None
        v_cmd, _, turn_beh, reason = beh.plan(s, vp, tl_stop_dist=tl_stop, lane_offset=off, avoiding=avoiding)
        if avoiding and reason == "LANE_KEEP":
            reason = "AVOID"
        steer = pp.steer(ex, ey, eh, ev, offset_path(fwd, off))
        accel = lon.accel(v_cmd, ev, dt)

        ev = max(0.0, ev + accel * dt)
        eh += (ev / L) * math.tan(steer) * dt
        ex += ev * math.cos(eh) * dt
        ey += ev * math.sin(eh) * dt
        traj.append((ex, ey, ev, reason))

        for o in objs:
            if math.hypot(o.x - ex, o.y - ey) < 1.5 + max(o.length, o.width) * 0.5:
                collided = True
        if collided or math.hypot(sc.ego_goal[0] - ex, sc.ego_goal[1] - ey) < 3.0:
            break
        t += dt
    return route, traj, actor_tracks, collided, sc


def draw(ax, sc, route, traj, actor_tracks, collided):
    xs = [p[0] for p in traj]
    # 차선/도로
    xmax = max(xs) + 15 if xs else 200
    ax.axhline(0, color="#e6b800", ls="--", lw=1, zorder=0)
    for yl in (-1.75, 1.75):
        ax.axhline(yl, color="#bbb", ls="-", lw=1, zorder=0)
    for yl in (-3.5, 3.5):
        ax.axhline(yl, color="#ddd", ls="-", lw=0.8, zorder=0)
    # ego 궤적(상태색)
    for i in range(1, len(traj)):
        c = STATE_COLOR.get(traj[i][3], "#2c7fb8")
        ax.plot([traj[i - 1][0], traj[i][0]], [traj[i - 1][1], traj[i][1]], color=c, lw=2.6, zorder=3)
    # 액터
    for aid, tr in actor_tracks.items():
        pts = tr["pts"]
        px = [p[0] for p in pts]; py = [p[1] for p in pts]
        typ = tr["type"]; l, w = tr["size"][0], tr["size"][1]
        if typ == "obstacle" or (len(pts) and abs(px[-1] - px[0]) < 1):
            ax.add_patch(Rectangle((px[-1] - l / 2, py[-1] - w / 2), l, w, color="#555", zorder=4))
        else:
            col = "#d7301f" if typ == "pedestrian" else "#6a51a3"
            ax.plot(px, py, color=col, lw=1.2, ls=":", zorder=2)
            ax.scatter([px[-1]], [py[-1]], color=col, s=40, zorder=4,
                       marker=("*" if typ == "pedestrian" else "s"))
    ax.scatter([route[0][0]], [route[0][1]], color="green", s=60, zorder=5, label="출발")
    ax.scatter([sc.ego_goal[0]], [sc.ego_goal[1]], color="red", s=120, marker="*", zorder=5, label="목표")
    # 보호구역
    for z in sc.zones:
        ax.axvspan(z.x_start, z.x_end, color="#fff2ae", alpha=0.6, zorder=0)
    status = "충돌!" if collided else "완주"
    ax.set_title(f"{sc.name}  [{status}]", fontsize=11)
    ax.set_xlim(-10, xmax)
    ax.set_ylim(-7, 7)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.grid(alpha=0.2)


def main():
    args = sys.argv[1:]
    out = args[0] if args and args[0].endswith(".png") else "runs.png"
    scen = [a for a in args if a.endswith(".json")] or [
        "scenarios/static_obstacle.json", "scenarios/ped_crossing.json",
        "scenarios/school_zone.json", "scenarios/signal_compliance.json",
    ]
    try:
        plt.rcParams["font.family"] = "NanumGothic"
    except Exception:
        pass
    plt.rcParams["axes.unicode_minus"] = False
    n = len(scen)
    rows = (n + 1) // 2
    fig, axes = plt.subplots(rows, 2, figsize=(15, 3.2 * rows))
    axes = axes.flatten() if n > 1 else [axes]
    for ax, path in zip(axes, scen):
        route, traj, tracks, collided, sc = simulate(Scenario.load(path))
        draw(ax, sc, route, traj, tracks, collided)
    for ax in axes[len(scen):]:
        ax.axis("off")
    axes[0].legend(loc="upper left", fontsize=8)
    fig.suptitle("HL-FMA 2026 제어스택 — 시나리오별 주행 궤적 (파랑=차선유지 주황=회피 빨강=정지/양보 초록=추종)", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out, dpi=110)
    print("저장:", out)


if __name__ == "__main__":
    main()
