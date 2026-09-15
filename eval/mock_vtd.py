"""
오프라인 시뮬 + 자동 채점 — VTD 없이 9910 프로토콜로 시나리오를 재생하고 주행을 평가한다.

상태 기반 액터: Ego 거리 트리거로 속도변경/차선변경 지원(공식 v1~v7 재현).
  motion:
    {"kind":"static","pos":[x,y]}
    {"kind":"lane","x0":..,"y0":..,"speed":v}     # 차선 주행(+x), y0=차선
    {"kind":"crossing","pos":[x,y],"vel":[vx,vy]} # 보행자 횡단
    {"kind":"waypoints","pts":[...],"speed":v}
  events: [{"ego_within":30,"set_speed":13.9,"rate":4.0,"lane_change":0.0,"lane_time":3.0}]

종료: ego 목표도달 / 충돌 / 시간초과. 사용: python3 mock_vtd.py --scenario X.json
"""
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "src"))
import argparse
import math
import socket
import time

from vtd_io import pack_data, parse_ctrl, CTRL_SIZE, Obj, TL_UNSET, TL_GREEN, TL_YELLOW, TL_RED
from scenario import Scenario
from evaluate import Scorer

WHEELBASE = 2.95
TLMAP = {"UNSET": TL_UNSET, "GREEN": TL_GREEN, "YELLOW": TL_YELLOW, "RED": TL_RED}


def tl_state_at(light, t):
    if not light.phases:
        return TL_UNSET
    period = sum(p[1] for p in light.phases)
    tt = t % period
    acc = 0.0
    for name, dur in light.phases:
        if tt < acc + dur:
            return TLMAP[name]
        acc += dur
    return TLMAP[light.phases[-1][0]]


def init_actor_state(a):
    m = a.motion
    k = m["kind"]
    if k == "static":
        # ★`hd`(방위[rad])를 받는다 — 없으면 예전대로 0. 합성 직선 판은 도로가 +x 라 0 이
        #   맞지만, **실제 지도 경로를 그대로 먹이는 판**에서는 0 이 거짓이다. 채점기는
        #   상자 대 상자(SAT)로 재므로(evaluate.py `oriented_gap`), 12.99m 버스를 방위 0 으로
        #   두면 상자가 도로를 가로질러 눕는다. 실측 2026-09-12 `HL_FMA_2026_VIDEO_A_EXACT`
        #   재현판: 경로에서 5.8m 옆에 선 버스와 **없는 충돌**(-50)이 찍혔다.
        x, y = m["pos"]
        return dict(x=x, y=y, hd=float(m.get("hd", 0.0)), sp=0.0, y_t=y, sp_t=0.0,
                    rate=4.0, wp_s=0.0, done=set())
    if k == "lane":
        return dict(x=m["x0"], y=m["y0"], hd=0.0, sp=m.get("speed", 0.0),
                    y_t=m["y0"], sp_t=m.get("speed", 0.0), rate=4.0, wp_s=0.0, done=set())
    if k in ("constant", "crossing"):
        x, y = m["pos"]; vx, vy = m["vel"]; sp = math.hypot(vx, vy)
        return dict(x=x, y=y, hd=math.atan2(vy, vx), sp=sp, y_t=y, sp_t=sp, rate=4.0, wp_s=0.0, done=set())
    if k == "waypoints":
        p0 = m["pts"][0]; return dict(x=p0[0], y=p0[1], hd=0.0, sp=m.get("speed", 1.0),
                                      y_t=p0[1], sp_t=m.get("speed", 1.0), rate=4.0, wp_s=0.0, done=set())
    x, y = m.get("pos", [0, 0]); return dict(x=x, y=y, hd=0.0, sp=0.0, y_t=y, sp_t=0.0, rate=4.0, wp_s=0.0, done=set())


def step_actor(a, st, ex, ey, dt):
    """트리거 적용 + 상태 적분."""
    # 이벤트 트리거 (ego 거리)
    for i, ev in enumerate(a.events):
        if i in st["done"]:
            continue
        if math.hypot(ex - st["x"], ey - st["y"]) <= ev.get("ego_within", 0):
            st["done"].add(i)
            if "set_speed" in ev:
                st["sp_t"] = ev["set_speed"]; st["rate"] = ev.get("rate", 4.0)
            if "lane_change" in ev:
                st["y_t"] = ev["lane_change"]; st["lane_time"] = ev.get("lane_time", 3.0)
    # 속도 -> 목표
    ds = st["sp_t"] - st["sp"]
    st["sp"] += max(-st["rate"] * dt, min(st["rate"] * dt, ds))
    # 차선 -> 목표(횡)
    #  ⚠️ crossing(횡단)은 vel 로 y 를 적분하므로 여기서 y_t 로 끌어당기면 안 된다.
    #     (버그였음: 보행자가 제자리에서 안 건너가 ego 가 영원히 양보 대기 — 2026-08-15)
    if a.motion["kind"] != "crossing" and abs(st["y_t"] - st["y"]) > 1e-3:
        vy = (st["y_t"] - st["y"])
        max_dy = 3.5 / st.get("lane_time", 3.0) * dt
        st["y"] += max(-max_dy, min(max_dy, vy))
    # 위치 적분
    if a.motion["kind"] == "waypoints":
        st["wp_s"] += st["sp"] * dt
        pts = a.motion["pts"]; acc = 0.0
        st["x"], st["y"] = pts[-1]
        for i in range(1, len(pts)):
            seg = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            if acc + seg >= st["wp_s"]:
                r = (st["wp_s"] - acc) / seg if seg > 0 else 0.0
                st["x"] = pts[i - 1][0] + r * (pts[i][0] - pts[i - 1][0])
                st["y"] = pts[i - 1][1] + r * (pts[i][1] - pts[i - 1][1])
                break
            acc += seg
    else:
        st["x"] += st["sp"] * math.cos(st["hd"]) * dt
        # crossing은 vel 방향(hd)에 y성분 포함; lane/static은 hd=0 -> y는 차선변경으로만
        if a.motion["kind"] in ("constant", "crossing"):
            st["y"] += st["sp"] * math.sin(st["hd"]) * dt


def spawned(a, t, ex, ey, spawn_t):
    if a.id in spawn_t:
        return True
    sp = a.spawn
    ok = (("at_time" in sp and t >= sp["at_time"])
          or ("ego_x" in sp and ex >= sp["ego_x"])
          or ("ego_within" in sp and math.hypot(ex - sp["of"][0], ey - sp["of"][1]) <= sp["ego_within"]))
    if ok:
        spawn_t[a.id] = t
    return ok


def run(scenario_path, host="127.0.0.1", port=9910, hz=50.0):
    sc = Scenario.load(scenario_path)
    scorer = Scorer(sc)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port)); srv.listen(1)
    print(f"[mock] '{sc.name}' | 9910 대기 {host}:{port} ...")
    conn, addr = srv.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    conn.setblocking(False)
    print(f"[mock] 접속 {addr} | 제한 {sc.speed_limit*3.6:.0f}km/h")

    ex, ey, eh = sc.ego_start[0], sc.ego_start[1], sc.ego_start[2]
    ev = 0.0
    dt = 1.0 / hz
    t = 0.0
    buf = b""
    spawn_t = {}
    astate = {}
    done_rp = set()
    steer = accel = 0.0
    end = ""

    while t < sc.duration:
        try:
            while True:
                d = conn.recv(4096)
                if not d:
                    end = "클라이언트 종료"; break
                buf += d
        except BlockingIOError:
            pass
        if end:
            break
        while len(buf) >= CTRL_SIZE:
            steer, accel, _ts = parse_ctrl(buf[:CTRL_SIZE]); buf = buf[CTRL_SIZE:]

        ev = max(0.0, ev + accel * dt)
        eh += (ev / WHEELBASE) * math.tan(steer) * dt
        ex += ev * math.cos(eh) * dt
        ey += ev * math.sin(eh) * dt

        # 리스폰 시뮬(경로이탈 순간이동) — 좌표 확 점프
        for i, rp in enumerate(sc.respawns):
            if i not in done_rp and t >= rp.get("at_time", 1e9):
                done_rp.add(i)
                ex, ey = rp["to"][0], rp["to"][1]
                if len(rp["to"]) > 2:
                    eh = rp["to"][2]
                ev = 0.0
                print(f"[mock] >>> RESPAWN t={t:.1f} -> ({ex:.1f},{ey:.1f})")

        objs = []
        dump = []
        for a in sc.actors:
            if not spawned(a, t, ex, ey, spawn_t):
                continue
            if a.id not in astate:
                astate[a.id] = init_actor_state(a)
            st = astate[a.id]
            step_actor(a, st, ex, ey, dt)
            objs.append(Obj(a.id, st["x"], st["y"], 0.0, st["hd"], st["sp"],
                            a.size[0], a.size[1], a.size[2]))
            dump.append({"id": a.id, "type": a.type, "x": st["x"], "y": st["y"], "size": a.size,
                         "hd": st["hd"]})

        tl_id, tl_state = -1, TL_UNSET
        ahead = [lt for lt in sc.lights if lt.stop_line_x > ex - 5]
        if ahead:
            lt = min(ahead, key=lambda l: l.stop_line_x)
            tl_id, tl_state = lt.id, tl_state_at(lt, t)

        scorer.update(t, ex, ey, ev, dump, tl_state, dt, eh=eh)
        if scorer.collided:
            end = "충돌 발생"; break
        if scorer.reached:
            end = "목표 도달"; break

        try:
            conn.sendall(pack_data(ex, ey, 0.0, eh, 0.0, 0.0, objs[:30], tl_id, tl_state))
        except (BrokenPipeError, ConnectionResetError):
            end = "연결 끊김"; break

        if int(round(t * hz)) % int(hz) == 0:
            print(f"[mock] t={t:5.1f} ego x={ex:6.1f} v={ev*3.6:4.0f}km/h objects={len(objs)}")
        time.sleep(dt)
        t += dt

    if not end:
        end = "시간초과"
    print(f"[mock] 종료: {end}\n")
    print(scorer.report(t))
    try:
        conn.close()
    except OSError:
        pass
    srv.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="scenarios/ped_crossing.json")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9910)
    args = ap.parse_args()
    run(args.scenario, args.host, args.port)
