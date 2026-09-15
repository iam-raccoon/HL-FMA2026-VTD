"""제어PC 진입점 — CLI / VTD 연결 / 수신 루프 / 제어 송신 / 로깅 / 종료.

'어떻게 달릴지'(판단·제어)는 전부 drive.DrivingStack 에 있다. 여기는 I/O만 담당한다.

실전:   python3 main.py --scenario ../routes/official_v1_v6.json
        (--host 를 안 주면 자기 IP 대역으로 대회망/연습망을 판별한다 — src/netcfg.py)
오프라인: (터미널1) python3 ../eval/mock_vtd.py --scenario ../eval/scenarios/ped_crossing.json
          (터미널2) python3 main.py --host 127.0.0.1 --scenario ../eval/scenarios/ped_crossing.json
"""
import argparse
import json
import math
import os
import time

from netcfg import default_host, describe as net_describe
from vtd_io import VTDLink, TS_RIGHT
from scenario import Scenario
from drive import DrivingStack, Command
from run_logger import RunLogger

GOAL_BRAKE_HOLD = 4.0      # 목표 도달 후 브레이크를 물고 있는 시간[s]
# ★정차 중 표시등. **비상등은 VTD 에 없다**(vtd_io 주석의 디스어셈블 참고) —
#   우측 가장자리에 붙여 세운 상태를 알리는 데 쓸 수 있는 건 우측 지시등뿐이다.
#   GOAL_STOP_SIG=0 이면 종전대로 소등한다.
GOAL_STOP_SIG = int(os.environ.get("GOAL_STOP_SIG", TS_RIGHT))
BRAKE_ACCEL = -4.0
# 판단이 예외로 죽은 프레임에 줄 감속. 급제동(-4.0)까지는 안 가고 확실히 줄인다.
SAFE_DECEL = -2.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=None,
                    help="VTD 주소. 안 주면 자기 IP 대역으로 판별한다 "
                         "(대회 192.168.50.11 / 연습 192.168.100.1). "
                         "VTD_HOST 환경변수로도 강제 가능")
    ap.add_argument("--port", type=int, default=9910)
    ap.add_argument("--scenario", default=None, help="지도지식(경로/제한/보호구역/정지선) JSON")
    ap.add_argument("--speed-limit", type=float, default=8.33)
    ap.add_argument("--max-speed", type=float, default=None,
                    help="상한[m/s]. 지도 제한보다 느리게 달린다(11.1=40km/h). "
                         "차로 안 정지물이 있는 코스에서 VTD 프레임 붕괴를 피할 때 쓴다")
    ap.add_argument("--log-csv", default=None, help="매 프레임 주행 기록(채점·정지선 매핑용)")
    ap.add_argument("--tl-map", default=None,
                    help="맵 전체 신호 정지선 DB(routes/tl_map_*.json). "
                         "대회 당일 새 경로여도 신호를 알 수 있게 해준다")
    ap.add_argument("--crosswalks", default=None,
                    help="횡단보도 DB(routes/crosswalks.json). **어린이보호구역의 무신호 "
                         "횡단보도**에서 보행자 유무와 관계없이 일시정지하기 위한 것 "
                         "[도로교통법 제27조⑦]. 없으면 그 규칙만 꺼진다")
    ap.add_argument("--stoplines", default=None,
                    help="고아 정지선 DB(routes/stoplines.json). 신호등도 횡단보도도 없는 "
                         "**도색 정지선**에서 일시정지한다 [시행규칙 별표6 노면표시 530]. "
                         "없으면 그 규칙만 꺼진다")
    ap.add_argument("--stoplines-all", default=None,
                    help="도색 정지선 **전부**(routes/stoplines_all.json, 710개). "
                         "VTD 가 맵에 없는 신호 id 를 보고할 때(실측: tl 80·82) 정지선 "
                         "위치를 여기서 빌린다. 없으면 그 신호는 RED_STOP_BLIND 로 "
                         "**그 자리에** 선다 — 교차로 한복판 정지가 된다")
    ap.add_argument("--lane-plan", default=None,
                    help="경로점별 차로 계획(routes/lane_plan_*.json). 지정차로(회전 차로)·"
                         "중앙선 점선 여부·차로별 여유. 도교법 준수에 필수")
    ap.add_argument("--no-turn-lane", action="store_true",
                    help="회전 전 지정차로 진입을 끈다. 기본은 켜짐(도교법 제25조) — "
                         "다만 공식 경로가 직진차로로 좌회전해서, 지키면 그 회전에서 "
                         "VTD 경로이탈 리스폰이 1건 붙는다")
    ap.add_argument("--allow-centerline-escape", action="store_true", default=None,
                    help="완주 우선 비정상/실험 전략. 중앙선을 탈출 공간으로 허용한다. "
                         "평가상 중앙선 침범이며 면책되지 않는다. 환경변수 "
                         "ALLOW_CENTERLINE_ESCAPE=1 로도 켤 수 있다")
    args = ap.parse_args()
    if args.host is None:
        args.host = default_host()
        print(f"[main] VTD 주소 자동판별 -> {net_describe()}")

    sc = Scenario.load(args.scenario) if args.scenario else None
    # 정지선: 맵 전체 DB(xodr 계산)를 깔고, 시나리오의 실측값이 있으면 그걸 우선한다.
    tl_stops = {}
    if args.tl_map:
        tl_stops.update({int(k): v for k, v in json.load(open(args.tl_map, encoding="utf-8")).items()})
    crosswalks = None
    if args.crosswalks:
        crosswalks = json.load(open(args.crosswalks, encoding="utf-8"))["crosswalks"]
    orphan_stops = None
    if args.stoplines:
        orphan_stops = json.load(open(args.stoplines, encoding="utf-8"))["stoplines"]
    all_stops = None
    if args.stoplines_all:
        all_stops = json.load(open(args.stoplines_all, encoding="utf-8"))["stoplines"]
    lane_plan = None
    if args.lane_plan:
        lane_plan = json.load(open(args.lane_plan, encoding="utf-8"))["pts"]
    stack = DrivingStack(scenario=sc, base_limit=args.speed_limit, tl_stops_extra=tl_stops,
                         max_speed=args.max_speed,
                         lane_plan=lane_plan, turn_lane=not args.no_turn_lane,
                         crosswalks=crosswalks, stoplines=orphan_stops,
                         stoplines_all=all_stops,
                         allow_centerline_escape=args.allow_centerline_escape)

    link = VTDLink(args.host, args.port)
    link.connect()
    print(f"[main] connected {args.host}:{args.port} | route pts={len(stack.route)} "
          f"{stack.total:.0f}m | limit={stack.base_limit*3.6:.0f}km/h | "
          f"신호정지선 {len(stack.tl_stops)}개 | "
          f"차로계획 {'없음' if not lane_plan else str(sum(1 for r in lane_plan if r)) + '점'}")

    logger = RunLogger(args.log_csv)
    last = t_start = time.time()
    n = 0
    goal_since = None
    step_errors = 0
    last_cmd = None

    try:
        while True:
            _t_wait0 = time.time()
            s = link.recv_state()
            _t_recv = time.time()
            if s is None:
                # ⚠️ 링크가 끊기면 VTD는 '마지막 제어값'을 계속 유지한다(조향 물린 채 폭주).
                #    끊기기 직전에 정지명령을 한 번 넣어 안전측으로 남긴다.
                try:
                    link.send_ctrl(0.0, BRAKE_ACCEL, 0)
                except Exception:
                    pass
                if step_errors:
                    print(f"[main] ❌❌ step 예외 총 {step_errors}프레임 "
                          f"— 이 판의 판단은 일부 비었다")
                print("[main] link closed")
                break

            now = time.time()
            dt = max(1e-3, now - last)
            last = now
            if s.respawned:
                print(f"[main] ⚠ RESPAWN 감지 -> 재정위 (x={s.x:.1f} y={s.y:.1f})")

            # ★판단이 예외로 죽어도 **주행은 계속한다.** 실측 2026-08-24 주변교통 판:
            #   overtake.py 의 `block[2]` 가 NoneType 이라 프로세스가 통째로 내려갔고
            #   그 판이 끝났다 — 대회였으면 그대로 실격이다. 한 프레임 판단이 실패해도
            #   차는 굴러가고 있으므로, **감속하며 직전 조향을 유지**하는 게 맞다.
            #   ⚠️ 조용히 넘기지 않는다. 매번 크게 찍고 카운트해서 끝에 요약한다 —
            #      숨기면 '왜 이상하게 갔는지'를 영영 모른다.
            try:
                cmd = stack.step(s, dt, now)
                last_cmd = cmd
            except Exception as e:
                step_errors += 1
                if step_errors <= 5 or step_errors % 50 == 0:
                    import traceback
                    print(f"[main] ❌❌ step 예외 #{step_errors} @({s.x:.1f},{s.y:.1f}) "
                          f"{type(e).__name__}: {e}")
                    traceback.print_exc()
                cmd = Command(steer=(last_cmd.steer if last_cmd else 0.0),
                              accel=SAFE_DECEL, turn=0, reason="STEP_ERROR")

            if cmd.goal_reached and s.speed < 0.3:
                # ⚠️ 즉시 종료하면 제어입력이 끊겨 VTD 물리로 경사에서 슬금슬금 굴러간다.
                #    잠시 브레이크를 물고 있다가 종료(종료 후엔 시뮬 Stop 권장).
                if goal_since is None:
                    goal_since = now
                    print(f"[main] ✅ 목표 도달 (x={s.x:.1f} y={s.y:.1f}, {now-t_start:.0f}s)")
                # ★붙여 세운 뒤에도 우측 지시등을 켜 둔다. 비상등을 켜고 싶지만
                #   프로토콜에 없다(3 을 82프레임 보냈으나 화면 무반응 — 실측).
                link.send_ctrl(0.0, BRAKE_ACCEL, GOAL_STOP_SIG)
                cmd.turn = GOAL_STOP_SIG
                logger.log(now - t_start, s, cmd)
                if now - goal_since > GOAL_BRAKE_HOLD:
                    break
                continue

            link.send_ctrl(cmd.steer, cmd.accel, cmd.turn)
            logger.log(now - t_start, s, cmd)

            # ★프레임 시간을 '기다린 시간'과 '계산한 시간'으로 쪼개 기록한다.
            #   왜: v7 코스에서 프레임율이 25Hz -> 3.6Hz 로 무너졌는데, 양쪽 PC 모두
            #   CPU 가 놀고 있어 어디가 병목인지 추측만 하고 있었다(2026-08-16).
            #   wait 가 크면 VTD 가 안 보내는 것이고, calc 가 크면 우리 탓이다.
            wait_ms = (_t_recv - _t_wait0) * 1e3
            calc_ms = (time.time() - _t_recv) * 1e3
            n += 1
            if n % 25 == 0:
                ts = {0: "-", 1: "L", 2: "R"}.get(cmd.turn, "?")
                print(f"x={s.x:8.1f} y={s.y:8.1f} v={s.speed*3.6:5.1f}km/h -> "
                      f"vcmd={cmd.v_cmd*3.6:5.1f} a={cmd.accel:+.2f} "
                      f"steer={math.degrees(cmd.steer):+5.1f}deg off={cmd.lane_offset:+.1f} "
                      f"sig={ts} obj={len(s.objects)} tl={s.tl_id}/{s.tl_state} [{cmd.reason}]"
                      f" | wait {wait_ms:6.1f}ms calc {calc_ms:5.1f}ms")
    finally:
        logger.close()
        link.close()


if __name__ == "__main__":
    main()
