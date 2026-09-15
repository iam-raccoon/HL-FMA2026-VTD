"""주행 판단·제어 스택 — 상태(State) 하나를 받아 제어명령을 만든다.

main.py는 I/O(연결·수신·송신·로깅·종료)만 담당하고, '어떻게 달릴지'는 전부 여기에 있다.
소켓이 없어도 State만 만들어 넣으면 되므로 유닛테스트가 가능하다.

파이프라인(매 프레임):
  경로 최근접점 -> 목표속도 -> 객체를 경로기준으로 투영 -> 추월 FSM -> 행동(도교법)
  -> 추월 중 속도제한 -> 무한정지 탈출 -> 조향(Pure Pursuit) + 종방향 PI
"""
import bisect
import math
import os
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from control import PurePursuit, speed_profile_2pass, apply_speed_limits, LongPI
from behavior import Behavior
from overtake import Overtaker
from route_frame import RouteFrame, offset_path
from vtd_io import (TL_RED, TL_YELLOW, TL_GREEN, TL_GREEN_LEFT, TS_LEFT, TS_RIGHT, is_vru, classify_object, ObjectClass,
                    VRU_MAX_SPEED)


def route_extent(o, path_hdg):
    """물체의 (길이, 폭)을 **경로 축 기준 외접 크기**로 바꾼다.

    ⚠️ 이걸 안 하면 굽은 길에 비스듬히 선 차의 횡폭을 과소평가한다. 안전망은
       `|fy| - 내반폭 - 폭/2` 로 통로를 재는데, 그 폭이 '경로와 나란한 상자'를
       가정한 값이라서다. 실측(2026-08-16 EV_LEADBRAKE, 곡선 위 정지차):
       계산 여유 +0.38m 로 '지나가도 된다'고 판단했지만 실제 차체 여유는 -0.06m,
       그대로 파고들어 **접촉 2515프레임**이 났다. 물체가 경로에 14° 틀어져 있었고,
       그 14° 가 만든 횡폭 차이가 정확히 0.5m 였다.

    회전한 직사각형의 축정렬 외접상자를 쓴다 — 조금 보수적이지만 그게 맞는 방향이다.
    """
    c, s = abs(math.cos(o.heading - path_hdg)), abs(math.sin(o.heading - path_hdg))
    return (o.length * c + o.width * s,          # 경로 종방향 크기
            o.length * s + o.width * c)          # 경로 횡방향 크기


def straight_route(length=400.0, step=1.0):
    return [(i * step, 0.0) for i in range(int(length / step))]


def _env_enabled(name):
    """명시적인 opt-in 환경변수만 참으로 읽는다."""
    return os.environ.get(name, "0").strip().lower() in ("1", "true", "yes", "on")


START_SNAP = 20.0   # 첫 프레임에 경로 시작점이 이 안이면 무조건 인덱스 0 에서 출발


def nearest_index(route, x, y, prev=None, back=15, ahead=80):
    """경로상 최근접 인덱스.

    ⚠️ 매 프레임 '경로 전체'에서 찾으면, 코스가 교차로 등에서 자기 경로 근처를 다시 지날 때
       인덱스가 엉뚱한 구간으로 튀어 **직진 중 갑자기 핸들을 꺾는다**. 직전 인덱스 주변만
       보고(연속성 보장), 그 창에서도 너무 멀면(이탈/리스폰) 그때만 전역 재탐색한다.
    """
    def d2(i):
        return (route[i][0] - x) ** 2 + (route[i][1] - y) ** 2
    if prev is None:
        # ★첫 프레임은 **코스 출발점**이다. 전역 최근접으로 잡으면 안 된다 — 코스가
        #   출발점 근처로 되돌아오면(순환 코스) 뒤쪽 구간을 집어 **앞부분을 통째로
        #   건너뛴다.** 실측 2026-08-19 새 코스 A: 스폰 지점에서 rt[0] 이 2.96m 인데
        #   rt[1724] 도 2.96m 라 뒤쪽을 집었고, 2.4km 를 안 간 채 완주로 끝났다
        #   (경로 커버리지 56%). 공식 v1~v7·WP9 는 순환이 아니라 안 드러났었다.
        #   ego 는 언제나 경로 시작점에 놓인다(시나리오 PathRef StartS≈3m).
        return 0 if d2(0) <= START_SNAP ** 2 else min(range(len(route)), key=d2)
    lo, hi = max(0, prev - back), min(len(route), prev + ahead)
    i = min(range(lo, hi), key=d2)
    if d2(i) > 15.0 ** 2:                    # 창 밖으로 벗어남 -> 전역 재획득
        i = min(range(len(route)), key=d2)
    return i


@dataclass
class Command:
    """한 프레임의 제어 출력 + 로깅/판정에 필요한 부가정보."""
    steer: float = 0.0
    accel: float = 0.0
    turn: int = 0
    v_cmd: float = 0.0
    reason: str = "LANE_KEEP"
    goal_reached: bool = False
    lane_offset: float = 0.0
    ov_state: str = "FOLLOW"
    cap_by: str = "SPEED_LIMIT"     # 실제로 속도를 잡은 제약(reason 과 다를 수 있다)
    rf_objs: List[tuple] = field(default_factory=list)
    # ★진단용 분해값. `lane_offset` 하나만 남기면 '왜 그리로 갔는지'를 못 읽는다.
    #   실측 2026-08-18 EV_COMBO_PASSPED 를 푼 게 이 세 칸이다(로거가 CSV 로 남긴다).
    #   ⚠️ run_logger 는 getattr(...,0.0) 로 읽는다 — 필드를 지우면 **조용히 0** 이 찍히고
    #      진단이 통째로 헛돈다. 실제로 한 번 유실돼서 오프라인 판이 전부 0 이었다.
    d_ego: float = 0.0              # 경로기준 내 횡위치(실제)
    base: float = 0.0               # 지정차로 유지용 기본 오프셋
    nudge: float = 0.0              # 장애물 횡회피 오프셋


class DrivingStack:
    # 추월 중에는 전방주시거리를 줄여 오프셋 경로를 '바짝' 따라간다.
    #  ⚠️ 기본 lookahead(4m)로는 횡이동이 느려, 정지차 8.7m 뒤에서 시작한 추월이
    #     통과 시점에 횡 2.0m밖에 못 벌어 실여유 0.11m였다(2026-08-14 실측).
    LOOKAHEAD_NORMAL = 4.0
    LOOKAHEAD_PASS = 2.2
    # ★급커브에서는 전방주시를 짧게 — 길면 안쪽으로 파고든다(코너컷).
    #   문턱은 **공식 직선 코스가 원리상 안 걸리도록** 잡았다(v1·v7 최대 4.8°/10m).
    # ★급커브 전방주시. **짧을수록 총량은 좋아지고 최대값은 나빠진다** — 단조롭다.
    #   실측 2026-08-27 공식 WP9(주변교통 없어 거의 결정적, 각 1판):
    #       4.0m(원래)  최대 2.70m · >0.5m 8.1% · >0.8m 1.9% · 차선밟기 5.2s
    #       3.0m        최대 3.00m · >0.5m 6.1% · >0.8m 1.4%
    #       2.2m        최대 3.35m · >0.5m 4.2% · >0.8m 0.9% · 차선밟기 3.4s
    #   채점은 **초 단위**(차선밟기)로 세므로 총량이 기준이다. 2.2m 가 차선밟기를
    #   1.8초 줄이고 중앙선을 0.2초 늘린다 — 순증 -1.6초.
    #   ⚠️ 남은 최대값 3.35m 은 (441,-416) **86° 급회전 한 곳**이고, 파고드는 게 아니라
    #      **크게 돌아 나가는 오버슛**이다(d_ego -0.6 -> +3.3 -> 0.0). 더 줄이려면
    #      전방주시가 아니라 **회전 탈출 가속**을 봐야 한다 — 아직 안 봤다.
    LOOKAHEAD_TURN = 2.2
    TURN_CURVE_DEG = 20.0       # 10m 앞까지 방위가 이만큼 꺾이면 급커브[도]
    TURN_SIG_HOLD_DPS = 6.0     # 자차 회전율이 이 아래로 떨어질 때까지 지시등을 문다[도/s]
    TURN_CURVE_LOOK = 10.0
    PASS_SPEED_CAP = 3.0        # 추월 중 서행(횡이동 끝나기 전 가속하면 옆구리 접촉)
    STALL_SEC = 30.0            # 신호 외 이유로 이만큼 멈춰있으면 서행 탈출
    # ★'정상적인 대기' — 이 이유로 서 있는 동안은 탈출 타이머를 세지 않는다.
    #   적신호와 같은 취급이다. 탈출이 교차로로 밀어넣으면 안 된다.
    HOLD_REASONS = ("YIELD_ONCOMING", "WAIT_LEFT_ARROW", "WAIT_GREEN", "WAIT_LEFT_BLIND")
    FROZEN_SEC = 75.0           # 적신호 포함해도 이만큼 못 움직이면 뭔가 잘못된 것
    FROZEN_MOVE = 3.0           # 이만큼 움직이면 '가고 있다'로 보고 동결 시계를 되돌린다[m]
    STALL_V = 1.5
    HORIZON_PTS = 90            # 추종할 전방 경로 점 수
    RESPAWN_WINDOW = 40.0       # 이 시간 안에
    RESPAWN_TRIP = 2            # 리스폰이 이만큼 나면 = 추종 실패 중
    RESPAWN_SLOW = 0.55         # 그때 제한속도의 이 비율로 서행(이탈 재발 방지)
    ZONE_MARGIN = 0.92          # 보호구역 제한속도 여유(추적지연 대비). 0.85는 과도했음

    # ★작은 인레인 장애물 횡회피(nudge) — 연료통·라바콘처럼 '서진 않지만 밟으면 안 되는' 것.
    #  발견 경위(2026-08-15): 공식 v5의 Fuelcan01이 경로 중심선에서 **4cm**에 있는데
    #  차폭 절반이 0.94m라 그대로 밟고 지나갔다. 취약대상은 is_vru, 큰 차는 추월 FSM이
    #  담당하고, 그 사이 소형 사물은 이 nudge가 맡는다.
    #  강의 [70:57] "저 앞에 연료통 3개 ... 얘는 회피하려고 차선 이동을 해서 간 거 같고요"
    #  -> 연료통은 피해야 하는 장애물이 맞다.
    HALF_WIDTH = 0.943          # 차폭 1.886m의 절반
    # ★'밟아도 되는 노면인가' 도 `vtd_io.classify_object` 가 판정한다(문턱 0.25 동일).
    #   여기 상수로 두면 중앙 판정과 소리 없이 어긋난다 — 사람 판정이 그렇게 갈라졌었다.
    NUDGE_MAX_DIM = 1.2         # 이보다 크면 차 -> 추월 FSM 담당
    # 통과 여유. 이 지점 차선폭 3.30m/인접 lane+1 도 같은 방향 주행차선(점선)이라
    # 차체가 0.8m쯤 걸쳐도 중앙선 침범이 아니다 -> 여유를 넉넉히 준다(xodr 실측 2026-08-15).
    NUDGE_CLEAR = 0.40
    # 이 거리 안으로 들어오면 비키기 시작. 차로 하나(3.3m)를 1.2m/s 로 옮기는 데
    # 2.8초가 걸리므로 20m(=2.5초)로는 다 못 벌린다 -> 실여유 0.11m 사고(EV_PED).
    NUDGE_AHEAD = 40.0
    NUDGE_BEHIND = -3.0         # 뒤로 이만큼 지나가면 복귀
    NUDGE_MAX = 3.8             # 상한(차로 하나를 통째로 옮길 수 있어야 한다)
    NUDGE_LANES = 3             # 비킬 자리가 막혔을 때 **몇 차로까지** 더 내다볼지
    # ★★경로가 **사람이 서 있는 차로로 차선을 바꿔 들어가면** 지금 차로를 지키다 지나간 뒤 바꾼다
    #   (`_lc_hold_target`). 차로계획의 l(좌 여유)·r(우 여유)로 경로가 도로 위에서 옮겨 가는 양을 잰다.
    LC_HOLD_WIDTH_TOL = 0.5     # 지금~사람 사이 차도 폭(l+r)이 이보다 더 바뀌면 경로 쏠림으로 안 본다[m]
    LC_HOLD_MIN_DRIFT = 0.5     # 경로가 사람 쪽으로 이만큼은 옮겨 가야 '차선을 바꿔 들어가는 중'[m]
    LC_HOLD_LOOK = 4.0          # 유지 목표를 이만큼 앞 경로점에서 잰다(= LOOKAHEAD_NORMAL)[m]
    # ★차체 안전망(behavior 의 NARROW_BLOCK)과 **같은 식**으로 여유를 잰다.
    #   달라지면 nudge 가 안전망이 거부할 자리로 가라 하고, 안전망은 세우고,
    #   아무도 물러서지 않아 교착이 된다(실측 EV_COMBO_PASSPED 250초 정지).
    FIT_GAP_CAR = 0.35          # behavior.body_gap_min 과 같은 값
    FIT_GAP_PED = 1.2           # behavior.body_gap_ped 와 같은 값
    NUDGE_RATE = 1.2            # m/s. 오프셋 변화율 제한(급조향 방지)
    # ★★비켜 갈 때 **뒤·옆을 본다**(2026-09-10 코스 H (966,657) 접촉).
    #   `_lat_free` 는 앞만 본다(`NUDGE_BEHIND` = 뒤 3m). 서 있는 사람을 피하려고 우측으로
    #   3.0m 나가는 동안, 오른쪽 뒤 10.4m 에서 6.8m/s 로 오던 차가 옆에 붙어 **실여유
    #   -0.24m 로 긁었다**. 사용자: "차선 바꿀 때 뒷차를 봐야지 걍 들이밀어서 사고남".
    #   [법 제19조③ · 제38조] 진로변경은 뒤차의 정상 통행을 방해하면 안 된다.
    NUDGE_REAR_WATCH = 35.0     # 비킬 쪽 **뒤**로 이만큼 본다[m] — LC_REAR_WATCH 와 같은 값
    NUDGE_REAR_SLOW = 1.0       # 나보다 이만큼 이상 느린 차만 무시한다[m/s]
    # ★★**서 있는 차는 뒤에서 오는 차가 아니다**(2026-09-10 코스 H (1462,929) 135초 교착).
    #   내가 멈춰 있으면 `spd > ego_speed - 1.0` 이 `0 > -1.0` 이라 **내 뒤에 선 정체 차량이
    #   전부 '따라붙는 차'** 가 됐다. 그래서 사고현장 우회가 폭을 잡아 놓고도 한 발도 못 나갔다.
    #   사용자: "왜 안 되냐". 뒤에 **서 있는** 차는 내가 옆으로 비켜도 나를 덮치지 않는다.
    NUDGE_REAR_MIN_V = 1.0      # 뒤 물체는 이보다 빨라야 '오는 차'다[m/s] (JAM_STOPPED_V 와 같은 값)
    EGO_REAR_EDGE = 1.04        # 뒷축에서 뒷범퍼까지[m] (run_logger 의 EGO_CX·EGO_HALF_L 에서)
    NUDGE_SIDE_GAP = 0.5        # 비킨 자리에서 이 안으로 들어오면 '거기 차가 있다'[m]
    NUDGE_SPEED_CAP = 6.0       # 비키는 동안 서행(횡추종 정확도)
    JUNCTION_WAIT = 20.0        # 교차로에서 이만큼 갇히면 감점을 감수하고 앞지르기 허용
    GREEN_GRACE_S = 4.0         # 녹색으로 바뀐 뒤 이만큼은 정지선 안쪽 앞차를 '출발 대기 중'으로 본다(추월 대기 금지)
    V_LAG_S = 0.35              # 속도 추정(0.3초 창)의 가속 중 지연 보상[s] — 실측 E t=78: 추정 48.7 / 참 52
    OVT_AVOID_TTL = 5.0         # 추월 방향 배제의 시한[s]. 지나면 반대쪽을 다시 본다
    JUNC_LOOK = 60.0            # 전방 교차로를 이 거리까지 내다본다(우회전 일시정지용)
    JUNC_NO_PASS = 25.0         # 교차로가 이 안에 있으면 추월을 시작하지 않는다[m]
    DESPERATE_MIN_ROOM = 2.0    # 절박 모드로 나갈 때 그쪽 여유가 최소 이만큼(차폭 1.886+α)은 돼야 한다[m]
    # ★회전 차로에서는 앞차가 신호를 기다리는 것일 수 있다. 앞차를 장애물로 보고
    #   옆차로로 나갔다가 출발하면 다시 들어오는 동작은 불필요한 두 번의 진로변경이다.
    #   추월 감지거리(28m)+기동거리(20~30m)를 감안해 회전 60m 전부터 새 추월을 막는다.
    # ★★**좌·우를 가리지 않는다**(2026-09-08). 처음엔 우회전만 봤는데, 좌회전 대기는
    #   신호 한 주기를 통째로 기다리므로 오히려 **더 길게** 서 있다 — 오인 추월이 나기
    #   더 쉬운 쪽이 무방비였다. 실측(고치기 전, 우회전과 똑같은 배치를 뒤집어서):
    #     우회전 `FOLLOW` 유지·오프셋 0.00  /  좌회전 `WAIT` -> **3.15초 `PASS`**·**3.30m**
    #   넓히는 비용은 작다. 60m 창이 경로에서 차지하는 비율(전 코스 적분, 2026-09-08):
    #     우회전 A 20.9% · B 16.5% · D 15.6% · E 4.9% · G 26.0% · H 9.4%
    #     좌회전 A  2.1% · B  3.4% · D  3.8% · E 7.1% · G  0.0% · H 10.3%
    #   ⚠️ `_turn_within` 은 **자차 경로가 회전할 때만** 0 이 아니다. 자차가 직진인데
    #      옆에 좌회전 전용차로가 있는 경우는 애초에 안 걸린다 — 그건 막는 게 아니다.
    #   ⚠️ 이 제한은 **오래 갇히면 푼다** — 안 풀면 회전 60m 안의 죽은 차 하나에
    #   미완주가 확정된다(재현: 300초 돌려도 WAIT·오프셋 0.00). 다만 그 문턱은
    #   앞차가 무엇이냐에 따라 다르다(아래 TURN_GUARD_RELAX_MOVED).
    #   이미 시작한 기동은 안전상 그대로 끝낸다.
    OVERTAKE_TURN_GUARD = 60.0
    # ★★게이트 해제 문턱은 **두 단**이다(2026-09-08).
    #   한 단(20초)으로 두면 '신호를 기다리는 앞차'와 '죽은 NPC'가 구분되지 않는다.
    #   실측(고치기 전, 우회전 39m 앞 · 20m 앞 차량 45초):
    #     처음부터 정지(죽은 NPC)    20.1초 `WAIT` -> 23.2초 `PASS`
    #     달리다 멈춘 차(신호 대기)  20.1초 `WAIT` -> 23.2초 `PASS`  ← **완전히 동일**
    #   즉 이 게이트가 애초에 막으려던 동작이 20초 뒤엔 그대로 나왔다.
    #   측정된 신호 한 주기는 **36초**다(behavior.py, 2026-09-06 코스 E 신호108) —
    #   20초로는 한 주기도 못 기다린다. 달리다 멈춘 차는 한 주기 + 출발 지연을 준다.
    #   ⚠️ 긴 쪽도 **반드시 유한**하다. NPC 는 달리다가도 죽는다 — '영영 안 푼다'는
    #      그 자리에서 미완주(0점)를 확정하는 것이고, 앞지르기 감점보다 훨씬 비싸다.
    TURN_GUARD_RELAX_MOVED = 45.0
    # ★교차로 꼬리물기 금지 [도교법 제25조⑤]: "교차로에 들어가려는 경우에는 진행하려는
    #   진로의 앞쪽에 있는 차의 상황에 따라 교차로에 정지하게 되어 다른 차의 통행에
    #   방해가 될 우려가 있는 경우에는 그 교차로에 들어가서는 아니 된다."
    #   실측 2026-08-25 코스 H: 녹색에 진입했는데 5m 앞에 정지차가 있어 **교차로 한복판에
    #   153초** 서 있었다(꼬리물기 + 교차 교통 전면 차단). 진입 전에 물어봤어야 했다.
    JAM_LEN = 4.848             # 자차 길이[m] — 교차로를 완전히 빠져나가려면 이만큼 더 필요
    JAM_MARGIN = 2.0            # 그 뒤 여유[m]
    JAM_STOPPED_V = 1.0         # 이 아래면 '서 있는 차'로 본다[m/s]
    JAM_LANE_HALF = 2.0         # 경로기준 이 안이면 내 진로[m]
    JAM_LOOK_S = 4.0            # 앞차 위치를 이만큼 앞까지 내다본다[s] — 아래 참고
    #   ★★**'서 있는 차'만 보면 늦는다.** 기어가는 줄은 서 있지 않으니 통과되는데,
    #   우리가 교차로에 들어간 뒤에 멈춘다. 실측 2026-08-28 스윕 코스 E (919,208):
    #     t=357 적신호 감속 -> t=358 녹색, clr +4.49 인 줄 뒤로 16km/h 로 진입
    #     t=368 교차로 **한복판**(j=1)에서 정지 -> **212초** 갇혀 미완주(동결 중단)
    #     그때 주변 30대가 전부 v=0.0 — 교차 교통을 통째로 막은 꼬리물기다.
    #   -> 속도로 자르지 말고 **우리가 출구에 닿을 때쯤 그 차가 어디 있을지**로 본다.
    #      기어가는 줄(1.5m/s)은 4초에 6m 밖에 못 가니 그대로 걸리고,
    #      흐르는 차(8m/s)는 32m 를 가니 저절로 풀린다.
    ESCAPE_WATCH = 12.0         # 탈출 금지 판단: 앞 이 거리까지 본다
    ESCAPE_SIDE_OK = 0.3        # 옆으로 이만큼 비켜 있으면 통로가 열린 것으로 본다
    # ★★사고현장 우회 — **죽은 차** 앞에서의 마지막 탈출(2026-09-09 코스 E (915,216) 65초 교착·미완주).
    #   합법 통로(l/r)가 없고 막은 것들이 전부 오래 서 있으면, 지도의 **물리 폭**(pl/pr, 대향
    #   연결로 포함)과 물체 기준 빈 통로로 비켜 간다. 주최측 답변상 중앙선 침범(항목4)이지만
    #   미완주(0점)보다 싸다 — 사용자 결정 2026-09-09 "전방 사고 피하는 거 왜 안 만드냐".
    # ★★얼마나 서 있어야 '죽은 것'인가 — 로그 11판(스윕 6 + 재주행 5)에서 **앞차가 멈췄다가
    #   다시 출발한 경우**를 전부 재서 정했다(2026-09-09). 그런 경우는 6회뿐이었고
    #   가장 길었던 대기가 **13.7초**(코스 H), 그 다음이 8.9 · 8.6 · 6.0 · 2.8 · 2.7 초다.
    #   20초를 넘긴 적은 한 번도 없다. 그런데 이 맵 신호의 적색이 18초라, 적색 시작에 맞춰
    #   도착하면 정상 대기도 20초에 닿는다 — 여유가 너무 얇다. 그래서 **45초로 통일**한다.
    #   (사용자 지적 2026-09-09: "20초는 좀 짧지 않냐". 죽은 차 앞에서 25초를 더 기다리는
    #    비용은 한 판에 한두 번이고, 살아 있는 차를 앞지르는 건 제22조 위반이다.)
    #   ⚠️ 자차 쪽 갇힘 시계(`stuck_long`, 20초)는 그대로 둔다 — 그건 적신호 대기 중에는
    #      멈춰 있어(`red_wait_blocked`) 사실상 **녹색에서만 쌓이는** 시간이다.
    DEAD_BLOCK_S = 45.0         # 원래 서 있던 것(사고차·공사차)
    DEAD_BLOCK_MOVED_S = 45.0   # 달리다 멈춘 차 — 신호 한 주기를 더 보고도 안 가면 죽은 것
    DEAD_CW_GUARD = 25.0        # 앞 이 안에 무신호 횡단보도가 있으면 우회하지 않는다[m]
    DEAD_ESCAPE_CLEAR = 0.5     # 죽은 차 옆으로 남길 **최소** 여유[m] — behavior.body_gap_min(0.35) 보다 넓게
    DEAD_ESCAPE_CLEAR_MAX = 1.0 # 물리 폭이 허락하면 이만큼까지 벌린다[m] — 비스듬히 빠져나갈 때 모서리가 스친다
                                #   (2026-09-09 모의: 여유 0.5 로 나가다 앞우측 모서리가 자전거에 0.58m 로 접근해
                                #   SWEEP_NOW(0.6) 에 걸려 NARROW_BLOCK — 옆으로 빠지는 궤적은 목표 여유보다 좁게 스친다)
    DEAD_ESCAPE_HALF_PAD = 0.25 # 목표 통로 판정 반폭 = 차 반폭 + 이만큼(차로 반폭 1.5 대신)[m]
    DEAD_PASS_ROOM = 10.0       # 막은 것 너머 통로에 필요한 빈 길이[m](평소 PASS_ROOM 20)
    DEAD_PASS_CLEAR = 3.0       # 막은 차 중심이 이만큼 뒤로 가면 복귀[m](평소 pass_clear 6)
    DEAD_ESCAPE_V = 2.5         # 우회 중 속도 상한[m/s] — 9km/h
    STEP_METERS = 1.6           # 제어 한 주기에 나아가도 되는 최대 거리[m](수신 지연 대비)
    GOAL_MIN_RUN = 30.0         # 이만큼은 달려야 '완주' 를 인정한다[m] — goal_reached 주석
    SPEED_MARGIN = 2.0 / 3.6    # 제한속도에서 미리 빼 두는 여유[m/s] — 근거는 아래 __init__
    LIM_LEAD_M = 12.0           # 제한이 낮아지는 지점 이만큼 **앞에서 이미 그 속도**가 되게 당긴다[m] — __init__ 참조
    # ★종점 정지. 예전엔 '12m 남으면 (남은거리-2)×0.6' 이라는 선형 램프였는데,
    #   50km/h(13.9m/s)로 달려오면 12m 지점에서 갑자기 6.0m/s 를 요구한다 —
    #   그 속도의 제동거리는 21.5m 라 **못 선다**. 실측 2026-08-19 v6:
    #     종점 2.9m 전까지 50.3km/h(cap_by=PROFILE) -> 급제동 -> **8.9m 지나서 정지**
    #   `sqrt(2·a·거리)` 로 바꾸면 38m 앞에서 물려 감속도가 내내 2.5m/s^2 로 일정하다.
    # ★어린이보호구역의 **무신호 횡단보도**는 보행자 유무와 관계없이 일시정지 [법 제27조⑦].
    #   사용자 지적 2026-08-20: "골목길 횡단보도 정지선에서는 왜 안 멈춤?" — 우리 코드는
    #   신호등만 봤다. 코스 A 한 판에 해당 지점이 **24곳**인데 한 곳도 안 섰다.
    # ★45.0 이었다. 보호구역 판정을 **횡단보도 자기 도로**로 바꾸면서(위 참조) 이제
    #   50km/h 로 달리는 중에도 30 구간 입구의 횡단보도가 보이는데, 그러면 45m 로는
    #   **원리상 못 선다** — 50km/h(13.9m/s) 제동거리 38.6m + 본체 여유 8.0m = 46.6m.
    #   실측 2026-08-25 코스 E road 2488: 44.8m 에서 49.4km/h, 남은 36.8m < 필요 37.6m
    #   로 **0.8m 모자랐다**. 멀리 보는 것 자체는 공짜다(정지 프로필이 sqrt(2ad) 라
    #   먼 거리에서는 제한속도보다 높은 값을 내 아무 영향이 없다).
    CW_LOOK = 60.0              # 앞쪽 이 거리까지 횡단보도를 본다[m]
    CW_LAT = 6.0                # 진행방향 옆 이 안의 것만 내 것으로 본다[m]
    CW_STOP_V = 0.3             # 이 아래로 떨어지면 '섰다'로 인정[m/s]
    # ★**그 횡단보도 앞에서** 서야 인정한다. 이 여유 안에 들어와 있어야 한다[m].
    #   실측 2026-08-20 코스 A: 이 조건이 없어서 24곳 중 18번만 섰다 —
    #   적신호에 서 있는 동안 20m 앞 횡단보도까지 같이 '섰다'로 소진되고,
    #   붙어 있는 횡단보도(road 2574 는 19m, 2801 은 13m 간격)도 한 번에 지워졌다.
    CW_DONE_NEAR = 2.5
    # ★★**횡단보도 위에 사람이 있으면 정지선 앞에 선다** [법 제27조①] (2026-09-10 코스 H).
    #   실측 (956,653) 신호151: 적신호에 섰다가 녹색이 되자 **횡단보도 한복판에 서 있는
    #   사람 옆(2.3m)을 7.7km/h 로 기어 지나갔다**(PED_STILL_ASIDE). 사용자: "사람이
    #   횡단보도 건널 때는 정지선에서 대기하고". 맞다 — 서 있어도 횡단 중이다.
    CW_PED_LONG = 6.0           # 횡단보도 본체 앞뒤로 이만큼 안에 있으면 그 횡단보도 위[m]
    CW_PED_LAT = 3.0            # **서 있는** 사람은 내 진로 옆 이 안에 있을 때만 센다[m]
    #   ★2026-09-11 코스 H (1294,674): 다 건너가 길가에 선 사람(경로 옆 2.17m)에 12초를 섰다.
    #     사용자: "사람이 이미 횡단보도를 다 건너갔는데 여전히 있다고 착각해서 멈춤".
    #     그렇다고 아예 안 서면 **채점 항목⑩(사람 반경 3m 를 안 서고 통과)** 에 걸린다
    #     (`eval/score_fma.PED_NEAR` = 3.0). 그래서 폭은 채점과 같은 3m 로 맞추고,
    #     **오래 붙들지 않는다** — 아래 CW_PED_MAX_S 로 짧게 서고 서행 통과로 넘긴다.
    # ★★**건너오는 중인 사람은 더 멀리서 센다**(2026-09-10 코스 H (1302,491) 재실측).
    #   옆 5m 로만 보면, 인도에서 1.3m/s 로 걸어오는 사람이 5m 안에 들어왔을 때 우리는 이미
    #   정지선 1.5m 앞이라 **선을 1.8m 넘어 횡단보도 위에 섰다**(항목⑫). 사용자: "슬금슬금
    #   가다가 횡단보도에서 멈추면 어캄". 걷고 있고 **가로지르는 방향**이면 9m 까지 본다.
    CW_PED_LAT_MOVING = 9.0     # 걷는 중이면 이만큼까지[m]
    CW_PED_CROSS_DEG = (40.0, 140.0)   # 경로 기준 상대방위가 이 범위면 '가로지르는 중'
    CW_PED_MAX_S = 5.0          # 그 사람이 이만큼 **가만히** 서 있으면 다 건넌 것이다 -> 서행 통과로
                                #   (12초는 길다 — 채점은 '섰나'만 보고, 그 뒤엔 비켜 가는 게 낫다)
    CW_GAP_LINE = 4.8           # 도색 정지선이 있으면 그 앞 여유(앞범퍼 3.81 + 1.0)[m]
    CW_GAP_BODY = 8.0           # 없으면 횡단보도 중심에서 (반폭 3 + 앞범퍼 3.81 + 여유)[m]
    CW_ZONE_LIM = 30 / 3.6 + 0.1  # 이 이하 제한속도면 보호구역으로 본다
    RTOR_LOOK = 10.0            # 적신호 우회전 양보 판정 기준점: 경로 이만큼 앞[m]
    GOAL_DEC = 2.5              # 종점 정지 계획 감속[m/s^2]
    GOAL_STOP_BACK = 1.0        # 종점보다 이만큼 앞에서 0 이 되게 잡는다[m]

    # ★★종점 우측 정차 [법 제34조 · 시행령 제11조①1 "차도의 오른쪽 가장자리에 정차"]
    #   지금까지는 목표 좌표가 있는 차로(대개 **1차로**)에 그대로 섰다. 사람이라면
    #   목적지 전에 우측 지시등을 켜고 가장자리 차로로 붙여 세운다.
    #   ⚠️ 다만 **주최측이 준 목표 좌표 자체가 1차로 한복판**이다(실측: 코스 A 목표
    #      (1033.2,-474.9) 는 우리 경로 끝에서 0.41m). 끝까지 붙으면 그 좌표에서
    #      멀어진다 — 공식 코스(road 1926)는 같은 방향이 **4차로**라 최우측까지
    #      **9.3m**. 도달 판정 반경을 모르는 상태에서 9.3m 는 완주를 거는 도박이다.
    #      그래서 기본값은 **한 차로까지만**(`lane`). 사전테스트에서 도달 판정 기준을
    #      확인하면 `PULLOVER=edge` 로 열면 된다.
    #   모드: off(종전) | lane(최대 한 차로, 기본) | edge(차도 오른쪽 끝까지)
    PULLOVER_MODE = os.environ.get("PULLOVER", "lane").strip().lower()
    PULLOVER_START_M = 90.0     # 이 거리 안에서 '붙일 자리'를 계산하기 시작한다[m]
    PULLOVER_START_EDGE_M = 150.0  # edge 모드는 더 멀리서 본다(4차로 9.4m 는 8.8초 걸린다)
    PULLOVER_SIGNAL_M = 45.0    # 지시등은 이 안에서 켠다 [시행령 별표2: 30m 이상 전]
    PULLOVER_SIG_LEAD_S = 3.5   # 지시등을 켠 뒤 이만큼 지나서 붙기 시작한다[s] — 대회 기준 3초 선행(항목13)
    PULLOVER_MAX_LANES = 1      # lane 모드에서 옮길 차로 수. edge 는 갈 수 있는 데까지
    PULLOVER_EDGE_GAP = 0.5     # 차도 오른쪽 끝에서 (반폭 + 이만큼) 안쪽에 세운다[m]
    PULLOVER_RATE = 0.7         # 최소 횡이동 속도[m/s]
    PULLOVER_RATE_MAX = 1.2     # 상한[m/s]. 3.1m 를 2.6초 — 사람 차선변경(3.5m/4초)과 비슷
    PULLOVER_SLOPE_MAX = 0.30   # 간 거리 1m 당 옮길 수 있는 최대 횡거리[m] = 16.7°. 서 있으면 0 이 된다
    PULLOVER_HOLD_V = 5.6       # 붙기 구간 속도 하한[m/s] = 20km/h (그 아래로는 안 묶는다)
    # ※상한(PULLOVER_MAX_OFF 3.5m)을 두려다 되돌렸다(2026-09-11). 코스 A 종점은 경로가 두 차로
    #   왼쪽으로 합류해(r 1.5 -> 7.7) **6m 넘게 오른쪽으로 옮기는 것이 정답**이다
    #   (`tests/test_run_0906b.py::test_종점_붙기는_도로_기준으로_고정된다`). 폭으로는 못 가른다.
    PULLOVER_CLEAR_BACK = 15.0  # 옮길 자리를 뒤로 이만큼까지 확인한다[m]
    PULLOVER_CLEAR_PAD = 0.4    # 그 자리 물체와의 최소 여유[m]

    # ★지정차로 — 회전은 그 회전이 허용된 차로에서 해야 한다(도교법 제25조).
    #   실측 2026-08-15: junction 39 에서 lane -1 만 좌회전 연결로로 이어지는데
    #   우리는 lane -2(직진 전용)로 진입해 좌회전했다. 지도에서 미리 계산한 need 를 따라간다.
    LANE_RATE = 0.9             # m/s. 지정차로 이동 속도. 3.2m 한 차로를 3.6초에 옮긴다.
    #   0.6 이면 한 차로에 5.3초 = 50km/h 로 74m 라, 계획이 주는 이동 구간(65m)
    #   안에 못 끝내고 교차로에 걸친 채로 들어간다. 사람 차선변경이 3.5m/4초쯤이라
    #   0.9 는 여전히 얌전한 편이다.
    TURN_SIGNAL_AHEAD = 35.0    # 이 앞까지의 경로가 꺾이면 회전 **판단**(route_turn)
    TURN_SIGNAL_MIN = math.radians(25)
    # ★★지시등은 **회전 시작점 30m 앞**에서 켜야 한다 [법 제38조① · 시행령 별표2].
    #   그런데 위 `_upcoming_turn` 은 '앞 35m 가 25° 꺾이나'를 보므로, 급한 코너일수록
    #   창이 코너를 물어야 문턱을 넘어 **선행이 저절로 짧아진다.**
    #   실측 2026-08-30 전 코스: 회전 65곳의 선행 **중앙값 21.0m** — 30m 를 넘는 곳이
    #   하나도 없었다(최소 0.0m). 사용자 지적 "깜빡이 늦게 킴"이 이것이다.
    #   -> 판단(route_turn)과 **점등 시점을 분리한다.** 코너의 급함은 짧은 창으로 재고,
    #      점등은 그 코너 시작점 기준 LEAD 앞부터. 그래야 선행이 각도와 무관해진다.
    #   ⚠️ route_turn 은 그대로 둔다 — 그건 양보·비보호좌회전·적신호우회전 판단에
    #      쓰이므로 켜지는 시점을 앞당기면 엉뚱하게 일찍 양보한다.
    # ★★대회 채점은 **거리가 아니라 시간**이다 [안내문 2026-08-27 · 평가항목 13]:
    #   "차로 변경 시 **3초 내 미점등**: 경미(-3)". 법(제38조① 30m)과 둘 다 만족해야 한다.
    #   50km/h(13.9m/s)에서 3초 = **41.7m** 라 34m 로는 2.45초밖에 안 된다.
    #   -> 선행거리를 **속도에 따라** 잡는다: max(법 30m + 여유, 3초 × 지금 속도 + 여유).
    #   ⚠️ 상한을 둔다. 정지 상태에서 되살아난 뒤 가속하면 속도가 확 오르는데, 그때마다
    #      창이 늘면 앞쪽 엉뚱한 회전을 미리 켠다. 60m 면 50km/h 4.3초로 충분하다.
    TURN_SIG_LEAD = 34.0        # 하한(법 30m + 격자·측정오차 여유 4m)
    TURN_SIG_SEC = 3.0          # 대회 기준[s]
    TURN_SIG_LEAD_MAX = 60.0    # 상한[m]
    TURN_SIG_WIN = 22.0         # '급한 회전'을 재는 창. 이 안에서 MIN 이상 꺾이면 회전
    SIGNAL_RATE = 0.08          # m/s. 횡오프셋이 이보다 빨리 변하면 '차선 변경 중'
    LAT_PENDING_SIG = 1.5       # 차체가 명령 자리에서 이만큼 벗어나 있으면 '아직 옮기는 중'[m] — 지시등
    LAT_PENDING_DONE = 0.5      # 이 안으로 들어오면 끝났다[m]
    # ★★경로에 박힌 차선변경도 **뒤차를 보고** 들어가야 한다 [법 제19조③]:
    #   "그 변경하려는 방향으로 오고 있는 다른 차의 정상적인 통행에 장애를 줄 우려가
    #   있을 때에는 진로를 변경하여서는 아니 된다." 사용자 지적 2026-08-30 코스 E
    #   (1249,-108)~(1288,-178): sig=L 을 켜고 옆차로로 들어가면서 뒤를 안 봤다.
    #   ⚠️ 우리는 횡위치를 직접 못 민다 — 그 이동은 **경로 기하에 박혀** 있다(off=0).
    #      대신 **속도를 줄이면** 경로상 그 지점에 늦게 도착하므로 진입이 늦춰진다.
    #      그래서 개입은 '감속'뿐이다. 절대 가속하지 않는다.
    LC_REAR_WATCH = 35.0        # 목표 차로 **뒤쪽** 이 거리까지 본다[m]
    LC_REAR_TTC = 4.0           # 이 시간 안에 내 옆에 붙을 차만 문제 삼는다[s]
    LC_REAR_MIN_V = 2.8         # 양보 감속의 하한[m/s] — 서 버리면 그게 더 위험하다
    LC_REAR_MARGIN = 1.5        # 그 차보다 이만큼 느리게 가서 먼저 보낸다[m/s]

    # ★사람이 계속 서 있어 못 가는 경우: 한참 기다린 뒤에는 넉넉히 벌려 서행 통과한다.
    #   즉시 반응은 '정지'가 맞지만, 영원히 서 있으면 완주를 못 한다(사용자 결정 2026-08-15).
    PED_BYPASS_SEC = 60.0
    PED_BYPASS_CLEAR = 1.5      # 사람 옆을 지날 때 최소 여유(물건의 0.4m 보다 훨씬 크게)
    PED_BYPASS_V = 2.0          # 7km/h 서행
    PED_LATCH_CLEAR = -4.0      # 사람이 뒤로 이만큼 가야 우회 래치를 푼다
    PED_BLOCK_CLR = 0.7         # 서 있는 사람과 **차로 안에서** 지나갈 때 실여유가 이보다 작으면 '막았다'[m]
                                #   = 행동층 ped_aside_clr 0.4 + 추적 오차 0.3

    def __init__(self, scenario=None, route=None, base_limit=8.33, tl_stops_extra=None,
                 lane_plan=None, turn_lane=True, max_speed=None, crosswalks=None,
                 stoplines=None, stoplines_all=None, allow_centerline_escape=None):
        self.sc = scenario
        # ★주최측 공식 답변: 장애물 유무·점선/실선과 관계없이 중앙선을 넘으면 위반이다.
        #   기본 주행은 절대 반대 차로를 회피 공간으로 쓰지 않는다. 이 스위치는 완주 가능성만
        #   실험하는 명시적 opt-in 이며, 켜도 채점기에서는 똑같이 중앙선 침범으로 감점된다.
        self.allow_centerline_escape = (_env_enabled("ALLOW_CENTERLINE_ESCAPE")
                                        if allow_centerline_escape is None
                                        else bool(allow_centerline_escape))
        self._centerline_escape_selected_logged = False
        if self.allow_centerline_escape:
            print("[drive] ⚠ ALLOW_CENTERLINE_ESCAPE 활성화 — 평가상 중앙선 침범이며 "
                  "면책되지 않는 완주 우선 비정상/실험 전략", flush=True)
        if route is None:
            route = scenario.route_points() if scenario else straight_route()
        self.route = route
        self.base_limit = scenario.speed_limit if scenario else base_limit

        # ★경로점별 제한속도 — 차로계획의 lim(xodr 노면표시에서 읽음)을 쓴다.
        #   시나리오 JSON 의 speed_limit 은 우리가 임시로 박은 값이라, 지도가 말하는
        #   값이 있으면 그게 우선이다(실측: 공식 v1~v7 코스는 전 구간 **50km/h** 인데
        #   30 으로 달리고 있었다). 보호구역 **진입 전에** 미리 감속해야 하므로
        #   프로파일 단계에서 반영한다 — 들어가서 줄이면 이미 위반이다.
        # ★상한(--max-speed). 지도값보다 **느리게** 달리고 싶을 때 쓴다. 느린 건 위반이 아니다.
        #   왜 필요한가(2026-08-17 실측): 차로 안 정지물 옆을 제한 45 이상으로 지나가면
        #   VTD 출력이 25Hz -> 3.4Hz 로 무너지고 **끝까지 회복되지 않는다**(제어를 끊고
        #   25초 세웠다 재접속해도 3.4Hz. VTD 재시작 외엔 안 풀린다). 40 이하면 안 걸린다.
        #   그런 코스를 만나면 당일 이 플래그 하나로 40 으로 낮춘다 — 미완주보다 26초가 싸다.
        self.max_speed = max_speed
        self._lim_idx = [(p or {}).get("lim") for p in lane_plan] if lane_plan else None
        # ★★교차로 구멍(lim=None)에는 **직전 제한을 이어 붙인다** — 제한속도는
        #   교차로에서 증발하지 않는다(보호구역 표지는 해제 지점까지 유효하다).
        #   실측(합성, 2026-08-31): 30 구간 사이 20m 구멍에서 프로파일이 **39km/h**
        #   까지 올라갔다(전진 패스가 base_limit 로 가속). 현재 코스 6개에는 걸리는
        #   자리가 없지만(스캔 0곳) 대회 코스는 모른다. 잇는 쪽은 항상 합법이고,
        #   손해는 교차로 통과 몇 초뿐이다. 첫 유효값 이전 구간은 그대로 둔다.
        if self._lim_idx:
            _cur = None
            for _i, _x in enumerate(self._lim_idx):
                if _x is not None:
                    _cur = _x
                elif _cur is not None:
                    self._lim_idx[_i] = _cur
        # ★★제한이 낮아지는 지점 **LIM_LEAD_M 앞에서 이미 그 속도**가 되게 당긴다(2026-09-06).
        #   역방향 패스(control.apply_speed_limits)는 경계점에서 **정확히** 목표에 닿게
        #   짜여 있어, 추적 지연만큼 경계를 넘은 뒤에야 30 이 된다. 실측 코스 E (1226,-385):
        #   경계에서 30.9km/h, 8m 지나서 27. 사용자: "붉은 도로에 도착했을 때 30 이 되도록
        #   미리 브레이크". 앞 LIM_LEAD_M 안의 최소 제한을 끌어온다 — 낮추는 쪽만이라
        #   항상 합법이고, 비용은 50->30 경계마다 ~0.5초.
        if self._lim_idx and self.LIM_LEAD_M > 0.0:
            _cum = [0.0]
            for _a, _b in zip(route, route[1:]):
                _cum.append(_cum[-1] + math.hypot(_b[0] - _a[0], _b[1] - _a[1]))
            _src = list(self._lim_idx)
            _n = min(len(_src), len(_cum))
            _j = 0
            for _i in range(_n):
                if _src[_i] is None:
                    continue
                _j = max(_j, _i)
                while _j + 1 < _n and _cum[_j + 1] - _cum[_i] <= self.LIM_LEAD_M:
                    _j += 1
                _win = [x for x in _src[_i:_j + 1] if x is not None]
                if _win:
                    self._lim_idx[_i] = min(_win)
        if max_speed is not None and self._lim_idx:
            self._lim_idx = [None if x is None else min(x, max_speed) for x in self._lim_idx]
        # ★★**제한속도를 목표로 삼으면 넘는다.** 실측 2026-08-30 코스 E 753초:
        #     50 구간 최대 +4.68km/h · 30 구간 최대 +2.15km/h
        #     1km/h 초과 프레임 4.28% / 3.58%, 초과 구간 29개 · 합 27.2초
        #   원인은 두 겹이다. ① 프로파일이 제한값을 **그대로** 목표로 준다.
        #   ② 우리 `speed` 는 0.30초 창 이동거리라 가속 중에는 실제보다 낮게 읽혀
        #      제어기가 계속 밀어붙인다. 그래서 목표에서 미리 빼 둔다.
        #   안내문은 +1km/h 까지만 봐준다[항목 1·2] — 넘으면 구간마다 경미 -3 이고,
        #   그건 항목 1·2 두 개니까 구간당 최대 -6 이다.
        #   비용(같은 CSV 로 계산): 여유 1.0 -> +9.9초 / 1.5 -> +14.0 / 2.0 -> +18.4초.
        #   순위는 **총점이 완주시간보다 먼저**라 18초로 점수를 사는 게 맞다.
        if self.SPEED_MARGIN > 0.0:
            if self._lim_idx:
                self._lim_idx = [None if x is None else max(0.5, x - self.SPEED_MARGIN)
                                 for x in self._lim_idx]
            self.base_limit = max(0.5, self.base_limit - self.SPEED_MARGIN)
        mapped = [x for x in (self._lim_idx or []) if x is not None]
        if mapped:
            self.base_limit = max(mapped)
        if max_speed is not None:
            self.base_limit = min(self.base_limit, max_speed)
        v_prof = speed_profile_2pass(route, v_max=self.base_limit)
        if mapped:
            v_prof = apply_speed_limits(route, v_prof, lambda _x: self.base_limit,
                                        a_dec=1.2, per_index=self._lim_idx)
        elif scenario and scenario.zones:
            v_prof = apply_speed_limits(route, v_prof, self._plan_limit, a_dec=1.2)
        self.v_prof = v_prof

        self.pp = PurePursuit()
        self.lon = LongPI()
        self.beh = Behavior(speed_limit=self.base_limit)
        lane_w = getattr(scenario, "lane_width", 3.5) if scenario else 3.5
        ov_side = getattr(scenario, "overtake_side", 1.0) if scenario else 1.0
        self.overtaker = Overtaker(lane_width=lane_w, side=ov_side)
        self.rf = RouteFrame(route)

        self.cum = [0.0] * len(route)
        for i in range(1, len(route)):
            self.cum[i] = self.cum[i-1] + math.hypot(route[i][0]-route[i-1][0],
                                                     route[i][1]-route[i-1][1])
        self.total = self.cum[-1] if self.cum else 0.0
        # 신호등 정지선 {tl_id: (x,y)} — 없으면 적신호는 '모르면 정지'
        self.tl_stops = dict(tl_stops_extra or {})          # 맵 전체 DB(계산값)
        if scenario:                                          # 실측값이 있으면 덮어쓴다
            self.tl_stops.update({int(k): v for k, v in (getattr(scenario, "tl_stops", None) or {}).items()})
            # ★오프라인 판(eval/scenarios/*.json)의 `lights[].stop_line_x` 도 쓴다.
            #   여기 없으면 `_tl_stop_dist` 가 늘 None 이라 **오프라인 신호가 전부
            #   `*_BLIND` 로 돈다** — 정지선을 아는 분기(YELLOW_STOP 거리판정,
            #   WAIT_LEFT_ARROW, 비보호 좌회전의 '아직 설 수 있나')가 한 번도 안 밟힌다.
            #   실측 2026-08-20: 그것 때문에 비보호 좌회전 대향차 양보를 넣고도
            #   오프라인에서 발동조차 안 했다. 모의 도로는 y=0 직선이라 (x, 0) 으로 둔다.
            for lt in (getattr(scenario, "lights", None) or []):
                self.tl_stops.setdefault(int(lt.id), (float(lt.stop_line_x), 0.0))

        # 무신호 횡단보도 DB. 보호구역 의무정지뿐 아니라 무신호 좌회전 양보 때
        # 횡단보도 위에 서지 않을 정지점 계산에도 쓴다. 없으면 두 보정만 꺼진다.
        self.cw = [c for c in (crosswalks or []) if not c.get("signal")]
        # ★★**신호 있는 횡단보도도 따로 들고 있는다**(2026-09-10 코스 H (960,655)).
        #   `self.cw` 는 제27조⑦(보호구역 무신호 의무정지)용이라 신호 있는 곳을 뺀다. 그런데
        #   **횡단보도 위 사람 앞 정지**(제27조①)는 신호 유무와 무관하다 — 우리 녹색에 아직
        #   건너는 사람이 있으면 서야 한다. 실측: 신호151 녹색이 되자 횡단보도 한복판에 선
        #   사람 옆 2.3m 를 7.7km/h 로 지나갔다. 그 횡단보도가 `signal=True` 라 애초에 목록에
        #   없어 `CROSSWALK_PED` 가 한 번도 안 걸렸다. 이 맵은 195곳 중 **121곳이 신호**다.
        self.cw_all = list(crosswalks or [])
        self._cw_by_key = {(c["road"], c["s"]): c for c in self.cw_all}
        # ★각 횡단보도를 **경로 위로 미리 투영**해 둔다(정적이라 한 번이면 된다).
        #   왜: ego 기준 횡거리로만 거르면 **코너를 돌아 들어가는 횡단보도를 못 본다.**
        #   실측 2026-08-25 코스 E road 2479: 회전 전에는 옆 15m 라 게이트(6m) 밖이었고,
        #   6m 안에 들어왔을 땐 이미 **앞범퍼 8.1m 앞 · 22.9km/h** — 필요 제동 8.1m 에
        #   남은 거리 0.1m 라 원리상 못 섰다(결과: 횡단보도 본체 위 1.9m 안쪽 정지).
        #   경로기준으로 보면 같은 곳을 **59.2m 앞**에서 잡는다.
        #   ⚠️ 경로기준만 쓰면 안 된다 — 경로가 되돌아오는 지점은 투영이 앞쪽으로
        #      안 잡힌다(실측 코스 B road 1222: 경로기준 '감지못함', ego 기준 60.0m).
        #      그래서 **둘의 합집합**으로 보고, 거리는 경로기준이 잡았으면 그쪽을 쓴다
        #      (코너에서는 직선거리가 실제 주행거리보다 짧아 제동 계획이 어긋난다).
        self._cw_rf = []
        self._cw_all_rf = []
        for c in self.cw:
            try:
                s_c, d_c, _ = self.rf.project(c["x"], c["y"])
                self._cw_rf.append((s_c, abs(d_c)))
            except Exception:
                self._cw_rf.append((None, None))
        for c in self.cw_all:
            try:
                s_c, d_c, _ = self.rf.project(c["x"], c["y"])
                self._cw_all_rf.append((s_c, abs(d_c)))
            except Exception:
                self._cw_all_rf.append((None, None))
        # ★신호등도 횡단보도도 없는 **도색 정지선**. 사용자 지적 2026-08-20 "정지선 무시".
        #   [시행규칙 별표6 노면표시 530] 정지선 = 차가 정지하여야 할 지점.
        self.sl = list(stoplines or [])
        # ★도색 정지선 **전부**(710개). `tl_stops` 에 없는 신호 id 가 올 때만 쓴다.
        self.sl_all = list(stoplines_all or [])
        # ★경로 위의 **신호 정지선**(도색선 중 신호 DB 점과 짝이 되는 것) — 미보고 신호 선감속용
        self._tl_lines = self._signalized_lines_on_route()
        self._sl_done = set()
        self._cw_done = set()               # 이미 선 횡단보도(road,s)
        self._cw_ped_wait = {}              # 횡단보도 위 사람 때문에 선 시각 {key: t}
        self._cwp_hold = {}                 # 횡단보도 위 사람이 있는 동안의 속도 상한 래치 {key: v} — 내려가기만 한다
        self._bi_prev: Optional[int] = None
        self._stalled_since: Optional[float] = None
        self._dead_log = None                 # 사고현장 우회 로그 시각(30초에 한 번)
        self._esc_want = None                 # 사고현장 우회로 나갈 횡오프셋[m] (좌 +)
        self._esc_latch = False               # 나가기로 한 뒤 지나갈 때까지 유지
        self._stopped_since: Optional[float] = None
        self._frozen_log: Optional[float] = None    # 교착 경고를 마지막으로 찍은 시각
        self._frozen_ref = None                     # (x, y, t) — 동결 판정 기준점
        self._respawns: List[float] = []
        self._turn_latch = 0                    # 회전 지시등 래치(회전이 끝날 때까지)
        self._yaw_rate = 0.0                    # 자차 회전율[도/s] — 래치 해제 판정용
        self._fast_ids = set()                  # 차량 속도로 달린 적 있는 객체 id(_rf_is_vru)
        self.beh._fast_ids = self._fast_ids     # Behavior 와 **같은 set** — reset() 한 번에 둘 다 비운다
        self._prev_hd = None
        self._odo = 0.0                         # 누적 주행거리[m] — 완주 판정 최소조건
        self._odo_xy = None
        self._nudge = 0.0                       # 현재 적용 중인 장애물 회피 오프셋
        self._dt_ema = None                     # 수신 주기 평활값(제어 주기 방어용)
        self._base = 0.0                        # 지정차로 유지용 기본 오프셋
        self._po_off = 0.0                      # 종점 우측 정차 오프셋(오른쪽 = 음수)
        self._po_on = False                     # 우측 정차 구간 진입(지시등 유지용)
        self._po_edge = None                    # 종점 붙기 목표: 오른쪽 끝에서 이만큼[m] (도로 기준 고정)
        self._po_edge_prev = None
        self._po_lat_v = 0.0                    # 도로 기준 횡속도 EMA[m/s] (지시등 방향)
        self._po_started = False
        self._po_net_left = 0                   # 종점 붙기의 순 횡이동 방향(+왼쪽/-오른쪽/0)
        self._po_presig = False                 # 붙기 시작점 PULLOVER_SIG_LEAD_S 전 — 우측 선행 점등 구간
        self._po_dir_cand = 0                   # 지시등 방향 후보(디바운스)
        self._po_dir_hold = 0.0                 # 그 후보가 이어진 시간[s]
        self._beh_left_age = 0.0                # 행동층 좌측 지시등이 연속으로 켜져 있던 시간[s]
        self._pend_sig = 0                      # 차체가 명령 자리로 아직 옮기는 중인 방향(+1 좌 / -1 우 / 0)
        self._lat_settled = False               # 차체가 명령 자리에 한 번이라도 들어와 있었나(출발·리스폰 직후 제외)
        self._sig_dir_age = 0.0                 # 지금 방향 지시등이 켜져 있던 시간[s] — 선행 점등 게이트
        self._turn_prev = 0
        self._tl_prev = None                    # (tl_id, tl_state) 직전값 — 신호 바뀐 시각
        self._tl_change_t = None
        self._a_prev = 0.0                      # 직전 프레임 가속 명령 — 속도 추정 지연 보상
        self._po_want = None                    # 고른 목표 오프셋(한 번만 고른다)
        self._po_want_prev = None               # 직전 프레임의 목표(도망가는 속도 계산용)
        self._off_prev = 0.0                    # 직전 총 오프셋(방향지시등 판정)
        self._ped_since: Optional[float] = None  # 사람에 막힌 시각
        self._ped_latch = False                  # 사람 우회 중(지나갈 때까지 유지)
        self._lc_hold = None                     # (사람 id, 차도 기준 내 자리) — 경로 차선변경 미루는 중
        self.nudge_side = ov_side               # 추월과 같은 쪽(주행 가능이 검증된 방향)

        # ★차로 계획(vtd/build_lane_plan.py 산출물). 경로점별로
        #   lane/w(차로폭)/l,r(같은방향 좌우 여유)/xl,xr(중앙선 점선일 때 반대차선 여유)/need(지정차로).
        #   None = 모름(교차로 등) -> 기존 기본 동작.
        #   왜 필요한가: 9910 패킷에 차로 정보가 전혀 없어서 도교법(지정차로·중앙선)을
        #   지키려면 지도를 미리 읽는 수밖에 없다.
        # ★★★**차로계획을 우리가 달리는 경로에 거리로 다시 깐다**(2026-09-11 코스 H).
        #   차로계획(routes/<코스>_lane.json)은 **원래 경로(ego_route) 점마다** 만든 것인데, 우리는 그걸
        #   조밀화한 경로를 달린다(`Scenario.route_points` — 2m 넘는 구간에 점을 끼운다). 코스 H 는
        #   2021점 vs 2015점이라 **880번부터 번호가 어긋나** 경로 끝에서는 6점(≈8m) **앞**의 차로계획을
        #   읽었다 — 지정차로·경로 차선변경 지시등·교차로 판정·종점 붙기가 전부 그만큼 일렀고, 마지막
        #   6점은 차로계획이 아예 없었다. 나머지 5코스(A B D E G)는 점 개수가 같아 영향이 없다.
        #   번호로 찾는 곳이 10군데가 넘어 하나하나 고치지 않고, 여기서 **한 번** 원래 경로의 누적거리로
        #   맞춰 다시 깐다(끼운 점은 그 구간 시작점의 값). 파일은 그대로 둔다 — vtd/·eval/ 도구들은
        #   원래 경로 번호로 읽는다. 사용자: "차선 인식 똑바로 하게".
        self.lane_plan_raw_n = len(lane_plan) if lane_plan else 0
        _raw = getattr(scenario, "ego_route", None) if scenario is not None else None
        if lane_plan and _raw and len(_raw) == len(lane_plan) and len(_raw) != len(route):
            _pc = [0.0]
            for _a, _b in zip(_raw[:-1], _raw[1:]):
                _pc.append(_pc[-1] + math.hypot(_b[0] - _a[0], _b[1] - _a[1]))
            lane_plan = [lane_plan[max(0, min(len(_pc) - 1, bisect.bisect_right(_pc, c + 1e-6) - 1))]
                         for c in self.cum]
            print(f"[drive] 차로계획을 달리는 경로에 다시 깔았다: {len(_raw)}점 -> {len(lane_plan)}점 "
                  f"(원래 경로와 조밀화 경로의 점 번호가 달라서)", flush=True)
        self.lane_plan = lane_plan
        # ★★종점에서 붙일 자리는 **종점의 차로 단면**으로 정한다. 지금 서 있는 점의
        #   단면으로 정하면 틀린다 — 코스 A 는 종점 80m 전이 편도 1차로(r=1.71)라
        #   "붙을 데 없음"으로 못박히고, 정작 마지막 30m 에서 2차로(r=4.58)로 넓어진다.
        # ★★★**교차로 점은 기준으로 쓰지 않는다**(2026-09-08). 교차로 안에서는 `l`/`r` 이
        #   연결로 반폭이라 '도로 우측 끝까지 얼마'라는 뜻이 아니다. 그걸 기준으로 잡으면
        #   붙기 목표가 통째로 엉뚱해진다.
        #   실측 사전테스트2: 종점이 **교차로 안**(j=1)이라 r=1.65 였는데, 바로 앞 점은
        #   r=11.85 였다. 목표가 '도로 우측 끝에서 1.44m' 로 잡히고 지금 도로의 우측 끝은
        #   10m 밖이라 **오프셋 -10.2m** 를 요구했다. 그 10m 를 다 못 옮기고 종점에 닿아
        #   비스듬히 섰다. 사용자: "왜 대각선으로 멈춰" / "여전히 대각선".
        #   `_pullover_offset` 은 이미 교차로 점에서 계산을 건너뛴다(`plan.get("j")`) —
        #   목표를 고를 때도 같은 기준이어야 한다.
        self._r_suffix = None                   # 차로계획 `r` 의 접미사 최솟값(처음 쓸 때 만든다)
        self._po_goal_plan = None
        for _q in reversed(lane_plan or []):
            if _q and not _q.get("j"):
                self._po_goal_plan = _q
                break
        if self._po_goal_plan is None:                  # 전부 교차로면 예전대로
            for _q in reversed(lane_plan or []):
                if _q:
                    self._po_goal_plan = _q
                    break
        self.default_side = ov_side

        # ★지정차로 추종 = 기본 켜짐. 회전은 그 회전이 허용된 차로에서 한다(도교법 제25조).
        #   실측 2026-08-15 junction 39 좌회전 — **의도대로 동작한다**:
        #      lane -2 -> lane -1(좌회전 전용) 진입 -> 좌회전 연결로(road 1955) -> 좌회전 완료.
        #   ⚠️ 대가: 그 구간에서 리스폰이 1건 난다(공식 경로가 lane 2 로 정의돼 있어
        #      lane 1 이 '경로 이탈'로 잡힌다). 다만 리스폰이 우리를 2차선으로 되돌리지는
        #      않는다 — 경로에 스냅할 뿐이고 그 시점 경로는 이미 좌회전이라 그대로 완주한다.
        #      (한때 '되돌려서 결국 직진차로로 좌회전한다'고 적었는데 로그 확인 결과 틀렸다)
        #   ⚠️ 부작용: 연속 주행 시 Traffic(ghostdriver) 이 경로 이탈을 못 견디고 죽는다
        #      (v1·v2·v3 연속 후 크래시, 끄면 12판 무사). 대회는 한 판이라 무관하지만
        #      우리가 연속 검증할 땐 몇 판마다 VTD 를 재시작해야 한다.
        #   끄려면 --no-turn-lane.
        self.turn_lane = turn_lane

    # ---- 지도 사전지식 ----
    def _plan_limit(self, x):
        """계획용 제한속도: 구간 12m 전부터, 제한의 85%로 여유(추적지연 대비)."""
        v = self.base_limit
        for z in (self.sc.zones if self.sc else []):
            if z.x_start - 12.0 <= x <= z.x_end:
                v = min(v, z.limit * self.ZONE_MARGIN)
        return v

    def _tl_stop_dist(self, s):
        """현재 보고된 신호의 정지선까지 전방거리[m] (모르면 None).

        반환: (거리 또는 None, 이미 지났나)
        ★'모르는 신호' 와 '아는데 이미 지난 신호' 를 구별해 돌려준다. 둘 다 None 이면
          behavior 가 후자에서도 '정지선 미상 -> 정지' 로 가서 **교차로 한복판에 선다**.
        """
        p = self.tl_stops.get(s.tl_id)
        if p is None:
            return self._tl_stop_painted(s)
        dx, dy = p[0] - s.x, p[1] - s.y
        fwd = dx * math.cos(-s.heading) - dy * math.sin(-s.heading)
        if fwd > 0.0:
            # ★★DB 점 앞 3~36m 에 내 방향 도색 정지선이 **또** 있으면 그게 정지선이다(2026-09-06).
            #   신호 137: VTD 신호 기둥이 교차로 건너편에 있어 DB 점(기둥 아래 도색선)이 진입부
            #   정지선보다 **28m 뒤**다. 우리는 28km/h 로 진입부 선을 지났다 — 사용자: "정지선
            #   무시함". 지도 조사(routes/stoplines_all 710개): 105·137·138·169·170 이 같은 꼴
            #   (28~34m). 앞선 선에 서면 심판이 어느 선을 쓰든 안전하다 — 적색에 DB 점을 안 넘는다.
            early = self._painted_fwd(s, 0.0, fwd - self.TLP_EARLY_MIN)
            if early is not None and fwd - early <= self.TLP_EARLY_MAX:
                return early, False
            return fwd, False
        return None, True

    def _painted_fwd(self, s, lo, hi):
        """내 방향 도색 정지선 중 전방거리 lo<f<hi · 횡 TLP_LAT 안에서 가장 가까운 f. 없으면 None."""
        if not self.sl_all or hi <= lo:
            return None
        c_, s_ = math.cos(-s.heading), math.sin(-s.heading)
        best = None
        for q in self.sl_all:
            dx, dy = q[0] - s.x, q[1] - s.y
            if abs(dx * s_ + dy * c_) > self.TLP_LAT:
                continue
            if math.cos(q[2] - s.heading) < 0.5:
                continue
            f = dx * c_ - dy * s_
            if lo < f < hi and (best is None or f < best):
                best = f
        return best

    TLP_LOOK = 60.0      # 모르는 신호일 때 앞으로 이만큼까지 도색 정지선을 찾는다[m]
    TLP_BACK = 30.0      # 뒤로 이만큼 안에 있으면 '이미 지났다'로 본다[m]
    TLP_LAT = 4.0        # 횡으로 이 안(차로별 도색이라 좁게)
    TL_ANT_LOOK = 60.0   # 이 안의 **미보고** 신호 정지선에 대해 미리 속도를 잡는다[m] — _tl_unseen_dist
    TL_ANT_PAIR = 12.0   # 도색선이 신호 DB 점과 이 안에 있으면 '신호 정지선'으로 본다[m]
    TL_ANT_SAME = 10.0   # 보고된 신호의 정지선이 그 선보다 이만큼 이상 멀지 않으면 '그 선을 안다'[m]
    TLP_EARLY_MIN = 3.0  # DB 정지점 바로 앞(같은 선)은 '앞선 선'으로 안 본다[m]
    TLP_EARLY_MAX = 36.0 # DB 정지점 앞 이 안에 내 방향 도색선이 또 있으면 **그게** 정지선이다[m] — _tl_stop_dist

    def _signalized_lines_on_route(self):
        """경로를 따라 정렬한 [(s, x, y)] — 내 방향 도색 정지선 중 신호 DB 점(`tl_stops`)과 짝이 되는 것.

        ★★**VTD 는 신호를 늦게 알려 줄 수 있다**(2026-09-09 6코스 스윕 실측). 대부분 60~300m 앞에서
          보고되지만 신호 130·167 은 앞범퍼 **9m**, 100 은 12m, 213 은 14m 앞에서야 tl_id 가 온다
          (같은 신호는 판마다 같은 거리 — 도로 구조다). 코스 B 는 167 을 37km/h 로 9m 앞에서 적색으로
          처음 보고 1.9m 지나쳤다(항목7 -6). 지도에는 정지선이 다 있으니, 보고 전에 미리 줄인다.
        도색선(`sl_all`, 방위 있음)을 쓰는 이유: 신호 DB 점은 정지선 **중앙**이라 차로에서 4m 넘게
        비껴 있을 수 있고(신호100: 4.3m) 방위가 없어 교차로의 다른 갈래 선과 구별이 안 된다.
        """
        out = []
        rf = getattr(self, "rf", None)
        if not self.sl_all or not self.tl_stops or rf is None or len(rf.pts) < 2:
            return out
        pts = list(self.tl_stops.values())
        for q in self.sl_all:
            if not any(math.hypot(q[0] - p[0], q[1] - p[1]) <= self.TL_ANT_PAIR for p in pts):
                continue                                           # 신호 없는 도색선(양보선 등)
            s_q, d_q, i_q = rf.project(q[0], q[1])
            if abs(d_q) > self.TLP_LAT or s_q <= 0.0 or s_q >= rf.total:
                continue
            if math.cos(q[2] - rf.heading_at(i_q)) < 0.5:
                continue                                           # 나를 세우는 선이 아니다(다른 갈래)
            out.append((s_q, q[0], q[1]))
        out.sort()
        return out

    def _tl_unseen_dist(self, s_ego, tl_stop_dist):
        """앞 TL_ANT_LOOK 안의 **아직 보고되지 않은** 신호 정지선까지 경로거리[m]. 없으면 None.
        보고된 신호(`tl_stop_dist`)의 정지선이 그 선(또는 그 앞)이면 아는 것이다 -> None.
        """
        for (s_q, _x, _y) in self._tl_lines:
            d = s_q - s_ego
            if d < 0.0:
                continue                                           # 지난 선
            if d > self.TL_ANT_LOOK:
                return None
            if tl_stop_dist is not None and tl_stop_dist <= d + self.TL_ANT_SAME:
                return None                                        # 그 신호를 이미 본다
            return d
        return None

    @staticmethod
    def _reported_tl_dist(s, tl_stop_dist):
        """`_tl_unseen_dist` 에 넘길 '보고된 신호의 정지선 거리'. 신호가 없으면 None.

        tl_id 가 0/음수면 VTD 가 아무 신호도 안 주는 것이다 — 그때의 `tl_stop_dist` 는
        `_tl_stop_painted` 가 메운 **가장 가까운 도색선**이지 '보고된 신호'가 아니다.
        """
        if s.tl_id is None or s.tl_id <= 0:
            return None
        return tl_stop_dist

    def _tl_stop_painted(self, s):
        """`tl_stops` 에 없는 신호 id — **도색 정지선**으로 메운다.

        ⚠️ VTD 가 보고하는 tl_id 가 xodr 의 controller id 와 늘 맞지는 않는다.
           실측 2026-09-04: 로그 48개에 나온 신호 60종 중 **80·82 두 개**가 맵에
           controller 로 없다(tl80 은 2895 프레임). 그때 예전 코드는 (None, False) 를
           줘서 적신호가 `RED_STOP_BLIND` 로 갔다 — 정지선을 모르니 **그 자리에 선다**.
           대회에서 그건 길 한복판 정지다. 눈앞의 도색선을 쓰면 제자리에 설 수 있다.

        못 찾으면 예전 그대로 (None, False) = '모르는 신호'.
        """
        if not self.sl_all:
            return None, False
        c_, s_ = math.cos(-s.heading), math.sin(-s.heading)
        ahead, behind = None, None
        for q in self.sl_all:
            dx, dy = q[0] - s.x, q[1] - s.y
            fwd = dx * c_ - dy * s_
            if abs(dx * s_ + dy * c_) > self.TLP_LAT:
                continue
            if math.cos(q[2] - s.heading) < 0.5:      # 나를 세우는 선이 아니다
                continue
            if 0.0 < fwd < self.TLP_LOOK:
                if ahead is None or fwd < ahead:
                    ahead = fwd
            elif -self.TLP_BACK < fwd <= 0.0:
                behind = True
        if ahead is not None:
            return ahead, False
        return (None, True) if behind else (None, False)

    def _crosswalk_ahead(self, s, lim, s_ego=None, zone_only=True, all_cw=False):
        """앞쪽 **의무 일시정지** 횡단보도까지 거리와 식별자. 없으면 (None, None).

        기본값의 의무 = 무신호 + 어린이보호구역 [법 제27조⑦].
        `zone_only=False`이면 보호구역 밖도 포함한다. 이 모드는 별도 정지를 만들지 않고,
        이미 필요한 무신호 좌회전 양보의 정지점을 횡단보도 앞으로 당길 때만 쓴다.

        ★★보호구역 판정은 **그 횡단보도가 있는 도로의 제한속도**(`c["lim"]`)로 한다.
          예전엔 **자차 현재 위치**의 제한속도로 걸렀다. 그러면 50 구간에서 30 구간으로
          **들어가는 입구의 횡단보도는 경계를 넘기 전까지 존재 자체가 안 보인다.**
          실측 2026-08-25 코스 E road 2488: 49km/h 로 접근하는 내내 무시되다가 자차
          제한이 30 으로 바뀐 순간(t=383.2) CROSSWALK_STOP 이 켜졌는데, 그때 횡단보도는
          이미 **앞범퍼 1.4m 뒤**였다. 설 방법이 원리상 없었다.
          `c["lim"]` 은 도로망 전파(`road_speed_limits_net`) 값이라 코스와 무관하게 같다.
        ⚠️ DB 에 `lim` 이 없는 옛 파일이면 예전처럼 자차 기준으로 떨어진다(호환).
        """
        src = self.cw_all if all_cw else self.cw
        src_rf = self._cw_all_rf if all_cw else self._cw_rf
        if not src:
            return None, None
        c_, s_ = math.cos(-s.heading), math.sin(-s.heading)

        def fl(px, py):
            dx, dy = px - s.x, py - s.y
            return dx * c_ - dy * s_, abs(dx * s_ + dy * c_)

        best = None
        for ci, c in enumerate(src):
            # ★보호구역 여부는 **`zone` 플래그**로 본다(2026-09-04).
            #   예전엔 `lim <= 30` 으로 갈랐는데, 주최측 답변에 따라 제한속도를
            #   '붉은 노면 위에서만 30' 으로 좁히면서 그 대리 판정이 깨졌다 —
            #   구역 안쪽인데 lim 이 50 인 횡단보도가 생긴다. 둘은 다른 것이다:
            #     lim  = 채점되는 속도 구간(붉은 노면)
            #     zone = 제27조⑦ 이 걸리는 어린이보호구역(구역 전체)
            #   `zone` 이 없는 옛 DB 는 예전처럼 lim 으로 떨어진다.
            if zone_only:
                if "zone" in c:
                    if not c["zone"]:
                        continue
                else:
                    clim = c.get("lim")
                    if clim is None:
                        clim = lim               # 옛 DB 호환 — 자차 기준으로 떨어진다
                    if clim is None or clim > self.CW_ZONE_LIM:
                        continue
            fwd, lat = fl(c["x"], c["y"])
            ego_ok = (0.0 < fwd < self.CW_LOOK) and lat <= self.CW_LAT
            # 경로기준(미리 투영해 둔 값 + 이번 프레임의 s_ego)
            rf_ok = False
            if s_ego is not None and ci < len(src_rf):
                s_c, d_c = src_rf[ci]
                if s_c is not None:
                    rfwd = s_c - s_ego
                    if 0.0 < rfwd < self.CW_LOOK and d_c <= self.CW_LAT:
                        rf_ok = True
                        fwd = rfwd               # 실제 주행거리 -> 제동 계획이 맞는다
            if not (ego_ok or rf_ok):
                continue
            key = (c["road"], c["s"])
            # 의무 일시정지를 이미 마쳤어도 YIELD_CROSS 위치 보정에는 계속 필요하다.
            # 여기서 지우면 정지 다음 프레임에 교차로 목표로 다시 출발해 횡단보도
            # 위에서 대향차를 기다릴 수 있다.
            if zone_only and key in self._cw_done:
                continue
            # ★정지선이 있으면 그 앞에 선다 [법 제27조⑦]. 후보 중 **내 앞이면서 횡단보도보다
            #   가까운 것**만 쓴다 — 그냥 제일 가까운 걸 집으면 **건너편 정지선**을 잡는다
            #   (실측 2026-08-20 코스 A: 24곳 중 10곳이 그랬다).
            # ★후보의 3번째 값은 그 정지선이 **세우는 차량의 진행방향**이다. 이게 내 진행
            #   방향과 같은 것만 내 정지선이다 — 건너편 선(180°)과 교차로 선(90°)이 한 번에
            #   걸러진다. 거리 비교로 어림짐작할 필요가 없다.
            tgt, gap = fwd, self.CW_GAP_BODY
            cand = []
            for q in c.get("stops", ()):
                qf, ql = fl(q[0], q[1])
                if not (0.0 < qf <= fwd + 0.5 and ql <= self.CW_LAT):
                    continue
                if len(q) > 2 and math.cos(q[2] - s.heading) < 0.5:
                    continue                     # 나를 세우는 선이 아니다
                cand.append(qf)
            if cand:
                tgt, gap = max(cand), self.CW_GAP_LINE   # 횡단보도에 가장 가까운 앞쪽 정지선
            if best is None or tgt < best[1]:
                best = (key, tgt, gap)
        return (best[0], (best[1], best[2])) if best else (None, None)

    # ★지금 선 자리로 **뒤따르는 횡단보도까지 함께 처리**되는 경우.
    #   사용자 지적 2026-08-20: "횡단보도 2개 있고 정지선 한 개면 정지선에서 한 번 멈추삼".
    #   실측 코스 A 에 30m 안 연속쌍이 5곳 있고, 그중 뒤엣것은 **자기 정지선이 없다**:
    #     road 2574 s103(19.7m 뒤, 내 정지선 0) · road 3464 s4(19.3m 뒤, 0)
    #     road 2793 s4(26.6m 뒤) · road 2422 s4(25.5m 뒤) · road 2801 s60(14.0m 뒤)
    #   특히 road 2311 -> 3464 쌍은 **사이가 교차로**다(경로점 11/15가 junction).
    #   거기서 또 서면 **교차로 안 정차**가 된다 — 제32조 위반이고 위험하다.
    #
    #   법 제27조⑦은 "횡단보도 앞(**정지선이 설치된 경우에는 그 정지선**)에서 일시정지"다.
    #   정지선이 정해진 정지 위치이므로, 두 횡단보도가 **같은 정지선 하나를 공유**하면
    #   거기서 한 번 서는 것으로 둘 다 만족한다. 자기 정지선을 따로 가진 횡단보도는
    #   건드리지 않는다 — 그건 별도의 정지 지점이다.
    CW_PAIR = 30.0              # 이 안에 뒤따르고
    CW_PAIR_LAT = 8.0           # 진행방향 옆 이 안이며
    # ★교차로 건너 횡단보도는 더 멀어도 함께 소진한다. 사용자 지적 2026-08-20 2차:
    #   고아 정지선(1097,350)에서 섰는데 **24m 뒤 road 2826 s4 에서 또 섰다** —
    #   그 횡단보도는 교차로 안이라 거기서 서면 **교차로 내 정차**(제32조)다.
    #   차 기준 거리는 30.3m 로 CW_PAIR 를 0.3m 차이로 빗나갔었다. 거리로 가르는 게
    #   애초에 잘못이다. **사이에 교차로가 있으면** 정지 지점은 교차로 진입 전 한 곳뿐이다.
    CW_PAIR_JUNC = 45.0

    def _cw_shares_line(self, s, bi=None):
        """방금 선 자리로 **함께 처리되는** 횡단보도 키들.

        조건: 앞쪽에 있고 · **자기(진행방향) 정지선이 따로 없고** ·
              (가까이 뒤따르거나 | 사이가 교차로라 거기서는 설 수 없거나)
        """
        out = set()
        if not self.cw:
            return out
        c_, s_ = math.cos(-s.heading), math.sin(-s.heading)

        def fl(px, py):
            dx, dy = px - s.x, py - s.y
            return dx * c_ - dy * s_, abs(dx * s_ + dy * c_)

        def junc_between(px, py):
            """내 앞 경로에서 그 지점까지 사이에 교차로가 있나."""
            if bi is None or not self.lane_plan:
                return False
            s0 = self.cum[bi] if bi < len(self.cum) else 0.0
            for k in range(bi, min(len(self.lane_plan), len(self.route))):
                if self.cum[k] - s0 > self.CW_PAIR_JUNC:
                    break
                # ⚠️ **교차로 판정을 먼저 한다.** 반대로 두면, 교차로 **안에 있는**
                #    횡단보도는 그 지점이 교차로점이기도 해서 '도착'으로 먼저 빠져나가
                #    영영 True 가 안 나온다. 실측 2026-08-24 `_TR` 판 road 2826(교차로안):
                #    30.3m 앞 정지선에서 섰는데 CW_PAIR(30.0m)를 **0.3m 차이로** 못 넘어
                #    소진되지 않았고, 결국 **28km/h 로 그냥 통과**했다. 같은 0.3m 경계에
                #    두 번 물린 셈이라, 이제 거리가 아니라 교차로 여부가 먼저다.
                if self.lane_plan[k] and self.lane_plan[k].get("j"):
                    return True
                if math.hypot(self.route[k][0] - px, self.route[k][1] - py) < 4.0:
                    return False                 # 교차로 없이 먼저 도착 -> 별개 정지점
            return False

        for c in self.cw:
            fwd, lat = fl(c["x"], c["y"])
            if not (0.0 < fwd <= self.CW_PAIR_JUNC) or lat > self.CW_PAIR_LAT:
                continue
            if fwd > self.CW_PAIR and not junc_between(c["x"], c["y"]):
                continue
            mine = False
            for q in c.get("stops", ()):
                qf, ql = fl(q[0], q[1])
                if not (0.0 < qf <= fwd + 0.5 and ql <= self.CW_LAT):
                    continue
                if len(q) > 2 and math.cos(q[2] - s.heading) < 0.5:
                    continue
                mine = True
                break
            if not mine:                     # 내 정지선이 따로 없다 -> 방금 선 자리가 그 자리
                out.add((c["road"], c["s"]))
        return out

    # ★고아 정지선 — 신호등도 횡단보도도 안 붙은 도색 정지선.
    #   실측 2026-08-20 코스 A: 경로 위 6곳(가장 가까운 신호가 30.7~201.7m, 횡단보도도
    #   15m 밖). 공식 v1/v7 코스에는 0곳이라 회귀에 영향이 없고, WP9 에 5곳이다.
    #   [시행규칙 별표6 노면표시 530] 정지선 = 차가 **정지하여야 할 지점**.
    #   ⚠️ 표지판 유무는 xodr 로 확인하지 못했다. 도색만 보고 서는 것이라 과잉일 수 있으나,
    #      서는 쪽은 위반이 아니고 안 서는 쪽은 틀리면 위반이다.
    SL_LOOK = 45.0
    SL_LAT = 4.0                # 도색은 차로별이라 횡으로 좁게 본다
    SL_GAP = 5.2                # 앞범퍼 3.81 + 여유 1.0  # (+0.4, 2026-09-06 behavior.stop_margin 과 같은 이유)
    SL_DONE_NEAR = 2.5
    SL_STOP_V = 0.3
    SL_SAME = 6.0               # 이 안의 정지선은 같은 자리(차로별 도색)로 본다

    def _stopline_ahead(self, s):
        """앞쪽 고아 정지선까지 (키, 남은거리). 없으면 (None, None)."""
        if not self.sl:
            return None, None
        c_, s_ = math.cos(-s.heading), math.sin(-s.heading)
        best = None
        for q in self.sl:
            dx, dy = q[0] - s.x, q[1] - s.y
            fwd = dx * c_ - dy * s_
            lat = abs(dx * s_ + dy * c_)
            if not (0.0 < fwd < self.SL_LOOK) or lat > self.SL_LAT:
                continue
            if math.cos(q[2] - s.heading) < 0.5:      # 나를 세우는 선이 아니다
                continue
            key = (round(q[0], 1), round(q[1], 1))
            if key in self._sl_done:
                continue
            if best is None or fwd < best[1]:
                best = (key, fwd)
        return best if best else (None, None)

    def reset(self, now=None):
        """리스폰(좌표 순간이동) 시 제어기 상태 초기화."""
        if now is not None:
            self._respawns.append(now)
        self.lon.reset()
        self.overtaker.reset()
        # ★★`_bi_prev` 는 지우지 않는다(2026-09-06). 리스폰은 몇 m 순간이동이라 직전 인덱스
        #   창 안에서 다시 잡힌다. None 으로 지우면 **전역 최근접**으로 떨어지는데, 코스 G 는
        #   같은 길을 두 번 지나(idx 104 ↔ 1060 겹침) 첫 번째 지나감으로 잡혀 그쪽 속도
        #   프로파일(26km/h)로 40m 를 기었다 — 사용자: "30 도로 아닌데 왜 30 으로 가니???".
        #   창 밖(15m)이면 nearest_index 가 스스로 전역 재탐색한다.
        self._stalled_since = None
        self._stopped_since = None
        self._frozen_ref = None
        self._nudge = 0.0
        self._turn_latch = 0
        self._yaw_rate = 0.0
        self._fast_ids.clear()
        self._prev_hd = None
        self._base = 0.0
        self._po_off = 0.0
        self._po_on = False
        self._po_edge = None
        self._po_edge_prev = None
        self._po_lat_v = 0.0
        self._po_started = False
        self._po_net_left = 0
        self._po_presig = False
        self._po_dir_cand = 0
        self._po_dir_hold = 0.0
        self._beh_left_age = 0.0
        self._pend_sig = 0
        self._lat_settled = False
        self._sig_dir_age = 0.0
        self._turn_prev = 0
        self._a_prev = 0.0
        self._po_want = None
        self._po_want_prev = None
        self._ped_since = None
        self._ped_latch = False
        self._lc_hold = None
        self._cw_done.clear()
        self._cw_ped_wait.clear()
        self._cwp_hold.clear()
        self._sl_done.clear()

    def _escape_blocked(self, rf_objs):
        """무한정지 탈출(STALL_ESCAPE)을 하면 안 되는 상황 = 내 통로에 진짜 물체가 있다.

        ⚠️ 예전엔 `reason != "YIELD_PED"` 로 막았는데, **어느 구속이 라벨을 가져갔느냐**에
           의존하는 취약한 조건이었다. 실측 2026-08-15 EV_PED: 보행자 앞 2.02m 에 제대로
           섰지만 그 지점 신호가 적색이라 라벨이 RED_STOP 이었고, 탈출이 발동해 145초에 걸쳐
           2.03m 를 기어가 사람에 닿았다(-0.39m). 탈출은 '이유 없이 멈춰있을 때'만 쓰는 것이다.
        """
        for ds, d, _spd, olen, owid, _ohgt, _oid, *_ in rf_objs:
            if not (-1.0 < ds < self.ESCAPE_WATCH):
                continue
            if abs(d) - self.HALF_WIDTH - owid / 2.0 > self.ESCAPE_SIDE_OK:
                continue                       # 옆으로 충분히 비켜 있다
            return True
        return False

    def _rtor_point(self, st, s_ego):
        """적신호 우회전 양보 판정의 기준점 = **회전을 마친 뒤 우리가 있을 자리**(자차 기준).

        자차 위치로 판정하면 뒤차와 왼쪽으로 지나가는 차까지 충돌 상대가 된다
        (실측 2026-08-25 코스 H: 왼쪽 3.1m 의 1.4m/s 짜리 차 하나에 18초를 섰다).
        경로는 회전 뒤 어디로 가는지 알고 있으니 그 점을 쓴다.
        """
        if s_ego is None:
            return None
        p = self.rf.point_at(s_ego + self.RTOR_LOOK)
        if p is None:
            return None
        dx, dy = p[0] - st.x, p[1] - st.y
        c, sn = math.cos(-st.heading), math.sin(-st.heading)
        return (dx * c - dy * sn, dx * sn + dy * c)

    # ★회전 중 접촉 가드(behavior._turn_cut_conflict)가 쓸 **앞으로 갈 자리**.
    #   원호 외삽은 이미 돌기 시작해야 켜지는데, (1456,939) 무신호 좌회전에서는
    #   그때가 접촉 0.8초 전이었다. 경로는 4초 앞을 이미 알고 있으니 그걸 준다.
    TURN_CUT_T = 4.0            # 이만큼 앞까지[s]
    TURN_CUT_DT = 0.25          # 격자[s]
    TURN_CUT_VMIN = 4.0         # 느려도 이 속도로 간다고 보고 앞을 본다[m/s]

    def _path_ahead(self, st, s_ego, v):
        """[(t, x, y, heading)] — 자차 기준으로 본 앞으로 갈 경로. 모르면 None."""
        if s_ego is None or self.rf is None:
            return None
        vv = max(v, self.TURN_CUT_VMIN)
        c, sn = math.cos(-st.heading), math.sin(-st.heading)
        out = []
        prev = None
        t = 0.0
        while t < self.TURN_CUT_T:
            t += self.TURN_CUT_DT
            p = self.rf.point_at(s_ego + vv * t)
            if p is None:
                break
            dx, dy = p[0] - st.x, p[1] - st.y
            x, y = dx * c - dy * sn, dx * sn + dy * c
            h = 0.0 if prev is None else math.atan2(y - prev[1], x - prev[0])
            out.append((t, x, y, h))
            prev = (x, y)
        return out or None

    # ★갇힘 탈출 — 막은 차 **너머**까지 나갈 폭을 잰다(2026-09-06 코스 A 미완주).
    ESCAPE_LOOK = 30.0          # 이 앞까지의 정지물만 본다[m]
    ESCAPE_CLEAR = 0.35         # 막은 차 옆으로 남길 여유[m]
    ESCAPE_MAX_W = 7.0          # 이보다 더는 안 나간다[m]

    def _escape_width(self, plan, rf_objs, d_ego, side, lane_w):
        """오래 갇혔을 때 **막은 차 너머**까지 나갈 비킬 거리[m]. 못 나가면 None.

        왜 필요한가(실측 2026-09-06 코스 A t=467~656 — **188초 이동 0.00m, 미완주**):
          13m 짜리 버스가 역주행으로 마주 와 우리 앞 -0.19m 에 섰다. 그 버스는 우리
          차로와 왼쪽 차로를 **같이** 물고 있었다(경로기준 +0.25~+2.75m).
            같은 방향 좌측 여유 4.35m · 우측 1.5m · 반대편까지 7.15m · 차로폭 3.0m
          한 차로치만 나가는 추월은 목표차선이 곧 그 버스라 영영 `OVT:WAIT`,
          오른쪽은 1.5m 라 차폭 1.886m 가 안 들어간다. 끝까지 오른쪽으로 붙어도
          **0.14m 가 겹친다**(계산 확인). 즉 오른쪽으로는 원리상 못 빠진다.
          빠지려면 버스 너머 **+4.0m** 까지 나가야 했다.

        기본 모드는 같은 방향 차도의 `l/r` 경계 안에서만 폭을 넓힌다.
        `ALLOW_CENTERLINE_ESCAPE=1` 인 비정상/실험 모드만 `xl/xr` 까지 센다. 이 경우도
        평가상 중앙선 침범이며 면책되지 않는다.
        ⚠️ 이 함수는 '거기로 가라'가 아니라 '거기를 봐라'다. 부르는 쪽이 `side_clear`
           로 실제로 비어 있는지 확인한 뒤에만 쓴다.
        """
        if not plan:
            return None
        far = None
        for row in rf_objs:
            ds, d, spd, _olen, owid = row[0], row[1], row[2], row[3], row[4]
            if not (0.0 < ds < self.ESCAPE_LOOK) or spd > 1.0:
                continue                       # 앞의 **서 있는** 것만
            d_path = d + d_ego
            if abs(d_path) - owid / 2.0 > self.HALF_WIDTH:
                continue                       # 내 진로를 막는 게 아니다
            edge = d_path + side * owid / 2.0  # 그 물체가 탈출 방향으로 뻗은 끝
            if far is None or side * edge > side * far:
                far = edge
        if far is None:
            return None
        want = side * far + self.HALF_WIDTH + self.ESCAPE_CLEAR
        if want <= lane_w or want > self.ESCAPE_MAX_W:
            return None                        # 한 차로치로 충분하거나, 너무 멀다
        # 기본은 같은 방향 차도 끝까지만. 실험 옵션을 명시한 경우에만 반대 차로까지 센다.
        same_room = (plan.get("l") if side > 0 else plan.get("r")) or 0.0
        room = same_room
        if self.allow_centerline_escape:
            cross_room = (plan.get("xl") if side > 0 else plan.get("xr")) or 0.0
            room = max(room, cross_room)
        if want + self.HALF_WIDTH > room:
            return None
        return want

    # ★★죽은 차 앞 마지막 탈출(사고현장 우회) — 상수 DEAD_* 주석.
    def _dead_blockers(self, rf_objs, d_ego, now, latched=False):
        """내 진로를 막고 선 것들이 **전부 죽은 것**인가 -> (dead, 막은 줄 목록).

        `latched=True` 면 **이미 나가기로 한 뒤**다 — '얼마나 서 있었나'를 다시 묻지 않고
        막은 것이 아직 앞에 있는지만 본다(그게 래치를 푸는 조건이다).

        막은 것 = 앞 ESCAPE_LOOK 안 · 경로 기준 몸통이 내 폭 + DEAD_ESCAPE_CLEAR 안.
        죽은 것 = JAM_STOPPED_V 아래로 DEAD_BLOCK_S 이상 서 있음(달리다 멈춘 차는
        DEAD_BLOCK_MOVED_S — 신호를 기다리는 차일 수 있다). 하나라도 살아 있으면 기다린다.
        ⚠️ 서 있은 시각은 추월기의 `still_since`(plan() 이 갱신, 한 프레임 늦음 — 무해).
        """
        rows = []
        for row in rf_objs:
            ds, d, owid = row[0], row[1], row[4]
            if not (0.0 < ds < self.ESCAPE_LOOK):
                continue
            if abs(d + d_ego) - owid / 2.0 > self.HALF_WIDTH + self.DEAD_ESCAPE_CLEAR:
                continue
            rows.append(row)
        if not rows:
            return False, rows
        if latched:
            return True, rows
        for row in rows:
            spd, oid = row[2], row[6]
            if spd >= self.JAM_STOPPED_V:
                return False, rows
            t0 = self.overtaker.still_since.get(oid)
            if t0 is None:
                return False, rows
            need = (self.DEAD_BLOCK_MOVED_S if oid in self.overtaker.was_moving
                    else self.DEAD_BLOCK_S)
            if now - t0 < need:
                return False, rows
        return True, rows

    def _cw_blocks_escape(self, s, s_ego):
        """앞 `DEAD_CW_GUARD` 안에 **무신호 횡단보도**가 있나 -> 사고현장 우회를 하지 않는다.

        그 앞에 선 차는 보행자를 보내는 중일 수 있다. 사람은 그 차에 가려 우리 센서에
        안 잡힐 수 있고, **법령에 따라 정지한 차를 앞지르는 것은 제22조가 금지**한다.
        여기서만은 죽은 것처럼 보여도 기다리는 게 맞다(사용자 지적 2026-09-09 의 연장).
        """
        cw = self._crosswalk_ahead(s, self.beh.speed_limit, s_ego, zone_only=False)[1]
        return cw is not None and cw[0] < self.DEAD_CW_GUARD

    @staticmethod
    def _phys_room(plan, side):
        """그쪽으로 갈 수 있는 폭[m] = max(합법 l/r, 물리 pl/pr). 계획이 없으면 0."""
        if not plan:
            return 0.0
        legal = (plan.get("l") if side > 0 else plan.get("r")) or 0.0
        phys = (plan.get("pl") if side > 0 else plan.get("pr")) or 0.0
        return max(legal, phys)

    def _dead_escape_width(self, plan, rows, d_ego, side):
        """죽은 것들 **너머**로 나갈 횡오프셋[m]. 물리 폭이 모자라면 None.

        `_escape_width` 와 달리 한 차로치보다 작아도 되고(사고현장은 좁다), 경계는
        합법 폭이 아니라 **물리 폭**이다. 실제로 비었는지는 호출부가 `side_clear` 로 본다.
        """
        far = None
        for row in rows:
            edge = row[1] + d_ego + side * row[4] / 2.0    # 그 물체가 탈출 방향으로 뻗은 끝
            if far is None or side * edge > side * far:
                far = edge
        if far is None:
            return None
        room = self._phys_room(plan, side)
        need = side * far + self.HALF_WIDTH + self.DEAD_ESCAPE_CLEAR      # 최소
        if need <= 0.0 or need > self.ESCAPE_MAX_W or need + self.HALF_WIDTH > room:
            return None
        # 물리 폭이 남으면 더 벌린다(최대 DEAD_ESCAPE_CLEAR_MAX) — 남는 폭은 여유로 쓴다
        return min(side * far + self.HALF_WIDTH + self.DEAD_ESCAPE_CLEAR_MAX,
                   room - self.HALF_WIDTH)

    PO_SQUEEZE_M = 0.3          # 앞 도로가 목표보다 이만큼 더 좁아야 '밀린다'로 본다[m]

    @classmethod
    def _po_squeeze(cls, r_ahead, po_edge):
        """앞에서 도로 우측이 좁아져 **안쪽(왼쪽)으로 밀릴 것**인가.

        ★단위를 맞췄다(2026-09-08). 예전 식은
            `r_ahead - HALF_WIDTH - 0.15 < po_edge - 0.3`
        였는데, 왼쪽은 **오프셋**(경로 기준 얼마나 옮길 수 있나)이고 오른쪽은
        **가장자리 거리**(도로 우측 끝에서 얼마나 떨어져 설 것인가)다. 서로 다른 자를
        비교하니 차 반폭만큼 항상 기울어 **거의 언제나 참**이 됐다.

        실측 2026-09-08 사전테스트1 종점: 목표 `po_edge` 는 이미 최소값 **1.443m**
        (반폭 0.943 + 여유 0.5)이고 앞 도로도 **1.44m** 라 밀릴 데가 없는데,
        옛 식은 `0.347 < 1.143` 으로 참을 냈다. 그래서 붙기 내내 좌측이 켜졌다.
        둘 다 가장자리 거리로 재면 `1.44 < 1.143` 이 거짓 — 정답이다.

        A·G 종점(끝나는 차로에서 안쪽으로 밀림)은 그대로 잡힌다. 거긴 넓은 자리에서
        `po_edge` 가 크게 잡힌 뒤 도로가 그보다 좁아지는 경우다.
        """
        return r_ahead is not None and r_ahead < po_edge - cls.PO_SQUEEZE_M

    # 붙기 지시등 방향을 정하는 값들
    PO_DIR_M = 0.4              # 남은 이동이 이보다 크면 그 방향[m]
    PO_DIR_V = 0.25             # 횡속도가 이보다 크면 그 방향[m/s]
    PO_DIR_CMD = 0.05           # **이 프레임에 명령한** 오프셋 변화가 이보다 크면 그 방향[m] — 추정치보다 먼저 본다
    PO_DIR_HOLD_S = 0.8         # 방향이 바뀌려면 새 방향이 이만큼 이어져야 한다[s] — 한두 프레임 요동은 무시
    BEH_LEFT_MIN_S = 0.5        # 행동층 좌측 지시등이 이만큼 이어져야 붙기 우측을 밀어낸다[s]

    @classmethod
    def _po_direction(cls, delta, lat_v, squeeze, cmd=0.0):
        """붙는 중 지시등 방향(+1 좌 / -1 우 / 0 없음).

        delta   : 남은 이동(도로 기준, 왼쪽 +)
        lat_v   : 지금 횡속도(도로 기준, 왼쪽 +)
        squeeze : 앞에서 도로 우측이 좁아져 안쪽으로 밀릴 것
        cmd     : **이 프레임에 오프셋 제어기에 명령한 변화**(want - 지금 오프셋, 왼콝 +)

        ★★`cmd` 가 가장 먼저다(2026-09-09 스윕 G·H 항목13 -3). `lat_v` 는 실제 횡속도의
          EMA 라 **움직이기 시작한 첫 0.5초는 0 근처**다. 그 틈에 `squeeze`/`delta` 가
          좌측을 켰다 — 실측 H t=396.9~397.6: `off` 0.00 -> **-0.24(오른쪽)** 로 가는데
          지시등은 L. G t=440.0~440.9: off -1.25 -> -2.02(오른쪽) 인데 L. 둘 다 -3.
          우리가 이 프레임에 오른쪽으로 가라고 **명령한 것**은 추정이 아니다 — 그걸 먼저 본다.

        ★★**지금 실제로 가고 있는 쪽이 이긴다.** 예전엔 `squeeze` 가 먼저라,
        오른쪽으로 붙는 내내 좌측이 켜졌다.
        실측 2026-09-08 사전테스트1 종점(792.8,484.7): 종점 앞 54m 에서 도로 우측 여유가
        **4.55m -> 1.44m** 로 좁아져 `squeeze` 가 계속 참이었다. 그런데 차는 그 6.5초 동안
        `off` 0.00 -> **-1.28**(오른쪽)로 붙는 중이었다. 화면에는 "오른쪽으로 가면서 좌측
        깜빡이"가 됐다. 사용자: "오른쪽 가차선으로 붙는데 왼쪽 깜빡이 키고 난리남".
        `squeeze` 는 **지금 아무 데도 안 가고 있을 때만** 앞일을 예고하는 값이어야 한다.
        """
        if cmd < -cls.PO_DIR_CMD or delta < -cls.PO_DIR_M or lat_v < -cls.PO_DIR_V:
            return -1                                   # 오른쪽으로 가는 중/가야 함
        if cmd > cls.PO_DIR_CMD or delta > cls.PO_DIR_M or lat_v > cls.PO_DIR_V or squeeze:
            return 1
        return 0

    @classmethod
    def _po_direction_committed(cls, here, off, want, floor_ahead):
        """붙기가 **시작된 뒤**의 지시등 방향(+1 좌 / -1 우 / 0 없음).

        here        : 지금 지점에서 오른쪽으로 갈 수 있는 한계 오프셋(도로 끝 바닥, 왼쪽 +)
        off         : 지금 오프셋
        want        : 이 프레임의 목표 오프셋(한 방향 규칙까지 걸러진 값)
        floor_ahead : 앞 30m 의 그 바닥. 모르면 None

        ★★시작한 뒤에는 **명령한 것만** 본다(2026-09-11 사전주행2). 예전엔 '원래 목표까지
          남은 거리'(`_po_edge - car_edge`)로 쟀는데, 한 방향 규칙이 명령을 묶어 둔 사이
          그 거리는 왼쪽을 가리킨다 — 실측 t=114.1~115.9 에 오른쪽으로 가면서 좌측이 1.9초.
        ★왼쪽은 **도로가 좁아져 실제로 안쪽으로 밀릴 때**뿐이다. 지금 밀리는 중이거나(`here`),
          앞에서 밀릴 것이 확실할 때(`floor_ahead`). 둘 다 `off` 와 같은 자(오프셋)로 잰다 —
          `_po_squeeze` 는 가장자리 거리와 오프셋을 섞어 재서 널뛰는 `r` 에 속았다.
        """
        if here > off + 1e-3:
            return 1                                # 지금 안쪽으로 밀리는 중
        if want < off - 1e-3:
            return -1                               # 오른쪽으로 가라고 명령 중
        if floor_ahead is not None and floor_ahead > off + 0.05:
            return 1                                # 앞에서 차로가 끝난다 — 밀릴 것이 확실
        return 0                                    # 제자리 — `_final_turn` 이 우측을 유지한다

    def _final_turn(self, turn_av, turn_beh, s_remain):
        """지시등 우선순위: 추월기 > **종점 붙기** > 그 외(경로 차로변경·회전).

        붙는 중 지시등 = **도로 기준 순 횡이동 방향**(`_pullover_offset` 주석, 2026-09-06).
        왼쪽이 남았으면 좌측, 오른쪽으로 가면 우측, 가만히 있으면 종점 `PULLOVER_SIGNAL_M`
        안에서 우측(정차 [시행령 별표2]).

        ★★**붙는 중에는 붙기가 이긴다**(2026-09-06 코스 G 항목13 -3). 예전엔 붙기가
        `turn == 0` 일 때만 지시등을 잡아서, 경로에 박힌 차로변경 신호(왼쪽)와 프레임
        단위로 번갈아 켜졌다 — 실측 t=506.8~509.4 에 `2,1,1,1,1,2,2,2,2,2,2` 로,
        오른쪽이 **연속**으로 켜진 건 0.8초뿐이라 '선행점등 0.8초 < 3.0초' 로 -3.
        (사용자가 여러 번 지적한 "깜빡이 난리남"도 같은 원인이다.)
        붙는 중이면 경로의 차로변경은 어차피 **안 한다** — 우리는 서려고 오른쪽으로 간다.
        추월기는 그대로 우선한다. 그건 지금 실제로 하는 기동이다.
        """
        po_turn = 0
        # 행동층 좌측은 **연속으로 켜져 있어야** 인정한다 — 코너 출구에서 한 프레임 튀는
        # 경로 차로변경 신호(실측 A t=574.09 `2,1,2`)가 우측 선행 점등을 끊고 -3 을 만들었다.
        beh_left = (turn_beh == TS_LEFT and self._beh_left_age >= self.BEH_LEFT_MIN_S)
        if self._po_on:
            if self._po_net_left > 0:
                po_turn = TS_LEFT
            elif self._po_net_left < 0 or s_remain <= self.PULLOVER_SIGNAL_M:
                po_turn = TS_RIGHT
            elif not beh_left:
                po_turn = TS_RIGHT                    # 방향 미정(0)이면 우측을 **유지**한다 — 끊지 않는다
        elif self._po_presig and not beh_left:
            # ★★**선행 점등 구간**(2026-09-09 코스 B 항목13 -3): 48km/h 로 오면 시작점(90m)에서
            #   종점까지 6.5초뿐이라 3.5초 선행을 기다리면 붙기를 못 끝낸다 -> `_reach` 가
            #   바로 움직였고 지시등과 이동이 **같은 프레임**에 시작됐다(선행 1.2초 < 3초).
            #   시작점 PULLOVER_SIG_LEAD_S*속도 앞에서 우측을 먼저 켠다. 붙기는 언제나
            #   오른쪽으로 가는 일이라(`_po_edge <= car_edge`) 방향은 정해져 있다.
            po_turn = TS_RIGHT
        if turn_av != 0:
            return turn_av
        return po_turn if po_turn != 0 else turn_beh

    def _pending_lat_signal(self, off, d_ego, ov_state, plan):
        """차체가 명령 자리(off)로 **아직 옮기는 중**인 방향 — +1 좌 / -1 우 / 0.

        시작은 교차로 밖·FOLLOW 에서 벗어난 거리가 LAT_PENDING_SIG 를 넘을 때만(회전 중 코너를
        파고드는 추종 오차에 켜지지 않게). 한 번 켜면 LAT_PENDING_DONE 안으로 들어올 때까지
        교차로 안에서도 유지한다 — 옮기다 교차로에 들어갔다고 끄면 거기서 지시등 없이 옮긴다.
        """
        pend = off - d_ego
        # ★출발·리스폰 직후에는 차가 경로 옆에 놓여 있다(실측 v6 시작 3.6m) — 그걸 제자리로
        #   붙는 동안 켜면 매 판 첫 3초에 우측이 켜진다(녹화 6판 재생: A·B·D·H 전부 t=0~3.4).
        #   **한 번 제자리에 들어온 뒤** 벗어난 것만 '옮기는 중'으로 본다.
        if abs(pend) < self.LAT_PENDING_DONE:
            self._lat_settled = True
        if self._pend_sig != 0:
            if abs(pend) < self.LAT_PENDING_DONE or (pend > 0) != (self._pend_sig > 0):
                return 0
            return self._pend_sig
        if (self._lat_settled and ov_state == "FOLLOW" and plan and not plan.get("j")
                and abs(pend) > self.LAT_PENDING_SIG):
            return 1 if pend > 0 else -1
        return 0

    def _plan_at(self, bi):
        """bi 지점의 차로 계획 dict (lane/w/l/r/xl/xr/need). 모르면 None."""
        return self.lane_plan[bi] if (self.lane_plan and bi < len(self.lane_plan)) else None

    def _base_offset(self, plan, dt):
        """지정차로(회전 차로 등)를 지키기 위한 **기본** 횡오프셋. 변화율을 제한해 부드럽게.

        ⚠️ 교차로 안(plan=None)에서는 **지금 값을 유지**한다. 0으로 되돌리면 회전 연결로
           한복판에서 차로를 가로질러 어중간하게 걸치고, VTD가 경로이탈로 보고 리스폰시킨다
           (2026-08-15 실측: 좌회전 차로 진입은 성공했는데 교차로 안에서 감쇠하다 리스폰).
           교차로를 빠져나와 다시 차로를 알게 되면 그때 need(=0)로 돌아간다.
        """
        if plan is None:
            return self._base
        target = plan["need"] if self.turn_lane else 0.0
        step = self.LANE_RATE * max(dt, 1e-3)
        self._base += max(-step, min(step, target - self._base))
        if abs(self._base) < 1e-3:
            self._base = 0.0
        return self._base

    # ※시도했다 되돌림: '앞 45m 구간을 통째로 확인하고 시작' (2026-08-15).
    #   교차로 밖에서 시작한 추월이 교차로 안으로 이어져 리스폰이 났길래 넣어봤는데,
    #   **장애물이 교차로 안에 있으면 어차피 나가야 해서** 대기만 길어졌다.
    #   EV_COMBO_CHAIN 164s/리스폰2 -> 224s/리스폰2 (60초 손해, 리스폰 그대로) -> 폐기.
    #   시작점 판정만으로도 EV_OBSTACLE 은 리스폰 1 -> 0 이 된다.

    def _side_for(self, plan, need, desperate=False, exclude=0.0):
        """need[m] 만큼 비킬 수 있는 방향(+1 좌 / -1 우). 없으면 0(비키면 안 됨).

        기본 모드는 `desperate=True` 여도 같은 방향 차로(`l/r`)만 쓴다. 차로 배치를 모르는
        `plan=None` 에서는 0을 반환해 새 횡기동을 시작하지 않는다.

        `ALLOW_CENTERLINE_ESCAPE=1` 을 명시한 경우에만 오래 갇힌 상황에서 종전의 `xl/xr`
        및 물리적으로 넓은 쪽 전략을 쓴다. 이는 평가상 중앙선 침범이며 면책되지 않는
        완주 우선 비정상/실험 전략이다.
        """
        if not plan:
            if desperate and self.allow_centerline_escape:
                self._log_centerline_escape("차로 배치 미상 구간")
                return self.default_side
            # ★★**차로 미상이면 예전대로 기본 방향을 준다**(2026-09-08).
            #   차로계획이 없으면 중앙선이 어디인지 모른다 — 그래서 막고 싶어진다. 그런데
            #   **실주행에서는 여기로 오지 않는다.** `drive.sh` 가 언제나 차로계획을 함께
            #   넘긴다(경로 끝을 넘어선 인덱스 정도가 예외). 여기로 오는 건 **오프라인 모의**
            #   뿐이고, 모의에는 차로계획이 아예 없어 모든 판이 여기를 탄다.
            #   실측: 이 갈래를 막으니 회귀 4판이 통째로 미완주가 됐다(static_obstacle ·
            #   v3_leadbrake · hz_blocker_leaves · hz_wrongway — 전부 `OVT:WAIT` 로 굳음).
            #   '갇혔을 때만' 으로 절충해도 20초 뒤에야 풀려 40초짜리 판은 못 끝낸다.
            #   유일한 안전망을 잃는 것보다 예전 동작이 낫다. 중앙선 정책은 아래 `xl/xr`
            #   잠금이 지킨다 — 차로계획이 있을 때 그게 작동한다.
            return self.default_side
        if plan["l"] >= need and exclude != +1.0:
            return +1.0                              # 같은 방향 좌측 차로
        if plan["r"] >= need and exclude != -1.0:
            return -1.0                              # 같은 방향 우측 차로
        if desperate and self.allow_centerline_escape:
            if plan["xl"] >= need and exclude != +1.0:
                self._log_centerline_escape("xl 반대 차로 후보")
                return +1.0                          # 편도1차선 + 점선 -> 반대차선
            if plan["xr"] >= need and exclude != -1.0:
                self._log_centerline_escape("xr 반대 차로 후보")
                return -1.0
            # 실험 모드: 차 한 대 폭만 있으면 종전처럼 물리적으로 넓은 쪽을 택한다.
            room_l = max(plan["l"], plan["xl"]) if exclude != +1.0 else -1.0
            room_r = max(plan["r"], plan["xr"]) if exclude != -1.0 else -1.0
            # ★★차 한 대 폭은 있어야 '넓은 쪽'이다(2026-09-06). 0.5m 문턱으로는 코스 E 신호114
            #   에서 r=1.36m(인도까지 여유)를 골라 **인도로 올라가** 리스폰했다. 2.0m 는 차폭
            #   1.886 + 여유 — 그 아래는 옆으로 비킬 수 있는 폭이 아니라 연석이다.
            if max(room_l, room_r) >= self.DESPERATE_MIN_ROOM:
                self._log_centerline_escape("물리적으로 넓은 쪽 후보")
                return +1.0 if room_l >= room_r else -1.0
        if desperate:
            # ★★**같은 방향 차도 안에서의 탈출은 막지 않는다**(2026-09-08).
            #   주최측 답변은 **중앙선**에 예외가 없다는 것이지, 내 쪽 차도 안에서 비키는
            #   것까지 금지한 게 아니다. `xl/xr`(중앙선 너머)만 잠그면 그 정책은 지켜진다.
            #   ⚠️ 실측으로 확인했다: 이 갈래까지 잠그니 오프라인 회귀 **4판이 미완주**로
            #      떨어졌다 — static_obstacle · v3_leadbrake · hz_blocker_leaves · hz_wrongway.
            #      넷 다 '목표 미도달'이고 넷 다 **중앙선과 무관한 자기 차도 안 회피**다.
            #      교차로·차로미상 게이트만 풀어서는 안 살아난다(그것만 푼 판을 따로 돌려
            #      확인했다 — static_obstacle·v3_leadbrake 여전히 미도달).
            #   미완주는 0점이다. 중앙선을 지키면서 이건 살려야 한다.
            same_l = plan["l"] if exclude != +1.0 else -1.0
            same_r = plan["r"] if exclude != -1.0 else -1.0
            if max(same_l, same_r) >= self.DESPERATE_MIN_ROOM:
                return +1.0 if same_l >= same_r else -1.0
        return 0.0                                   # 합법적으로 비킬 데가 없다

    def _log_centerline_escape(self, detail):
        """실험 전략 선택을 한 번만, 면책이 아님을 포함해 경고한다."""
        if self._centerline_escape_selected_logged:
            return
        self._centerline_escape_selected_logged = True
        print(f"[drive] ⚠ 중앙선 탈출 선택({detail}) — 평가상 중앙선 침범이며 "
              "면책되지 않는 완주 우선 비정상/실험 전략", flush=True)

    def _rf_is_vru(self, row):
        """경로축 치수가 아닌 9910 **원본 치수**로 취약대상을 판정한다.

        ★속도도 본다(`vtd_io.is_vru` 주석). 그리고 **한 번 빠르게 달린 id 는 느려져도
          차량으로 남긴다** — 이륜차가 앞에서 2km/h 로 기어갈 때 다시 사람으로
          바뀌면 분류가 프레임마다 뒤집혀 그게 곧 요동이다. 객체 id 는 시나리오 안에서
          유지된다(주최측 답변 2026-09-04).
        """
        ds, d, spd, olen, owid, ohgt, oid = row[:7]
        raw_w = row[7] if len(row) > 7 else owid
        raw_l = row[8] if len(row) > 8 else olen
        if spd > VRU_MAX_SPEED:
            self._fast_ids.add(oid)
        if oid in self._fast_ids:
            return False
        return is_vru(raw_l, raw_w, ohgt, spd)

    def _lat_free(self, off, rf_objs, d_ego):
        """경로기준 횡위치 `off` 로 갔을 때 내 차체가 **아는 물체 전부**를 피하는가.

        ⚠️ 차폭은 `owid` 로 잰다(`max(olen,owid)` 아님). 도로에 정렬해 선 차의 횡폭은
           길이가 아니라 폭이다 — 길이로 재면 옆 차로 하나 건너까지 '막힘'이 되어
           빠져나갈 자리를 스스로 지운다. 이건 behavior 의 안전망과 같은 규약이다.
        """
        for row in rf_objs:
            ds, d, _spd, olen, owid, ohgt, _oid = row[:7]
            if not (self.NUDGE_BEHIND < ds < self.NUDGE_AHEAD):
                continue
            if classify_object(olen, owid, ohgt) is ObjectClass.ROAD_SURFACE:
                continue                                # 노면 취급 — 밟아도 된다
            is_ped = self._rf_is_vru(row)
            gap = abs((d + d_ego) - off) - self.HALF_WIDTH - owid / 2.0
            if gap < (self.FIT_GAP_PED if is_ped else self.FIT_GAP_CAR):
                return False
        return True

    def _return_blocked(self, target, rf_objs, d_ego):
        """nudge 를 `target` 으로 옮기면 **아직 안 지나간, 비켜 가던 물체** 옆에 몸이 안 들어가나.

        `_nudge_offset` 이 비켜 가는 것(서 있는 사람·작은 물건)만 본다 — 같은 거름망이다.
        움직이는 사람은 행동층이 세우고, 차량은 추월기 몫이다. d_ego 도 같은 인자(base 기준).
        ★여유는 `_lat_free` 와 **같은 자**(사람 FIT_GAP_PED·물건 FIT_GAP_CAR)로 잰다. 그래서
          `_lat_free` 를 통과한 목표(옆 차로 중앙 등)는 여기서 안 막히고, 검사 없이 떨어지는
          0 만 걸린다. 비킬 때 쓰는 `need`(여유 1.5m)로 재면 차로 폭이 좁아 **정상 목표까지**
          붙잡는다(옆 차로 3.0m 중앙이 사람에게서 need 3.44m 안이다).
        """
        if abs(target - self._nudge) < 1e-3:
            return False
        for row in rf_objs:
            ds, d, spd, olen, owid, ohgt, _oid = row[:7]
            if not (self.NUDGE_BEHIND < ds < self.NUDGE_AHEAD):
                continue                                # 이미 지나갔다 -> 돌아가도 된다
            if classify_object(olen, owid, ohgt) is ObjectClass.ROAD_SURFACE:
                continue
            is_ped = self._rf_is_vru(row)
            if not is_ped and max(olen, owid) >= self.NUDGE_MAX_DIM:
                continue
            if spd > 0.5:
                continue
            d_path = d + d_ego
            gap_to = abs(d_path - target) - self.HALF_WIDTH - owid / 2.0
            if (gap_to < (self.FIT_GAP_PED if is_ped else self.FIT_GAP_CAR)
                    and abs(d_path - target) < abs(d_path - self._nudge)):
                return True
        return False

    def _nudge_rear_clear(self, off, rf_objs, d_ego, ego_speed):
        """`off` 로 비켜도 되나 — 그 자리에 **뒤에서 오는 차**나 이미 옆에 붙은 차가 없나.

        `_lat_free` 는 앞(-3m~40m)만 본다. 옆 차로를 뒤에서 빠르게 오는 차는 안 보인다.
        실측 2026-09-10 코스 H (966,657): 우측으로 3.0m 나가는 동안 오른쪽 뒤 10.4m 의
        6.8m/s 차가 옆에 붙어 실여유 -0.24m 접촉. 뒤를 35m 까지 본다(LC_REAR_WATCH 와 같다).
        ⚠️ **나보다 느린 차는 문제 삼지 않는다** — 안 그러면 정체에서 영영 못 비킨다.
        """
        if abs(off) < 0.3:
            return True
        for row in rf_objs:
            ds, d, spd, olen, owid = row[0], row[1], row[2], row[3], row[4]
            if self._rf_is_vru(row):
                continue                               # 사람은 위에서 이미 본다
            if max(olen, owid) < 1.2:
                continue                               # 콘·작은 물건
            if abs((d + d_ego) - off) - self.HALF_WIDTH - owid / 2.0 >= self.NUDGE_SIDE_GAP:
                continue                               # 내가 갈 자리가 아니다
            # ⚠️ **몸이 겹치는지**로 가른다. 완전히 뒤에 선 차는 내가 옆으로 비켜도 안 닿지만,
            #    앞끝이 내 뒷범퍼를 넘어와 있으면 서 있어도 옆으로 가면 긁는다.
            if (ds + olen / 2.0 <= -self.EGO_REAR_EDGE
                    and spd < self.NUDGE_REAR_MIN_V):
                continue                               # 뒤에 **서 있는** 차(내가 만든 정체)
            if self.NUDGE_BEHIND <= ds <= 0.0:
                return False                           # 이미 옆에 붙어 있다
            if -self.NUDGE_REAR_WATCH < ds < self.NUDGE_BEHIND:
                if spd > ego_speed - self.NUDGE_REAR_SLOW:
                    return False                       # 뒤에서 따라붙는 차
        return True

    def _cw_body_fwd(self, s, key):
        """그 횡단보도 **본체**까지 전방거리[m]. 모르면 None.

        ⚠️ `_crosswalk_ahead` 가 주는 거리는 **정지선**까지다(본체보다 몇 m 앞). 사람이 그
        횡단보도 위에 있나를 정지선 기준으로 재면 본체가 창 밖으로 나간다 — 실측 2026-09-10
        코스 H: 정지선 0.75m 앞에 서 있었고 사람은 8.9m 앞(본체 위)이라 |8.9-0.75|=8.2 로
        6m 창을 벗어나 **한 명도 못 셌다**.
        """
        c = self._cw_by_key.get(key)
        if c is None:
            return None
        dx, dy = c["x"] - s.x, c["y"] - s.y
        return dx * math.cos(-s.heading) - dy * math.sin(-s.heading)

    def _cw_stop_fwd(self, s, key, body_fwd):
        """그 횡단보도에서 **내가 서야 할 지점**까지 전방거리[m]. 내 방향 도색 정지선이 있으면
        그 앞, 없으면 본체 앞 `CW_GAP_BODY`.

        ⚠️ `_crosswalk_ahead` 의 거리는 경로투영을 쓰는데, 코너에서는 그 값이 실제 전방거리와
           크게 다르다(실측 2026-09-10 코스 H: 본체가 앞범퍼 9.0m 인데 투영은 0.75m). 사람이
           그 위에 있나·어디에 설까는 **자차 기준 실제 거리**로 재는 게 맞다.
        """
        c = self._cw_by_key.get(key)
        if c is None:
            return None
        c_, s_ = math.cos(-s.heading), math.sin(-s.heading)
        best = None
        for q in c.get("stops", ()):
            dx, dy = q[0] - s.x, q[1] - s.y
            qf = dx * c_ - dy * s_
            if abs(dx * s_ + dy * c_) > self.CW_LAT:
                continue
            if len(q) > 2 and math.cos(q[2] - s.heading) < 0.5:
                continue                       # 나를 세우는 선이 아니다(건너편·교차 방향)
            if 0.0 < qf <= body_fwd + 0.5 and (best is None or qf > best):
                best = qf
        if best is not None:
            return best - self.CW_GAP_LINE
        return body_fwd - self.CW_GAP_BODY

    CW_PED_LEAVE_LANES = 1.0    # 멀어지는 사람은 차도 가장자리에서 이만큼(차로 수) 더 나가야 '다 건넜다'

    def _ped_blocks(self, rf_objs, d_ego):
        """앞 25m 안에 **서 있는** 사람이 차로 안 서행 통과로는 실여유 PED_BLOCK_CLR 을 못 남기나."""
        return any(0.0 < row[0] < 25.0
                   and abs(row[1] + d_ego) - self.HALF_WIDTH - row[4] / 2.0 < self.PED_BLOCK_CLR
                   for row in rf_objs if self._rf_is_vru(row) and row[2] < 0.5)

    def _cwp_latch(self, key, v_cmd, v_cwp, d_tgt):
        """횡단보도 위 사람이 있는 동안의 속도 상한 — **지금 속도에서 정지선에 0 이 되게 줄여 간다**.

        v_cmd : 이 프레임 행동층이 낸 속도(YIELD_PED·NARROW_BLOCK 등이 이미 들어 있다)
        v_cwp : 정지선까지 감속 곡선이 허용하는 속도
        d_tgt : 정지 목표(정지선 앞)까지 남은 거리[m] — v_cwp 를 만든 그 거리
        사람이 내려가면(`_ped_on_crosswalk` 가 빈 목록) 호출부가 `_cwp_hold` 에서 지운다.

        ★★(2026-09-11 코스 B, 사용자 "사람이 차 정면을 건너는데 왜 기어가냐. 정지선이면
          정지선에서 딱 멈춰야지"). 처음엔 '내려가기만 하는 최솟값'이었다(PR #46). 그러면
          YIELD_PED 가 잠깐 준 7.2km/h 를 **붙잡고 그 속도 그대로** 정지선까지 굴러간다 —
          실측 t=15.7~17.5 에 6~7km/h 로 2.5m 를 기었다. 붙잡을 것은 속도가 아니라
          **'지금 속도에서 정지선에 0 이 되는 감속'**이다: v = v_last·√(d/d_last).
          멀어질 일이 없으니(다가가기만 한다) 값은 내려가기만 하고, 한 번 0 이면 계속 0 이다.
        """
        prev = self._cwp_hold.get(key)
        d = max(0.0, d_tgt)
        if prev is None:
            v = min(v_cmd, v_cwp)
        else:
            v_last, d_last = prev
            taper = 0.0 if d_last <= 0.3 else v_last * math.sqrt(min(d, d_last) / d_last)
            v = min(v_cmd, v_cwp, taper)
        self._cwp_hold[key] = (v, d)
        return v

    def _ped_on_crosswalk(self, rf_objs, cw_fwd, d_ego, plan=None):
        """그 횡단보도 위에 사람이 있나 -> [(정지해 있나), ...]. 없으면 빈 목록."""
        out = []
        _p = plan or {}
        _w = float(_p.get("w") or 3.3)
        _edge_l = float(_p.get("l") or 0.0) + _w * self.CW_PED_LEAVE_LANES
        _edge_r = float(_p.get("r") or 0.0) + _w * self.CW_PED_LEAVE_LANES
        for row in rf_objs:
            if not self._rf_is_vru(row):
                continue
            ds, d, spd = row[0], row[1], row[2]
            if abs(ds - cw_fwd) > self.CW_PED_LONG:
                continue
            dp = d + d_ego
            lat = abs(dp)
            if lat > self.CW_PED_LAT:
                # 걷는 중 + 가로지르는 방향 + **내 진로 쪽으로 오는 중**이면 멀리서도 센다.
                if lat > self.CW_PED_LAT_MOVING or spd <= self.CW_STOP_V:
                    continue
                _dh = math.degrees(row[9]) if len(row) > 9 else 90.0
                _adh = min(abs(_dh) % 360.0, 360.0 - abs(_dh) % 360.0)
                if not (self.CW_PED_CROSS_DEG[0] <= _adh <= self.CW_PED_CROSS_DEG[1]):
                    continue
                # ★★**멀어지는 사람은 세지 않는다**(2026-09-11 코스 H). 경로 기준 왼쪽(+)에
                #   있으면서 더 왼쪽으로 가면 이미 건넌 것이다. 가까워지는 쪽만 남긴다.
                # ★★단 **'멀어진다'가 곧 '다 건넜다'는 아니다**(2026-09-11 코스 B TRV).
                #   횡 3m(`CW_PED_LAT`)만 넘으면 바로 뺐더니, 왕복 6.5m 차도를 건너던 사람이
                #   아직 횡단보도 위인데 우리가 출발해 42km/h 로 그 옆을 지났다.
                #   사용자: "사람이 횡단보도를 다 지나가야 움직여야지 반 지나가자마자 움직이노".
                #   **내 차도를 벗어나 한 차로 더 나갈 때까지**는 센다 — 지도가 아는 선이고,
                #   그 뒤엔 우리 진로와 무관하다. 위 `CW_PED_LAT_MOVING`(9m)이 상한이다.
                if dp * math.sin(math.radians(_dh)) >= 0.0:
                    if lat > (_edge_l if dp > 0.0 else _edge_r):
                        continue
            out.append(spd < self.CW_STOP_V)
        return out

    def _plan_idx_at(self, s):
        """경로 누적거리 s[m] 에 해당하는 경로점(= 차로계획) 번호. 차로계획은 __init__ 에서 경로에 맞춰 깔았다."""
        i = bisect.bisect_left(self.cum, s)
        return max(0, min(len(self.cum) - 1, i))

    def _carriage_c(self, s):
        """누적거리 s 에서 (같은 방향 차도 중심의 경로 기준 횡위치[좌 +], 차도 폭). 모르면 None.

        차로계획의 l·r 은 **경로에서** 차도 가장자리까지다. 경로가 오른쪽 차로로 옮겨 가면 l 은 늘고
        r 은 줄고 합(차도 폭)은 그대로다 — 그래서 (l-r)/2 의 변화가 곧 경로가 도로 위에서 옮겨 간 양이다.
        """
        p = self._plan_at(self._plan_idx_at(s))
        if not p or p.get("j") or p.get("l") is None or p.get("r") is None:
            return None
        return (p["l"] - p["r"]) / 2.0, p["l"] + p["r"]

    def _lc_hold_target(self, rf_objs, d_ego, bi):
        """경로가 **서 있는 사람이 있는 차로로 차선을 바꿔 들어가는** 중이면, 지금 차로를 지키는 nudge
        목표(경로 기준, d_ego 와 같은 기준). 해당 없으면 None.

        ★★(2026-09-11 코스 H TRV (1480,515)) 경로가 3차로 -> 4차로로 옮겨 가는 **바로 그 자리의 4차로**에
          휠체어가 서 있었다. 경로 기준으로 재면 휠체어가 '진로 우 1.0m' 라 반대쪽으로 **한 차로를 통째로**
          비켰다 — 경로가 아직 3차로에 있던 자리라 차가 2차로 쪽으로 튀었다. 지도로 보면 휠체어는 4차로,
          우리는 3차로라 **그대로 가면 1.65m 떨어져 지나간다**. 사용자: "옆 차선에 서있고만 왜 차선을 바꿔".
          사람이 할 일은 '3차로로 계속 가서 휠체어를 지난 뒤 4차로로' 다. 차도 기준 지금 자리를 잡아 두고
          (`q`), 경로가 옮겨 가는 만큼 매 프레임 되돌려 준다 — 사람을 지나면 풀려서 경로의 차선변경을 마저 한다.
        조건: 서 있는 사람 · 경로로는 스친다(nudge 대상) · 그 사이 차도 폭 그대로(갈래·차로 증감 아님)
              · 경로가 사람 쪽으로 LC_HOLD_MIN_DRIFT 넘게 옮겨 간다 · 지금 차로로 가면 몸 옆에 FIT_GAP_PED 가 남는다.
        """
        if not self.lane_plan or bi is None or bi >= len(self.cum):
            self._lc_hold = None
            return None
        s0 = self.cum[bi]
        ce = self._carriage_c(s0)
        if ce is None:
            self._lc_hold = None
            return None
        held = self._lc_hold
        found = None
        for row in rf_objs:
            ds, d, spd, olen, owid, _ohgt, oid = row[:7]
            if not (self.NUDGE_BEHIND < ds < self.NUDGE_AHEAD):
                continue                                # 지나갔다 -> 경로의 차선변경을 마저 한다
            if spd > 0.5 or not self._rf_is_vru(row):
                continue
            co = self._carriage_c(s0 + max(ds, 0.0))
            if co is None or abs(co[1] - ce[1]) > self.LC_HOLD_WIDTH_TOL:
                continue                                # 차로가 늘거나 준다 — 경로 쏠림이 아니다
            d_path = d + d_ego                          # 그 사람 자리에서의 경로 기준 횡위치
            need = self.HALF_WIDTH + max(olen, owid) / 2.0 + self.PED_BYPASS_CLEAR
            if abs(d_path) >= need:
                continue                                # 경로대로 가도 안 스친다
            mine = held is not None and held[0] == oid
            q = held[1] if mine else d_ego - ce[0]      # 차도 중심 기준 내 자리(좌 +)
            hold_o = q + co[0]                          # 그 사람 자리에서 '지금 차로'의 경로 기준 위치
            drift = co[0] - ce[0]                       # + 면 경로가 그 사이 오른쪽으로 옮겨 간다
            toward = ((d_path < hold_o and drift > self.LC_HOLD_MIN_DRIFT)
                      or (d_path > hold_o and drift < -self.LC_HOLD_MIN_DRIFT))
            if not (toward or mine):
                continue
            if abs(d_path - hold_o) - self.HALF_WIDTH - owid / 2.0 < self.FIT_GAP_PED:
                continue                                # 지금 차로로도 못 지난다 -> 원래 비키기
            found = (oid, q)
            break
        if found is None:
            self._lc_hold = None
            return None
        self._lc_hold = found
        cl = self._carriage_c(s0 + self.LC_HOLD_LOOK)
        c_look = cl[0] if cl is not None else ce[0]
        return max(-self.NUDGE_MAX, min(self.NUDGE_MAX, found[1] + c_look))

    def _lc_hold_view(self, rf_objs, bi):
        """경로 차선변경을 미루는 중(`_lc_hold`)이면 그 사람의 횡위치를 **지금 차로 기준**으로 고친 rf_objs.

        rf_objs 의 d 는 그 사람 자리의 **경로** 기준이다. 경로가 그 사이 옆 차로로 옮겨 가면, 옆 차로에
        선 사람이 '내 진로 안'으로 잡힌다. 차로를 지키는 동안은 경로가 옮겨 간 양(차도 중심 기준 (l-r)/2 의
        변화)을 빼서 행동층·'막았나' 판정에 넘긴다. 실측 2026-09-11 코스 H TRV: 휠체어가 경로 기준 우 1.0m,
        지금 차로 기준 우 3.2m — 경로 기준으로 보고 45km/h 에서 a=-5 로 서다가 7km/h 로 기었다.
        사용자: "그냥 가면 되는거아님??".
        ⚠️ 붙잡은 그 사람 **하나만** 고친다. 다른 물체·nudge 자체는 원래 값을 쓴다.
        """
        h = self._lc_hold
        if h is None or bi is None or bi >= len(self.cum):
            return rf_objs
        s0 = self.cum[bi]
        ce = self._carriage_c(s0)
        if ce is None:
            return rf_objs
        out = []
        for row in rf_objs:
            if len(row) > 6 and row[6] == h[0]:
                co = self._carriage_c(s0 + max(row[0], 0.0))
                if co is not None:
                    row = (row[0], row[1] - (co[0] - ce[0])) + tuple(row[2:])
            out.append(row)
        return out

    def _nudge_offset(self, rf_objs, d_ego, dt, plan, ped_bypass=False, ego_speed=0.0,
                      hold_lat=False, force=None, bi=None):
        """인레인 장애물을 비켜갈 횡오프셋[m]. 없으면 0으로 되돌아온다.

        ⚠️ 어정쩡하게 1.6m 만 걸치면 그 자체가 진로위반이고 차선을 밟은 채 달리게 된다.
           **옆 차로를 통째로 쓸 수 있으면 차로 하나를 온전히 옮긴다.** 그럴 여유가 없을
           때만(편도 1차선 등) 최소 필요량만 비킨다.

        rf_objs 의 d 는 **ego 기준** 상대 횡거리(d_obj - d_ego)다. 그대로 목표를 만들면
        내가 비킬수록 목표가 같이 도망가 진동한다 -> 경로기준 좌표(d_path)로 되돌려 고정한다.

        ped_bypass=True 면 사람도 대상에 넣는다(오래 막혔을 때만. 여유를 훨씬 크게 준다).
        """
        # ★★**신호에 서 있는 동안에는 옆으로 밀지 않는다**(2026-09-10 코스 H (1331,203)).
        #   실측: 적신호에 선 채로 오프셋이 -3.16m 까지 갔다가 -0.90 으로 되돌아왔다. 차는
        #   1cm 도 안 움직이는데 **조향이 ±35°로 돌고 지시등이 좌우로 뒤집혔다**. 사용자:
        #   "에바임". 정지 중 횡이동은 물리적으로 불가능하니 명령만 요동친 것이다.
        #   움직이기 시작하면 그때 다시 민다.
        if hold_lat:
            return self._nudge
        lane_w = (plan or {}).get("w", 3.3)
        # ★사고현장 우회(`_esc_want`)는 합법 차로가 아니라 **물리 통로**로 나가는 것이라
        #   아래의 차로 기반 판정을 통째로 건너뛴다. 뒤차 확인은 그대로 받는다.
        if force is not None:
            target = max(-self.NUDGE_MAX, min(self.NUDGE_MAX, force))
            if not self._nudge_rear_clear(target, rf_objs, d_ego, ego_speed):
                target = self._nudge
            step = self.NUDGE_RATE * max(dt, 1e-3)
            self._nudge += max(-step, min(step, target - self._nudge))
            return self._nudge
        # ★경로가 사람 있는 차로로 차선을 바꿔 들어가는 중이면 지금 차로를 지킨다(`_lc_hold_target`).
        #   차로를 바꾸는 게 아니라 **지키는** 것이라 비킬 자리 검사(`_lat_free`)·뒤차 확인은 안 탄다 —
        #   내 차로 앞뒤 차는 원래 거기 있는 차다. 차로 안 앞차·사람은 행동층 안전망이 본다.
        hold = self._lc_hold_target(rf_objs, d_ego, bi)
        if hold is not None:
            step = self.NUDGE_RATE * max(dt, 1e-3)
            self._nudge += max(-step, min(step, hold - self._nudge))
            return self._nudge
        target = 0.0
        for row in rf_objs:
            ds, d, _spd, olen, owid, ohgt, _oid = row[:7]
            if not (self.NUDGE_BEHIND < ds < self.NUDGE_AHEAD):
                continue
            if classify_object(olen, owid, ohgt) is ObjectClass.ROAD_SURFACE:
                continue                                    # 노면 취급
            is_ped = self._rf_is_vru(row)
            if not is_ped and max(olen, owid) >= self.NUDGE_MAX_DIM:
                # 취약대상이 아닌 큰 객체/차량 -> 추월 FSM
                continue
            if is_ped and _spd > 0.5:
                continue                                    # 횡단 중인 사람 -> 진로가 겹친다, 정지
            if is_ped and not ped_bypass:
                continue                                    # 비킬 차로가 없다 -> 정지
            d_path = d + d_ego                              # 경로 기준으로 환원
            half = max(olen, owid) / 2.0
            clear = self.PED_BYPASS_CLEAR if is_ped else self.NUDGE_CLEAR
            need = self.HALF_WIDTH + half + clear
            if abs(d_path) >= need:                         # 경로 그대로 가도 안 스친다
                continue
            side = self._side_for(plan, need + self.HALF_WIDTH)
            if side == 0.0:                                 # 합법적으로 비킬 데가 없다
                continue
            if abs(d_path) > 0.3:                           # 물체가 이미 한쪽에 치우쳐 있으면
                side = 1.0 if d_path < 0 else -1.0          # 반대쪽으로(가까운 쪽으로 안 감)
            # 차로 하나를 통째로 쓸 수 있으면 그렇게. 아니면 최소 필요량만.
            room = (plan or {}).get("l" if side > 0 else "r", 0.0)
            full = side * lane_w
            want = full if room >= lane_w + self.HALF_WIDTH else d_path + side * need
            if abs(want) > abs(target):
                target = max(-self.NUDGE_MAX, min(self.NUDGE_MAX, want))

        # ★비킬 자리에 **다른 물체가 있으면 그 자리는 못 쓴다.** 거기로 가라고 하면
        #   차체 안전망(NARROW_BLOCK)이 우리를 세우고, nudge 는 계속 그리로 가라 해서
        #   아무도 물러서지 않는다 — 실측 2026-08-18 EV_COMBO_PASSPED: 250초 정지 후
        #   미완주, 게다가 STALL_ESCAPE 가 그 오프셋을 문 채 기어가 정지차에 0.81m
        #   겹치기까지 했다.
        # ★한 차로만 보면 안 된다. 그 판의 우여유는 **7.93m** 였다 — 정지차 **너머에**
        #   빈 차로가 하나 더 있었는데 NUDGE_MAX=3.8 이 그 자리를 아예 가리고 있었다.
        #   막혔으면 차로 단위로 더 나가 보고, 그래도 없으면 0(= 안 비키고 선다).
        #   서는 건 감점이지만 들이받는 것과 갇히는 것보다 낫다.
        if target and not self._lat_free(target, rf_objs, d_ego):
            side = 1.0 if target > 0 else -1.0
            room = (plan or {}).get("l" if side > 0 else "r", 0.0)
            target = 0.0
            for k in range(2, self.NUDGE_LANES + 1):
                cand = side * k * lane_w
                if abs(cand) + self.HALF_WIDTH > room:
                    break                               # 그쪽으론 더 갈 데가 없다
                if self._lat_free(cand, rf_objs, d_ego):
                    target = cand
                    break

        # ★비킨 자리에 **뒤에서 오는 차**가 있으면 더 들어가지 않는다(_nudge_rear_clear 주석).
        #   0 으로 되돌리지 않고 **지금 자리를 유지**한다 — 반쯤 나간 채 급히 돌아오는 것도
        #   그 자체로 위험하다. 그 차가 지나가면 다음 프레임에 다시 나간다.
        if target and not self._nudge_rear_clear(target, rf_objs, d_ego, ego_speed):
            target = self._nudge

        # ★★**비켜 가던 그 물체가 아직 앞에 있으면 그쪽으로 되돌아가지 않는다**(2026-09-11 코스 H TRV).
        #   실측 (1478,495): 차로 안 오른쪽 1.0m 에 선 휠체어를 좌측 차로(+3.17)로 비키는 중,
        #   298.8초에 뒤에서 온 차가 우리와 휠체어 사이로 파고들어 목표 자리 여유가 0.34m
        #   (< FIT_GAP_CAR 0.35)가 됐다. 위 `_lat_free` 가 막힘이라며 target=0 을 줬고
        #   nudge 가 3.17 -> 0 으로 무너지며 **휠체어 쪽으로 핸들을 -35° 꺾었다** — 그 차(-0.12m)·
        #   자전거(-0.24m)와 스쳤다. 그 뒤로는 왼쪽 차로 40m 안에 **달아나는** 차가 있어 다시
        #   못 나갔고, 차로 중심으로 기어 들어가다 휠체어에 **실여유 -0.18m** 로 닿았다.
        #   사용자: "사람보고 망함".
        #   '0 = 안 비키고 선다'는 아직 안 나갔을 때 얘기다. 이미 나가 있으면 0 은 **비켜 간
        #   것을 향해 돌아간다**가 된다. 그러면 지금 자리를 유지한다(위 뒤차 확인과 같은 원칙).
        #   그 물체를 지나가면(NUDGE_BEHIND) 풀려서 예전처럼 돌아온다.
        if self._return_blocked(target, rf_objs, d_ego):
            target = self._nudge

        step = self.NUDGE_RATE * max(dt, 1e-3)
        self._nudge += max(-step, min(step, target - self._nudge))
        if abs(self._nudge) < 1e-3:
            self._nudge = 0.0
        return self._nudge

    def _local_curve(self, bi):
        """앞 `TURN_CURVE_LOOK` m 동안 경로 방위가 꺾이는 양[도]. 국소 곡률.

        `_upcoming_turn` 과 다르다 — 저건 35m 를 내다보고 '교차로 회전인가'를 묻는
        **지시등용**이고, 이건 10m 앞 **지금 돌고 있는 곡률**이다.
        """
        s0 = self.cum[bi] if bi < len(self.cum) else 0.0
        j = bi
        while j < len(self.route) - 1 and self.cum[j] - s0 < self.TURN_CURVE_LOOK:
            j += 1
        if j <= bi + 1:
            return 0.0

        def hd(k):
            a = self.route[max(0, k - 2)]
            b = self.route[min(len(self.route) - 1, k + 2)]
            return math.atan2(b[1] - a[1], b[0] - a[0])
        return abs(math.degrees((hd(j) - hd(bi) + math.pi) % (2 * math.pi) - math.pi))

    def _junction_ahead(self, bi):
        """전방 **진짜 교차로**(jx) 진입까지 거리[m]. 없으면 None.

        신호 없는 교차로에서 **우회전 일시정지** 지점을 잡으려면 이게 필요하다 —
        신호가 있으면 정지선을 쓰지만, 없으면 교차로 입구가 정지점이다.

        ★`j` 가 아니라 `jx` 를 본다. 갈래 2개짜리 junction 은 그냥 모퉁이라
          일시정지할 이유도, 꼬리물기를 걱정할 이유도 없다(_upcoming_turn 주석 참고).
        """
        if not self.lane_plan:
            return None
        s0 = self.cum[bi] if bi < len(self.cum) else 0.0
        j = bi
        while j < len(self.lane_plan) and self.cum[j] - s0 < self.JUNC_LOOK:
            p = self.lane_plan[j]
            if p is not None and p.get("jx"):
                return self.cum[j] - s0
            j += 1
        return None

    def _junction_span(self, bi):
        """전방 교차로의 (진입까지, 빠져나올 때까지) 거리[m]. 없으면 None.

        꼬리물기 판정에 쓴다 — '들어가서 나올 때까지' 얼마나 필요한지 알아야
        그만큼 앞이 비었는지 물어볼 수 있다.
        """
        if not self.lane_plan:
            return None
        s0 = self.cum[bi] if bi < len(self.cum) else 0.0
        j, entry = bi, None
        while j < len(self.lane_plan) and self.cum[j] - s0 < self.JUNC_LOOK:
            p = self.lane_plan[j]
            # ★꼬리물기도 **진짜 교차로**만 본다 — 모퉁이엔 막을 교차 교통이 없다.
            inj = p is not None and p.get("jx")
            if entry is None:
                if inj:
                    entry = self.cum[j] - s0
            elif not inj:
                return (entry, self.cum[j] - s0)
            j += 1
        # ⚠️ 내다보는 거리 안에서 안 끝나면 **끝을 모르는 것**이다. 창 끝을 출구로 쓰면
        #    엉뚱한 자리를 검사하게 되니 None 을 준다(호출부가 규칙을 건너뛴다).
        return (entry, None) if entry is not None else None

    def _upcoming_turn(self, bi):
        """앞쪽 경로가 크게 꺾이면 그 방향(+1 좌 / -1 우). 회전 방향지시등용."""
        j = bi
        s0 = self.cum[bi] if bi < len(self.cum) else 0.0
        while j < len(self.route) - 1 and self.cum[j] - s0 < self.TURN_SIGNAL_AHEAD:
            j += 1
        if j <= bi + 2:
            return 0
        def hd(k):
            a = self.route[max(0, k - 3)]; b = self.route[min(len(self.route) - 1, k + 3)]
            return math.atan2(b[1] - a[1], b[0] - a[0])
        dh = (hd(j) - hd(bi) + math.pi) % (2 * math.pi) - math.pi
        if abs(dh) < self.TURN_SIGNAL_MIN:
            return 0
        # ★**교차로가 앞에 있을 때만** 회전으로 본다. 2026-08-20 사용자 지적:
        #   "도로가 좀 꺾여서 오른쪽으로 꺾인 도로에서 왜 오른쪽 깜빡이를 킴".
        #   실측 (856.5,114.8): 35m 방위변화 -28.7°인데 **그 구간에 교차로가 없다**.
        #   반경 80m 짜리 평범한 도로 곡선인데 25° 문턱을 넘어 우측 지시등이 켜졌다.
        #   도로교통법 제38조①은 **회전·진로변경** 때 신호하라는 것이지, 커브를 따라갈
        #   때가 아니다. 커브는 회전이 아니므로 켜면 오점등이다.
        #   (진로변경 지시등은 overtaker/차로계획이 따로 담당한다.)
        #   ★★2026-08-30: 여기서 `j`(교차로 안) 를 보다가 **`jx`(진짜 교차로)** 로 바꿨다.
        #      xodr 은 두 도로가 만나 꺾이는 **모퉁이**도 junction 으로 표시한다 —
        #      이 맵은 junction 94개 중 **41개(44%)가 갈래 2개**뿐이고, 코스 A 는
        #      j=1 점 699개 중 **259개(37%)** 가 그런 모퉁이다.
        #      사용자 지적 2026-08-30 코스 E (785,571): "길 자체가 그냥 휘어진 도로다.
        #      좌회전이 아니라 그냥 길이 굽은 도로" — junction 91 은 incomingRoad 가
        #      [2813, 2814] 둘뿐이고 road 3099 는 52.7m 에 93° 꺾이는 굽은 길이다.
        #      가로지를 교통이 없으니 회전도 아니다. 그런데 여기서 좌측 지시등이 켜졌고
        #      `route_turn>0` 이 되면서 **대향차 양보(YIELD_CROSS)로 26.9->0.0km/h** 섰다.
        if self.lane_plan:
            s_j = self.cum[j] if j < len(self.cum) else self.cum[-1]
            if not any(self.lane_plan[k] and self.lane_plan[k].get("jx")
                       for k in range(bi, min(len(self.lane_plan), j + 1))
                       if self.cum[k] - s0 <= s_j - s0):
                return 0
        return 1 if dh > 0 else -1

    def _sig_lead(self, speed):
        """지금 속도에서 필요한 지시등 선행거리[m] — 법(30m)과 대회(3초) 둘 다 만족."""
        return min(self.TURN_SIG_LEAD_MAX,
                   max(self.TURN_SIG_LEAD, self.TURN_SIG_SEC * max(speed, 0.0) + 4.0))

    def _signal_turn(self, bi, speed=0.0):
        """**지시등용** 회전 예고. 앞 `_sig_lead(speed)` 안에 '급한 회전'이 시작되면 그 방향.

        `_upcoming_turn` 과 다르다 — 저건 판단용(양보·비보호좌회전)이라 창이 35m 로
        고정이고, 그래서 급한 코너일수록 점등이 늦어진다(위 TURN_SIG_LEAD 주석).
        여기서는 코너의 급함을 짧은 창(TURN_SIG_WIN)으로 재고, **그 코너가 LEAD 안에
        있기만 하면** 켠다. 선행 거리가 코너 각도와 무관해진다.
        """
        return self._turn_within(bi, self._sig_lead(speed))

    def _turn_within(self, bi, lookahead):
        """앞 `lookahead` m 안의 첫 실제 교차로 회전(+좌/-우), 없으면 0.

        지시등과 우회전 직전 추월 금지가 같은 회전 판정을 공유한다. 서로 다른 기하
        판정을 쓰면 지시등은 우회전을 알고 있는데 추월기는 모르는 구간이 생긴다.
        """
        if bi >= len(self.cum):
            return 0
        s0 = self.cum[bi]

        def hd(k):
            a = self.route[max(0, k - 3)]
            b = self.route[min(len(self.route) - 1, k + 3)]
            return math.atan2(b[1] - a[1], b[0] - a[0])

        k = bi
        while k < len(self.route) - 1 and self.cum[k] - s0 <= lookahead:
            j = k
            while j < len(self.route) - 1 and self.cum[j] - self.cum[k] < self.TURN_SIG_WIN:
                j += 1
            if j > k + 2:
                dh = (hd(j) - hd(k) + math.pi) % (2 * math.pi) - math.pi
                if abs(dh) >= self.TURN_SIGNAL_MIN:
                    # 커브가 아니라 **진짜 교차로**여야 회전이다(_upcoming_turn 과 같은 규칙).
                    if not self.lane_plan or any(
                            self.lane_plan[m] and self.lane_plan[m].get("jx")
                            for m in range(k, min(len(self.lane_plan), j + 1))):
                        return 1 if dh > 0 else -1
            k += 1
        return 0

    def _lc_rear_yield(self, plan, rf_objs, d_ego, speed):
        """경로에 박힌 차선변경 중, 목표 차로 **뒤에서 오는 차**에 양보할 속도[m/s].

        양보가 필요 없으면 None. 위 LC_REAR_* 주석 참고 — 개입은 감속뿐이다.
        """
        sig = (plan or {}).get("sig", 0)
        if not sig or plan.get("j"):
            return None                       # 교차로 안은 회전 담당이다
        lane_w = plan.get("w") or 3.3
        # 목표 차로 중심(경로 기준 절대 횡위치). d>0 = 좌, sig>0 = 좌.
        want = d_ego + sig * lane_w
        half = lane_w / 2.0 + 0.3
        cap = None
        for ds, d, sp, _ol, _ow, oh, _oid, *_ in rf_objs:
            if oh < 0.8:                      # 노면물·연석은 뒤차가 아니다
                continue
            if not (-self.LC_REAR_WATCH < ds < 0.0):
                continue                      # 뒤에 있는 것만
            if abs((d + d_ego) - want) > half:
                continue                      # 목표 차로 밖
            closing = sp - speed
            if closing <= 0.3:
                continue                      # 안 다가온다 — 내가 더 빠르면 그냥 간다
            if (-ds) / closing > self.LC_REAR_TTC:
                continue                      # 아직 멀다
            # ★**내 속도보다 느리게** 잡아야 실제로 먼저 보낸다. 뒤차 속도에서 빼면
            #   그 차가 나보다 훨씬 빠를 때 상한이 내 속도보다 높아져 아무 효과가 없다.
            want_v = max(self.LC_REAR_MIN_V,
                         min(speed, sp) - self.LC_REAR_MARGIN)
            cap = want_v if cap is None else min(cap, want_v)
        return cap

    def _pullover_blocked(self, want, rf_objs, d_ego, s_remain) -> bool:
        """`want` 까지 훑고 갈 띠가 비었나. 내 차로 안(현재 위치 왼쪽)은 보지 않는다."""
        lo = want - self.HALF_WIDTH - self.PULLOVER_CLEAR_PAD
        hi = self._po_off - self.HALF_WIDTH
        for ds, d, _sp, _ol, owid, _oh, _oid, *_ in rf_objs:
            if not (-self.PULLOVER_CLEAR_BACK < ds < s_remain + 6.0):
                continue
            dp = d + d_ego                                  # 경로 기준 절대 횡위치
            if dp + owid / 2.0 > lo and dp - owid / 2.0 < hi:
                return True
        return False

    def _pullover_target(self, plan, s_remain, rf_objs, d_ego, speed) -> Optional[float]:
        """붙일 목표 오프셋(오른쪽 = 음수). 아직 고를 때가 아니면 None.

        ★후보는 **차로 중심**이다. `r - w/2` 가 오른쪽에 남은 차로들의 총 폭이니
          그걸 차로폭으로 나누면 몇 차로가 남았는지 나온다. 중간값을 목표로 삼으면
          **차선 위에 걸친 채로 서게 된다** — 안 옮긴 것보다 나쁘다.
        ★남은 거리로 **끝낼 수 있는 것만** 고른다. 공식 코스는 같은 방향 4차로에
          접근 50km/h 라, 40m 로는 한 차로도 못 끝낸다(실측).
        """
        if plan.get("j") or abs(plan.get("need") or 0.0) > 0.1:
            return None                         # 교차로 안 / 지정차로 이동 중에는 안 고른다
        gp = self._po_goal_plan
        if gp is None:
            return None
        w = float(gp.get("w") or 0.0)
        room = float(gp.get("r") or 0.0)
        if w < 1.0:
            return None
        right_room = room - w / 2.0             # 오른쪽에 남은 차로들의 총 폭
        n = int(round(right_room / w)) if right_room > 0 else 0
        if n >= 1:
            lw = right_room / n                 # 오른쪽 차로 평균 폭
            top = n if self.PULLOVER_MODE == "edge" else min(n, self.PULLOVER_MAX_LANES)
            cands = [k * lw for k in range(top, 0, -1)]
        else:
            # 오른쪽에 차로가 없다 = 이미 가장자리 차로. 차로 안에서 오른쪽으로 붙인다.
            cands = [max(0.0, room - self.HALF_WIDTH - self.PULLOVER_EDGE_GAP)]
        t_go = s_remain / max(speed, 2.0)
        for c in cands:
            if c < 0.25:
                return 0.0                      # 붙일 것도 없다 — 지시등만 켠다
            if c / self.PULLOVER_RATE_MAX > t_go - 1.0:
                continue                        # 남은 거리로는 못 끝낸다
            if self._pullover_blocked(-c, rf_objs, d_ego, s_remain):
                continue                        # 그 자리에 차가 있다
            return -c
        return None                             # 지금은 아무것도 못 고른다 — 다음 프레임에

    def _r_min_to_goal(self, bi):
        """`bi` 부터 **종점까지** 차로계획 최소 `r`(오른쪽 끝까지 거리). 모르면 None.

        ★★붙기가 **한 방향**으로 끝나려면 목표를 '지금 넓이'가 아니라 **앞으로 가장 좁은 곳**
          에 맞춰야 한다(2026-09-11 코스 A 종점). 실측: 종점 52m 앞 우측 여유 4.78m 를 보고
          -3.28m 까지 붙었는데 35m 앞에서 도로가 3.47m 로 좁아져(물리 실측 `pr` 도 3.4m —
          지도 잡음이 아니라 진짜 좁아진다) `here` 바닥이 -2.38 로 **0.9m 끌어당겼다**.
          지시등도 그 3초 동안 좌측으로 갔다. 사용자: "마무리 부분 지랄남".
          좁은 곳을 지나고 나면 이 값이 다시 커져 나머지를 마저 붙는다 — 되돌림이 없다.
        """
        if bi is None or not self.lane_plan:
            return None
        if self._r_suffix is None:
            n = len(self.lane_plan)
            suf = [None] * (n + 1)
            for k in range(n - 1, -1, -1):
                q = self.lane_plan[k]
                v = None if (not q or q.get("j")) else float(q.get("r") or 0.0)
                nxt = suf[k + 1]
                suf[k] = v if nxt is None else (nxt if v is None else min(v, nxt))
            self._r_suffix = suf
        return self._r_suffix[bi] if 0 <= bi < len(self._r_suffix) else None

    def _edge_ahead_min(self, bi, look_m=30.0):
        """앞 look_m 안 차로계획의 최소 `r`(오른쪽 끝까지 거리). 끝나는 차로(테이퍼) 예고용. 모르면 None."""
        if bi is None or not self.lane_plan or bi >= len(self.lane_plan):
            return None
        s0 = self.cum[bi] if bi < len(self.cum) else 0.0
        best = None
        for k in range(bi, min(len(self.lane_plan), len(self.cum))):
            if self.cum[k] - s0 > look_m:
                break
            q = self.lane_plan[k]
            if not q or q.get("j"):
                continue
            rr = float(q.get("r") or 0.0)
            best = rr if best is None else min(best, rr)
        return best

    def _pullover_offset(self, plan, s_remain, rf_objs, d_ego, dt, speed, bi=None) -> float:
        """종점 우측 정차 오프셋(오른쪽 = 음수). [법 제34조 · 시행령 제11조①1]
        `plan["r"]` = 그 점에서 **같은 방향 차도의 오른쪽 끝**까지 거리[m].

        ★★목표는 **도로 기준**으로 고정한다(2026-09-06): '오른쪽 끝에서 `_po_edge` m'.
          예전엔 경로 기준 오프셋(-3.1 등)을 고정했다. 그런데 종점 앞 경로는 주최측 좌표(1차로)로
          **왼쪽으로 합류**하므로, 그 오프셋을 따라가면 차가 경로와 함께 왼쪽으로 끌려가다 오른쪽으로
          되돌아온다 — 코스 A·G 종점 지그재그, 지시등 L/R 번갈아(사용자: "우측 깜빡이 키고 자꾸
          왼쪽으로 차선 변경"). 도로 기준으로 고정하면 경로가 어디로 합류하든 차는 **제 차로를
          지키다가**, 끝나는 차로(테이퍼)에서만 자연히 안쪽으로 밀린다(`here` 클램프가 그 일을 한다).
        ★지시등은 **도로 기준 순 횡이동**으로: 남은 이동·실제 횡속도가 왼쪽이면 좌측, 오른쪽이면 우측,
          가만히 있으면 종점 PULLOVER_SIGNAL_M 안에서 우측(정차 [시행령 별표2]). 앞 30m 에 차로가
          끝나면(`_edge_ahead_min`) 밀려날 것이 확실하니 미리 좌측을 켠다.
        """
        if self.PULLOVER_MODE == "off":
            return 0.0
        start = (self.PULLOVER_START_EDGE_M if self.PULLOVER_MODE == "edge"
                 else self.PULLOVER_START_M)
        # 선행 점등 구간: 시작점에 닿기 PULLOVER_SIG_LEAD_S 전(그 속도로). `_final_turn` 이 우측을 켠다.
        self._po_presig = s_remain <= start + max(speed, 2.0) * self.PULLOVER_SIG_LEAD_S
        if s_remain > start:
            self._po_on = False
            self._po_off = 0.0
            self._po_want = None
            self._po_edge = None
            self._po_edge_prev = None
            self._po_lat_v = 0.0
            self._po_net_left = 0
            self._po_dir_cand = 0
            self._po_dir_hold = 0.0
            self._po_started = False
            return 0.0
        if plan is None or plan.get("j"):
            return self._po_off                     # 교차로 안: 지금 값 유지
        r_now = float(plan.get("r") or 0.0)
        car_edge = r_now + d_ego                    # 지금 차가 오른쪽 끝에서 떨어진 거리
        if self._po_edge is None:
            pick = self._pullover_target(plan, s_remain, rf_objs, d_ego, speed)
            if pick is not None:
                gp = self._po_goal_plan or {}
                r_goal = float(gp.get("r") or r_now)
                # 종점 기준 목표(종점 차로에서 pick 만큼 오른쪽)보다 **지금 자리가 더 오른쪽이면 지금 자리**.
                # 붙기는 오른쪽으로 가는 일이지 왼쪽으로 옮기는 일이 아니다 — 끝나는 차로는 클램프가 밀어낸다.
                edge = min(r_goal + pick, car_edge)
                # ★★그런데 '지금 자리'가 **차선 위**일 수 있다(2026-09-11 사전주행2 종점).
                #   목표를 고르는 순간 경로가 마침 차로를 옮기는 중이면 `car_edge` 는 두 차로
                #   사이다 — 실측 6.76m(차로 중심은 5.05 / 8.45m). 그 값을 그대로 붙잡고 종점까지
                #   가면 **차선을 밟은 채로 선다**. `_pullover_target` 이 후보를 차로 중심으로만
                #   고르는 이유가 바로 그건데(같은 함수 주석 "차선 위에 걸친 채로 서게 된다 —
                #   안 옮긴 것보다 나쁘다"), 이 클램프가 그걸 무너뜨렸다.
                #   왼쪽으로는 어차피 안 가니 **지금 자리보다 오른쪽에 있는 가장 가까운 차로
                #   중심**으로 내린다. 같은 방향 차로가 2개뿐인 공식 6코스는 후보가 하나뿐이라
                #   이 가지가 아예 걸리지 않는다(실측: 종점 우측 여유 1.75~4.85m).
                if edge < r_goal + pick - 1e-6:
                    w_goal = float(gp.get("w") or 0.0)
                    rr = r_goal - w_goal / 2.0
                    nn = int(round(rr / w_goal)) if (w_goal > 1.0 and rr > 0.0) else 0
                    if nn >= 1:
                        lwg = rr / nn
                        below = [r_goal - k * lwg for k in range(nn + 1)
                                 if r_goal - k * lwg <= edge + 0.05]
                        if below:
                            edge = max(below)
                self._po_edge = max(self.HALF_WIDTH + self.PULLOVER_EDGE_GAP, edge)
                self._po_want = pick
                # 고른 순간 이미 그 자리면 '움직일 것 없음' — 이후 경로 추적(차는 가만히)은 게이트 대상이 아니다.
                self._po_started = abs(self._po_edge - car_edge) <= 0.3
        if self._po_edge_prev is not None and dt > 1e-3:
            self._po_lat_v = 0.7 * self._po_lat_v + 0.3 * (car_edge - self._po_edge_prev) / dt
        self._po_edge_prev = car_edge
        if self._po_edge is None:
            self._po_on = s_remain <= self.PULLOVER_SIGNAL_M
            self._po_net_left = 0
            return self._po_off
        want = self._po_edge - r_now                # 경로 기준 오프셋(음수 = 오른쪽) — 도로 기준으론 고정
        here = -max(0.0, r_now - self.HALF_WIDTH - 0.15)
        want = max(want, here)
        # ★★**남은 거리로 못 끝낼 만큼은 요구하지 않는다**(2026-09-08).
        #   `_po_edge` 는 '도로 우측 끝에서 몇 m' 라 도로가 넓어지거나 경로가 왼쪽으로
        #   합류하면 요구량이 계속 커진다. 그 자체는 맞다 — 도로 기준으로 제자리를 지키는
        #   게 A·G 종점을 고친 설계다. 문제는 **끝낼 수 없는 양까지 요구**하는 것이다.
        #   실측 사전테스트2 종점: 오프셋이 -3.3 -> **-10.2m** 로 커졌고, 다 옮기지 못한 채
        #   종점에 닿아 **비스듬히 섰다**(조향 -25°). 사용자: "왜 대각선으로 멈춰".
        #   남은 거리에 갈 수 있는 만큼만 요구하면 목표는 그대로 두고 대각선만 없어진다.
        _t_left = s_remain / max(speed, 2.0)
        _reach = self._po_off - self.PULLOVER_RATE_MAX * max(0.0, _t_left - 0.5)
        want = max(want, _reach)
        # ★★**앞으로 가장 좁은 곳까지만 붙는다**(2026-09-11 코스 A 종점, `_r_min_to_goal` 주석).
        #   나갔다 들어오는 것을 애초에 만들지 않는다. 좁은 곳을 지나면 값이 커져 마저 붙는다.
        _narrow = self._r_min_to_goal(bi)
        if _narrow is not None:
            want = max(want, -max(0.0, _narrow - self.HALF_WIDTH - 0.15))
        # ★★**붙기는 한 방향 기동이다 — 시작한 뒤에는 왼쪽으로 되돌아가지 않는다**(2026-09-11
        #   사전주행2). `r` 이 널뛰면 목표가 좌우로 흔들리고, 그대로 따라가면 종점 앞 100m 를
        #   -1.4 -> +1.1 -> -5.1 로 갈지자로 간다. 사용자: "왜이럼". 도로가 좁아져 오른쪽으로
        #   더 못 있을 때는 아래 `here` 바닥이 왼쪽으로 밀어 준다 — 그게 유일한 되돌림이다.
        if self._po_started and want > self._po_off:
            want = self._po_off
        if want < -0.05 and self._pullover_blocked(want, rf_objs, d_ego, s_remain):
            want = self._po_off
        self._po_on = True
        delta = self._po_edge - car_edge            # 남은 이동(왼쪽 +)
        r_ahead = self._edge_ahead_min(bi)
        squeeze = self._po_squeeze(r_ahead, self._po_edge)
        # ★★**지시등은 지금 명령한 기동을 가리킨다**(2026-09-11 사전주행2 종점).
        #   붙기가 시작된 뒤 목표는 한 방향으로만 간다(한 방향 규칙, 위). 그런데 방향 판정은
        #   여전히 **원래 목표까지 남은 거리**(`delta = _po_edge - car_edge`)로 재고 있어서,
        #   지도의 `r` 이 널뛰는 구간에서 둘이 어긋났다 — 실측 t=114.1~115.9: 명령은 오른쪽
        #   (off -1.38 -> -2.30)인데 `delta` 는 +2.30(왼쪽)이라 **좌측 지시등이 1.9초** 켜졌다.
        #   오른쪽으로 가면서 좌측을 켠 것이다(항목13). 시작한 뒤에는 명령만 본다.
        #   왼쪽은 **도로가 좁아져 실제로 안쪽으로 밀릴 때**뿐 — 코스 A·G 종점(끝나는 차로)이
        #   그 경우다. `squeeze` 대신 `here` 바닥을 앞에다 대고 재면(같은 자, 아래) 밀릴 것을
        #   예고하면서 널뛰는 `r` 에는 안 속는다.
        if self._po_started:
            cand = self._po_direction_committed(
                here, self._po_off, want,
                None if r_ahead is None else -max(0.0, r_ahead - self.HALF_WIDTH - 0.15))
        else:
            cand = self._po_direction(delta, self._po_lat_v, squeeze, cmd=want - self._po_off)
        # ★디바운스: 방향이 바뀌려면 새 방향이 PO_DIR_HOLD_S 이어져야 한다. 실측 H 종점 앞
        #   t=395.1~397.7 에 R/L 이 **10번** 뒤집혔다 — 채점기는 마지막 전환부터 선행시간을
        #   재므로 한 번만 튀어도 -3 이다. 붙기는 하나의 기동이다. 지시등도 하나여야 한다.
        if cand == self._po_net_left:
            self._po_dir_cand, self._po_dir_hold = cand, 0.0
        elif cand == self._po_dir_cand:
            self._po_dir_hold += max(dt, 0.0)
            if self._po_dir_hold >= self.PO_DIR_HOLD_S:
                self._po_net_left = cand
        else:
            self._po_dir_cand, self._po_dir_hold = cand, 0.0
        # ★첫 이동 전 선행 점등: 그 방향 지시등이 PULLOVER_SIG_LEAD_S 켜져 있어야(항목13 3초).
        #   남은 거리로 못 끝낼 때만 바로 간다. 경로 이동을 상쇄하는 추적(차는 가만히)은 미루지 않는다.
        moving_needed = abs(delta) > 0.3            # 차가 **도로 기준으로** 움직여야 할 때만(경로 추적은 미루지 않는다)
        if (not self._po_started and moving_needed
                and self._sig_dir_age < self.PULLOVER_SIG_LEAD_S
                and s_remain / max(speed, 2.0) - (self.PULLOVER_SIG_LEAD_S - self._sig_dir_age)
                > abs(want - self._po_off) / self.PULLOVER_RATE_MAX + 1.0):
            return self._po_off
        if moving_needed:
            self._po_started = True
        # 종점까지 **남은 시간 안에 끝나도록** 속도를 잡는다(고정 속도면 못 끝낸다).
        t_go = s_remain / max(speed, 2.0)
        # ★★**목표가 도망가는 속도까지 더한다**(2026-09-11 코스 H 종점, 사용자 "왼쪽으로
        #   갔다가 오른쪽으로 가"). 코스 H 는 종점 60m 앞에서 **경로 자체가 왼쪽 차로로
        #   옮겨 간다**(lane -2 -> -1, 우측 여유 1.7 -> 4.7m). 그러면 '도로 오른쪽 끝에 붙는'
        #   목표(`want`)가 1.08m/s 로 오른쪽으로 도망가는데, 이동 속도는 `|want-off|/t_go`
        #   로만 잡혀 **0.7m/s** 에 묶였다. 그래서 차가 경로에 끌려 왼쪽으로 1.3m 밀렸다가
        #   막판에 다시 오른쪽으로 돌아왔다 — 화면에는 종점에서 좌우로 흔드는 걸로 보인다.
        #   목표가 움직이는 속도(feed-forward)를 더해야 **끌려가지 않고 따라간다**.
        _want_v = 0.0
        if self._po_want_prev is not None and dt > 1e-3:
            _want_v = abs(want - self._po_want_prev) / dt
        self._po_want_prev = want
        rate = min(self.PULLOVER_RATE_MAX,
                   max(self.PULLOVER_RATE,
                       abs(want - self._po_off) / max(t_go - 0.8, 0.4) + _want_v))
        if not moving_needed:
            rate = self.PULLOVER_RATE_MAX           # 경로가 옆으로 옮겨 가는 걸 **따라잡기만** 하는 중 — 늦으면 끌려간다
        step = rate * max(dt, 1e-3)
        # ★★**서 있는 동안에는 옆으로도 안 간다**(2026-09-11 사전주행2 신호15).
        #   붙는 중에 적색을 만나 15초 서 있는 사이 명령만 -6.59 -> -6.80 으로 더 갔다.
        #   차는 못 움직이니 그 차이가 **빚**으로 남고, 녹색에 출발하면서 남은 6.6m 안에
        #   비스듬히 갚는다 — 조향 -22.2°, 종점에서 도로와 20.6° 틀어져 섰다.
        #   옆으로 가려면 앞으로 가야 한다. 한 프레임에 옮길 양을 **간 거리**로도 묶는다.
        #   (`_nudge_offset` 은 이미 같은 이유로 신호 정지 중 오프셋을 얼린다.)
        #   ⚠️ 상한은 느슨하게(16.7°) — 공식 6코스의 붙기 기울기는 최대 7.5°라 안 걸린다.
        #      여기서 하려는 일은 '기울기 제한'이 아니라 **정지 중 적립 금지**다.
        step = min(step, self.PULLOVER_SLOPE_MAX * max(speed, 0.0) * max(dt, 1e-3))
        if want < self._po_off - step:
            self._po_off -= step
        elif want > self._po_off + step:
            self._po_off += step
        else:
            self._po_off = want
        self._po_off = max(self._po_off, here)
        return self._po_off

    # ---- 매 프레임 ----
    def step(self, s, dt, now) -> Command:
        if s.respawned:
            self.reset(now)

        # 누적 주행거리. 리스폰 점프(순간이동)는 빼고 **실제로 굴러간 것**만 센다.
        # ⚠️ `reset()` 에서 지우지 않는다 — 리스폰은 판을 다시 시작하는 게 아니다.
        if self._odo_xy is not None and not s.respawned:
            _d = math.hypot(s.x - self._odo_xy[0], s.y - self._odo_xy[1])
            if _d < 5.0:                        # 한 프레임 5m 초과는 점프다
                self._odo += _d
        self._odo_xy = (s.x, s.y)

        # ★수신 주기가 벌어지면 그만큼 느리게 달린다.
        #   왜(2026-08-16 v7 실측): VTD 출력이 25Hz -> 3.5Hz 로 떨어졌다. 우리 계산은
        #   0.8ms 라 멀쩡한데, **제어 갱신이 0.28초에 한 번**이면 50km/h 로 달릴 때
        #   한 번 판단에 3.9m 를 눈감고 가는 셈이다. 실제로 그 판에서 적신호를 6~10건
        #   그냥 지났다. 이유가 시뮬 쪽이어도 결과는 우리 감점이므로, 보이는 만큼만 간다.
        #   '한 프레임에 갈 거리'를 STEP_METERS 로 묶는다(20Hz·50km/h = 0.7m 가 정상값).
        self._dt_ema = dt if self._dt_ema is None else self._dt_ema * 0.9 + dt * 0.1
        rate_cap = self.STEP_METERS / max(self._dt_ema, 1e-3)

        bi = nearest_index(self.route, s.x, s.y, self._bi_prev)
        self._bi_prev = bi
        fwd = self.route[bi:bi + self.HORIZON_PTS] or self.route[-1:]
        vp = self.v_prof[bi] if bi < len(self.v_prof) else self.base_limit
        vp = min(vp, rate_cap)

        # 경로 끝: '남은 경로거리' 기준 (직선거리로만 보면 지나쳐도 못 멈춤)
        s_remain = max(0.0, self.total - self.cum[bi])
        vp = min(vp, math.sqrt(2 * self.GOAL_DEC
                               * max(0.0, s_remain - self.GOAL_STOP_BACK)))
        # ★★**붙기 구간에서는 가속하지 않는다**(2026-09-11 코스 H 종점, 사용자 "마무리가
        #   여전히 불안하네"). 실측: 종점 86m 앞에 19.9km/h 로 들어와 **45.9km/h 까지 가속**한
        #   뒤 마지막 30m 를 3.8m/s^2 로 급제동했다(계획 상한을 8km/h 초과). 붙는 중에 속도를
        #   올릴 이유가 없다 — 들어온 속도를 유지하고 종점 프로파일로 부드럽게 선다.
        #   ⚠️ 하한(PULLOVER_HOLD_V)을 두는 이유: 신호에 서 있다 출발하는 경우까지 묶으면
        #      종점 앞에서 기어가게 된다.
        if s_remain <= self.PULLOVER_START_M:
            vp = min(vp, max(s.speed, self.PULLOVER_HOLD_V))
        # ★★**달린 적이 없으면 완주가 아니다.** 최근접 인덱스만 보면, 차가 경로 어디에도
        #   없을 때 '끝점이 제일 가깝다' 는 이유로 **첫 프레임에 완주**가 된다.
        #   실측 2026-09-04 사전테스트 준비: 시나리오가 안 올라가 ego 가 (0,0) 이었는데
        #     [main] ✅ 목표 도달 (x=0.0 y=0.0, 0s)
        #   로 아무것도 안 하고 끝났다(그 경로는 시작까지 1690m·끝까지 928m 라 끝이 더 가깝다).
        #   대회장이었으면 **조용히 미완주**다. 리스폰이 엉뚱한 데로 보내도 같은 일이 난다.
        #   경로는 최소 수백 m 라 30m 문턱이 진짜 완주를 막지는 않는다.
        goal_reached = ((bi >= len(self.route) - 2 or s_remain < 1.5)
                        and self._odo >= self.GOAL_MIN_RUN)
        if goal_reached:
            vp = 0.0

        # ★리스폰이 반복되면(경로추종 실패 중) 속도를 낮춰 추종 정확도를 올린다.
        #   리스폰은 매번 감점이므로 느려도 이탈을 멈추는 게 이득(2026-08-15 루프 관측).
        self._respawns = [t0 for t0 in self._respawns if now - t0 < self.RESPAWN_WINDOW]
        if len(self._respawns) >= self.RESPAWN_TRIP:
            vp = min(vp, self.base_limit * self.RESPAWN_SLOW)

        # 객체를 '경로 기준'으로 투영(곡선에서 ego좌표 fy는 곡률에 오염되어 차선 오판)
        s_ego, d_ego, i_ego = self.rf.project(s.x, s.y, hint=bi)
        rf_objs, ov_objs = [], []
        for o in s.objects:
            if math.hypot(o.x - s.x, o.y - s.y) > 140.0:      # 멀면 투영 생략(연산 절약)
                continue
            s_o, d_o, i_o = self.rf.project(o.x, o.y, hint=i_ego, window=90)
            ds, d = s_o - s_ego, d_o - d_ego
            olen, owid = route_extent(o, self.rf.heading_at(i_o))
            # ★원본 폭도 같이 싣는다(index 7). route_extent 로 낸 olen/owid 는 **경로축
            #   외접상자**라, 비스듬한 자전거는 폭이 0.6m 인데 1.8m 로 부푼다. 무엇인지
            #   가르는 데 그 값을 쓰면 자전거를 승용차로 오인한다(실측 2026-08-24 접촉).
            # index 9 = **경로 기준 상대방위**. 안전망이 상자 대 상자(SAT)로
            #   실여유를 재려면 방위가 있어야 한다 — 경로축 외접상자로는 비스듬한 차의
            #   옆폭이 부풀어 없는 접촉을 만든다(run_logger 가 같은 이유로 SAT 를 쓴다).
            rf_objs.append((ds, d, o.speed, olen, owid, o.height, o.id, o.width,
                            o.length, o.heading - self.rf.heading_at(i_o)))
            # ★원본 폭·길이도 같이 넘긴다(index 7,8). 추월기가 **사람/이륜차를 가리려면**
            #   경로축 치수로는 안 된다 — VTD 는 보행자도 자전거도 2.0x0.6x1.7 로 준다.
            ov_objs.append((o.id, ds, d, o.speed, olen, owid, o.height,
                            o.width, o.length))
        plan = self._plan_at(bi)
        base = self._base_offset(plan, dt)      # 지정차로(회전 차로 등) 기본 오프셋
        base += self._pullover_offset(plan, s_remain, rf_objs, d_ego, dt, s.speed, bi=bi)

        # 추월 FSM -> 횡오프셋. 나가기 전에 지도로 방향부터 고른다(기동 중엔 안 바뀜).
        # ★비킬 폭은 **그 지점 실제 차로폭**으로 잰다(차로계획이 알고 있다).
        #   추월기 기본값(3.5m)을 쓰면 좁은 차로에서 필요폭을 과대평가해
        #   **갈 수 있는데 안 간다** — 실측 2026-08-20 새 코스 G: 실제 3.03m·여유 4.38m
        #   인데 4.44m 가 필요하다고 봐서 54초 갇히고 미완주(커버리지 20%).
        self.overtaker.set_lane_width((plan or {}).get("w"))
        lane_w = abs(self.overtaker.W)
        # 오래 갇힌 사실만으로 중앙선/교차로 금지를 풀지 않는다. 명시적 실험 옵션이
        # 켜진 경우에만 종전의 완주 우선 전략을 복원한다.
        stuck_long = (self._stalled_since is not None
                      and now - self._stalled_since > self.JUNCTION_WAIT)
        risky_relax = stuck_long and self.allow_centerline_escape
        # ★고른 쪽이 **실제로 막혀 있으면** 반대쪽을 시도한다. 지도는 '차선이 있다'만
        #   알려주지 '비어 있다'는 모른다 — 실측 2026-08-16 EV_BOTHBLOCK: 좌우 여유가
        #   똑같아(4.65/4.65) 좌측을 골랐는데 하필 그 좌측이 막힌 쪽이라 462초를 서 있었다.
        #   한 번만 뒤집는다(둘 다 막혔을 때 좌우로 진동하면 안 된다).
        # ★추월 방향은 **양쪽을 각각 물어보고** 고른다.
        #   예전엔 `target_clear`(마지막에 **선택된** 쪽의 결과)만 보고 한 번 뒤집었고,
        #   그 배제는 `state != "WAIT"` 일 때만 풀렸다. WAIT 에서 못 나오면 영영 안 풀린다.
        #   실측 2026-08-25 코스 E:
        #     t=332.0  WAIT · W=+3.03(좌)  clear=False   <- 도착한 그 순간 좌측이 막혔다
        #     t=332.0  _ovt_avoid=+1.0 · W=-3.03(우)     <- 좌측을 배제하고 우측으로
        #     이후 **250초 정지** — 우측은 정지차 4대가 벽이고 좌측은 곧 비었는데 못 돌아간다
        #   게다가 뒤집을지 묻는 기준이 '지금 선택된(막힌) 쪽'이라 **매 프레임 같은 결론**이
        #   나오는 순환이었다. 시한을 줘도 그 프레임에 곧바로 다시 뒤집혔다.
        #   -> 합법인 쪽을 모두 뽑아 **실제로 빈 쪽**을 고른다. 빈 쪽이 없으면 선호 쪽을 유지.
        #   ⚠️ WAIT 중에는 FSM 이 오프셋을 0 으로 두므로 좌우를 매 프레임 다시 봐도
        #      차는 흔들리지 않는다. 기동에 들어가면(`set_side` 가 PASS 중 무시) 고정된다.
        need = lane_w + self.HALF_WIDTH
        cands = []
        first = self._side_for(plan, need, desperate=stuck_long)
        if first != 0.0:
            cands.append(first)
            second = self._side_for(plan, need, desperate=stuck_long, exclude=first)
            if second != 0.0:
                cands.append(second)
        ov_side = cands[0] if cands else 0.0
        _clear = False
        for c in cands:
            if self.overtaker.side_clear(ov_objs, s.speed, d_ego - base, c):
                ov_side = c
                _clear = True
                break
        # ★★**한 차로치로 안 되면 막은 차 너머까지 본다** — 오래 갇혔을 때만.
        #   기본 모드의 `_escape_width` 는 같은 방향 차도 l/r 안으로 폭을 제한한다.
        #   실측 2026-09-06 코스 A: 역주행 13m 버스가 우리 차로와 왼쪽 차로를 같이 물고
        #   서서, 한 차로 추월은 목표차선이 곧 그 버스라 **188초 이동 0.00m 미완주**였다.
        #   `_escape_width` 주석에 그 자리 숫자가 다 있다. 넓힌 폭도 반드시 `side_clear`
        #   로 확인한 뒤에만 쓴다 — 비어 있지 않으면 원래 폭으로 되돌린다.
        if stuck_long and not _clear and ov_side != 0.0:
            _wide = self._escape_width(plan, rf_objs, d_ego, ov_side, lane_w)
            if _wide is not None:
                _keep = abs(self.overtaker.W)
                self.overtaker.set_escape_width(_wide)
                if self.overtaker.side_clear(ov_objs, s.speed, d_ego - base, ov_side):
                    same_room = ((plan or {}).get("l" if ov_side > 0 else "r") or 0.0)
                    if _wide + self.HALF_WIDTH > same_room:
                        self._log_centerline_escape("막은 차 너머 탈출 폭")
                    print(f"[drive] ⚠ 갇힘 탈출 — 막은 차 너머로 비킨다 "
                          f"{_keep:.2f} -> {_wide:.2f}m ({'좌' if ov_side > 0 else '우'})",
                          flush=True)
                else:
                    self.overtaker.set_escape_width(_keep)
        # ★★**죽은 차 앞 사고현장 우회**(DEAD_* 주석) — 두 갈래다: 합법 차로가 비었으면 잠금만 풀고,
        #   합법 통로가 아예 없으면 물리 폭으로 마지막 탈출을 시도한다.
        #   실측 2026-09-09 코스 E (915,216): 자전거(경로 폭 +0.0~+1.75)·직각 승용차(-5.4~-1.0)가
        #   내 차로와 우측 차로를 막았다. 우측 4.45m 는 그 차가 다 먹었고, 좌측은 합법 1.51m 뿐
        #   (교차로 연결로라 xl=0). 물리로는 왼쪽에 4.25m(대향 연결로)가 있다 — 그리로 3.19m 비켜
        #   자전거 옆 0.5m 로 지나가고, 대향 정지차(32m 앞) 앞에서 복귀한다.
        #   ⚠️ 막은 것들이 **전부** 오래 서 있어야 한다(_dead_blockers). 움직이던 차는 45초.
        #   물리 폭이 넓은 쪽부터 본다. 나가기 전 `side_clear`(차폭 기준 반폭)로 실제로 비었는지 본다.
        #   ★★한 번 나가기로 했으면 **지나갈 때까지 붙든다**(`_esc_latch`, 2026-09-10).
        #   `stuck_long` 은 우리가 1cm 라도 움직이면 리셋된다 — 래치가 없으면 기어나가는 순간
        #   조건이 꺼져 오프셋이 0 으로 돌아가고, 다시 막혀 서고, 20초 뒤 또 나가는 순환이 된다.
        escape_dead = False
        _cw_block = self._cw_blocks_escape(s, s_ego)
        if self._esc_latch or (stuck_long and not _cw_block):
            _dead, _dead_rows = self._dead_blockers(rf_objs, d_ego, now,
                                                    latched=self._esc_latch)
            if _dead and _clear and ov_side != 0.0:
                # ★★**합법 차로가 비어 있는데 교차로 잠금 때문에 못 가는 경우**(2026-09-09 코스 E
                #   (1286,850) 재주행, 200초 정지 후 미완주). 사고로 죽은 차 두 대가 내 차로를
                #   15m·21m 앞에서 막았고 **우측 차로(직진 가능, 여유 4.5m)는 통째로 비어 있었다.**
                #   그런데 23.4m 앞이 교차로라 `near_junction` -> `ov_hold` 로 추월기가 WAIT 에도
                #   못 들어가 200초를 서 있었다(신호는 그동안 적->녹을 여러 번 돌았다).
                #   사용자: "우측 차선이 직진도 되는 차선이라 거기로 가면 완전 합법인데 왜 차선을
                #   안 바꾸고 멈춰있니". 맞다 — 제22조가 막는 것은 **앞지르기**지 죽은 차를 피하는
                #   **진로변경**이 아니다(`_side_for` 의 같은 판단). 폭도 통로도 이미 합법이므로
                #   여기서는 탈출 폭을 새로 잡지 않고 **잠금만 푼다**.
                escape_dead = True
                _w0 = self._dead_escape_width(plan, _dead_rows, d_ego, ov_side)
                self._esc_want = ov_side * (_w0 if _w0 is not None else lane_w)
                if self._dead_log is None or now - self._dead_log > 30.0:
                    self._dead_log = now
                    print(f"[drive] ⚠ 사고현장 우회 — 죽은 것 {len(_dead_rows)}개 앞에서 "
                          f"{'좌' if ov_side > 0 else '우'} 차로로 진로변경(합법 폭 "
                          f"{((plan or {}).get('l' if ov_side > 0 else 'r') or 0.0):.2f}m, "
                          f"교차로 잠금 해제)", flush=True)
            elif _dead and not _clear:
                for c in sorted((+1.0, -1.0), key=lambda q: -self._phys_room(plan, q)):
                    _w = self._dead_escape_width(plan, _dead_rows, d_ego, c)
                    if _w is None:
                        continue
                    if not self.overtaker.set_escape(_w, self.HALF_WIDTH + self.DEAD_ESCAPE_HALF_PAD,
                                                     self.DEAD_PASS_CLEAR, self.DEAD_PASS_ROOM):
                        break
                    if self.overtaker.side_clear(ov_objs, s.speed, d_ego - base, c,
                                                 block_fx=max(r[0] for r in _dead_rows)):
                        ov_side, _clear, escape_dead = c, True, True
                        self._esc_want = c * _w
                        if self._dead_log is None or now - self._dead_log > 30.0:
                            self._dead_log = now
                            print(f"[drive] ⚠ 사고현장 우회 — 죽은 것 {len(_dead_rows)}개 너머 "
                                  f"{'좌' if c > 0 else '우'} {_w:.2f}m 로 비킨다(물리 폭 "
                                  f"{self._phys_room(plan, c):.2f}m, 합법 폭 "
                                  f"{((plan or {}).get('l' if c > 0 else 'r') or 0.0):.2f}m) — "
                                  f"평가상 중앙선 침범일 수 있음, 미완주보다 싸다", flush=True)
                        break
                    self.overtaker.clear_escape()
        if escape_dead:
            self._esc_latch = True
        else:
            self._esc_latch = False
            self._esc_want = None
        risky_relax = risky_relax or escape_dead
        if ov_side != 0.0:
            # ★`now` 를 넘겨야 방향 확정 지연(SIDE_HOLD)이 돈다 — 안 주면 벽시계로
            #   떨어져 오프라인 재현이 흔들린다.
            self.overtaker.set_side(ov_side, now)

        # ★차선 배치를 모르는 곳(교차로)에서는 앞지르기를 시작하지 않는다.
        #   도교법 제22조가 교차로 앞지르기를 금지하고, 실측으로도 여기서 나가면
        #   VTD가 정상차선으로 되돌려 **리스폰(감점)** 이 난다(2026-08-15, s≈175/210).
        #   합법적으로 비킬 데가 없어도(ov_side==0) 시작하지 않는다.
        #   오래 갇혀도 기본 모드에서는 이 금지를 해제하지 않는다.
        #   ★차로계획이 교차로 안까지 채워지게 된 뒤로는(plan_route 가 도로·차로를
        #     같이 넘겨준다) plan is None 만으로는 교차로를 못 걸러낸다. j=1 로 본다.
        #     같은 방향 차선변경(아래 nudge/사람 우회)은 여전히 허용된다 — 금지되는 건
        #     '앞지르기'지 '진로변경'이 아니다.
        # ★신호 정지선 거리 — 아래 추월기 게이트와 신호 판단이 **같은 값**을 쓴다.
        tl_stop_dist, tl_passed = self._tl_stop_dist(s) if self.sc else (None, False)
        # ★★**보고된 신호가 있을 때만** 그 정지선 거리를 넘긴다(2026-09-09 재주행 B 항목7 -6 재발).
        #   `_tl_stop_dist` 는 tl_id=0(신호 없음)에도 `_tl_stop_painted` 로 앞 60m 도색선 거리를
        #   돌려준다. 그걸 그대로 넘기면 `_tl_unseen_dist` 가 '그 선은 이미 보고된 신호의 선'으로
        #   읽어 **언제나 None** — 스윕 뒤 넣은 TL_UNSEEN 이 실주행에서 한 번도 안 걸렸다
        #   (B 전 구간 cap 0회, 167 을 또 9m 앞에서 36km/h 로 봐 정지선 1.7m 넘어 정지).
        tl_unseen = self._tl_unseen_dist(s_ego, self._reported_tl_dist(s, tl_stop_dist)) \
            if self._tl_lines else None
        if (s.tl_id, s.tl_state) != self._tl_prev:
            self._tl_prev, self._tl_change_t = (s.tl_id, s.tl_state), now
        # 녹색으로 바뀐 직후 — 앞차가 출발할 시간(GREEN_GRACE_S). 실측 2026-09-06 코스 E 신호114:
        # 녹색이 된 **그 프레임**에 정지선 안쪽 앞차를 추월 대상으로 봐 나갔다.
        green_fresh = (s.tl_state in (TL_GREEN, TL_GREEN_LEFT) and self._tl_change_t is not None
                       and now - self._tl_change_t < self.GREEN_GRACE_S)
        in_junction = bool(plan) and plan.get("j")
        # ★★교차로가 **코앞에 있으면 추월을 시작하지 않는다** [도교법 제22조].
        #   기존엔 '지금 교차로 안인가'만 봤다. 그런데 기동은 20~30m 를 쓰므로,
        #   교차로 직전에 시작하면 **나가 있는 채로 교차로에 들어간다.**
        #   실측 2026-08-26 코스 A: t=357.6 에 추월 시작(j=0) -> 8m 뒤 j=1 로 진입 ->
        #   t=366.2 에 **VTD 리스폰(5.5m·42°)**. 그 뒤 차가 물리적으로 못 움직여
        #   `vcmd=14.2 a=+2.00` 전력 가속에도 **152초간 0.00m**, 미완주로 끝났다.
        #   기동을 중간에 접는 것도 위험하므로, **애초에 시작을 막는 게 맞다.**
        junc_soon = self._junction_ahead(bi)
        near_junction = junc_soon is not None and junc_soon < self.JUNC_NO_PASS
        unknown = (bool(self.lane_plan) and (plan is None or in_junction or near_junction)) \
            or ov_side == 0.0
        # ★★추월 **대기조차** 하지 않는 곳(overtake.plan 의 hold 주석, 2026-09-06):
        #   ① 적·황색 앞에서 정지선 안쪽에 선 앞차 = 나와 같은 대기자다(코스 G 신호 173·198·211,
        #      WAIT 로 좌측 지시등 14초). 신호가 바뀌면 간다 — 녹색인데도 안 가면 그때 WAIT.
        #   ② 교차로 안·직전 — 앞지르기 금지(제22조, 코스 H 우회전 중 좌측 지시등).
        #      명시적 중앙선 탈출 실험 모드에서 오래 갇힌 때만 종전 동작을 복원한다.
        red_wait = ((s.tl_state in (TL_RED, TL_YELLOW) or green_fresh) and tl_stop_dist is not None
                    and any(sp < self.JAM_STOPPED_V and 0.0 < ds < tl_stop_dist + 5.0
                            and abs(d + d_ego) < self.JAM_LANE_HALF
                            for ds, d, sp, *_ in rf_objs))
        ov_hold = red_wait or ((in_junction or near_junction) and not risky_relax)
        # ★회전 대기 차량을 정지 장애물로 오인해 옆차로로 나갔다 돌아오지 않는다.
        #   60m 창은 추월 왕복 기동을 끝낼 거리까지 포함한다. 실효 도달거리는 그보다
        #   멀다 — `_turn_within` 은 회전 **정점**이 아니라 회전이 **시작되는 지점**까지를
        #   재므로, 창이 60m 여도 정점 기준 **83m 앞**부터 걸린다(실측 2026-09-08).
        #   그 덕에 '창 밖에서 시작해 회전 직전에 왕복'이 안 난다 — 정점 90m 앞에서
        #   시작한 추월은 회전까지 **63.1m** 남기고 복귀를 끝낸다.
        #   (`test_회전_게이트는_창_상수보다_멀리_닿는다` 가 그 여유를 못 박는다.)
        # ★★네 가지를 원안에서 고쳤다(①② 2026-09-07 · ③④ 2026-09-08, 실측 근거는 아래).
        #   ① **오래 갇히면 푼다.** 원안은 아예 안 풀었는데, 그러면 회전
        #      60m 안에 죽은 차가 하나 서면 **영영 못 나간다**. 재현: 20m 앞 정지차 ·
        #      우회전 39m 앞 → 300초를 돌려도 `WAIT` · 오프셋 0.00 그대로였다.
        #      그 60m 창이 경로에서 차지하는 비율이 작지 않다(수치는 상수 주석의
        #      재측정 표를 본다 — 2026-09-08 에 좌회전 창까지 같이 쟀다).
        #      게다가 **교차로 앞지르기 금지(제22조)는 법으로 더 센 금지인데도**
        #      `stuck_long` 이면 풀어 준다. 이것만 그보다 엄격할 이유가 없다.
        #      미완주(0점)가 앞지르기 감점보다 훨씬 비싸다.
        #   ② **`ov_hold` 로 막는다.** `allow_start=False` 만으로는 FSM 이 `WAIT` 에
        #      들어가고, `WAIT` 의 지시등은 추월할 쪽을 켠다(`set_side` 주석) —
        #      **우회전하려는 참에 좌측 깜빡이가 켜진다**(항목13). 재현: 두 번째 프레임부터
        #      계속 좌측. 추월기 신호가 주행기의 우회전 신호를 덮어써서 우측이 아예 안 나온다.
        #      `hold` 는 적신호 앞에서 같은 거짓 점등을 막으려고 이미 있는 문이다.
        #   ③ **좌·우를 가리지 않는다.** 좌회전 대기가 오히려 더 길다(상수 주석의 실측).
        #   ④ 해제 문턱은 앞차가 무엇이냐로 갈린다 — 신호를 기다리는 차를 20초에 추월하면
        #      이 게이트가 애초에 막으려던 그 동작이다(상수 주석의 실측 표).
        turn_ahead = self._turn_within(bi, self.OVERTAKE_TURN_GUARD) != 0
        # ★판정은 추월기가 쓰는 것과 **같은 차**를 봐야 한다. 여기서 따로 '앞차'를
        #   고르면 게이트가 A 를 보고 추월기는 B 를 막는 어긋남이 생긴다.
        #   ⚠️ `plan()` 이 이 프레임의 `was_moving` 을 갱신하기 **전**이라 한 프레임
        #      늦다. 문턱이 20초·45초라 무해하다.
        turn_gate_relax = self.JUNCTION_WAIT
        if turn_ahead and self.overtaker.blocker_was_moving(ov_objs, d_ego - base):
            turn_gate_relax = self.TURN_GUARD_RELAX_MOVED
        turn_gate_stuck = (self._stalled_since is not None
                           and now - self._stalled_since > turn_gate_relax)
        turn_soon = turn_ahead and not turn_gate_stuck
        ov_hold = ov_hold or turn_soon
        allow_overtake_start = (not turn_soon
                                and ((not unknown) or risky_relax))
        off_ov, turn_av, ov_state, _blk = self.overtaker.plan(
            s, ov_objs, now=now, allow_start=allow_overtake_start,
            d_ego=d_ego - base, hold=ov_hold)

        # ★서 있는 사람이 앞을 막고 있으면, **옆 차로가 통째로 있으면 그냥 차선을 바꾼다.**
        #   차로가 없을 때만 선다. (횡단 중인 사람은 아래 nudge 에서 제외 -> 무조건 정지)
        lane_w = (plan or {}).get("w", 3.3)
        #  ⚠️ 판정에 **경로 기준**(d + d_ego) 횡거리를 쓴다. ego 기준을 쓰면 우리가 비킨 만큼
        #     사람이 창 밖으로 나가 우회가 풀리고, 오프셋이 되돌아오면 다시 들어와서
        #     **오프셋이 3.3 -> 0.85 -> 3.3 으로 진동**한다. 그 골짜기에서 사람에 가까워졌다
        #     (2026-08-15 EV_PED 실측: 실여유 0.42m). 자기 행동으로 조건이 풀리면 안 된다.
        rf_ped = self._lc_hold_view(rf_objs, bi)     # 차로를 지키는 중이면 그 사람은 지금 차로 기준으로
        ped_ahead = [(row[0], row[1] + d_ego) for row in rf_ped
                     if self._rf_is_vru(row) and row[2] < 0.5]
        # ★★'막았다'는 **차로 안에서 지나갈 때 실여유가 모자랄 때**만이다(2026-09-11 v6 수정판).
        #   예전 문턱은 경로 중심에서 횡 2.5m 였다. 그 사람은 옆 차로 주차차 옆 **2.6~3.3m**
        #   에 서 있었는데(차로 안에서 지나가도 실여유 1.3m) 경계에서 걸려 **한 차로 옆으로
        #   차선을 바꾸기 시작했다**. 적신호로 줄이는 중이었고, 37m 앞을 6.4m/s 로 뛰어
        #   건너는 사람까지 겹쳐 오프셋이 1.30 -> 0.34 -> 3.14 로 오갔고 조향이 ±14° 로
        #   흔들렸다. 사용자: "정신을 못 차리네".
        #   행동층은 실여유 `ped_aside_clr`(0.4m) 이상이면 차로 안에서 서행 통과한다
        #   (PED_STILL_ASIDE). 두 층이 다른 자를 쓰면 이렇게 서로 다른 일을 한다 — 같은
        #   자에 추적 오차 여유만 더해 판정한다.
        ped_block = self._ped_blocks(rf_ped, d_ego)
        ped_side = self._side_for(plan, lane_w + self.HALF_WIDTH) if ped_block else 0.0
        #  래치: 한 번 비키기 시작하면 **그 사람을 완전히 지나갈 때까지** 유지한다.
        #        차선변경을 중간에 되돌리는 것 자체가 위험하다.
        if ped_block and ped_side != 0.0:
            self._ped_latch = True
        if self._ped_latch and not any(ds > self.PED_LATCH_CLEAR for ds, _ in ped_ahead):
            self._ped_latch = False                 # 다 지나갔다
        ped_bypass = self._ped_latch
        # ★사고현장 우회 **기동 중(PASS)** 에는 누워 있는 자전거·서 있는 사람 옆을 차선을 바꿔 지나는
        #   것이므로 행동층에 ped_bypass 로 알린다 — 안 그러면 YIELD_PED 가 앞범퍼 5m 에서 완전정지
        #   시켜 대향차로 한복판에 굳는다(2026-09-09 모의 accident_junction: off=+3.2 로 나가다
        #   (44.1,0.9) 에서 0km/h). ⚠️ WAIT 에서는 주면 안 된다 — 서행이 풀려 0.3m 기어가는 순간
        #   `_stalled_since` 가 리셋돼 20초 갇힘이 다시 0 이 되고, 20초마다 0.3m 씩 기는 순환이 된다
        #   (같은 날 모의 실측: t=32.7/53.6/74.5/95.4 에 WAIT 0.5초 -> 크리프 -> FOLLOW).

        # 장애물 비키기(차로 단위). 추월이 돌고 있으면 추월 오프셋을 쓴다.
        # ★★**추월기가 못 움직이는 사고현장은 우리가 직접 민다**(2026-09-10 코스 H (1462,929)).
        #   실측: "사고현장 우회 … 우 2.49m" 로그를 30초마다 찍으면서 **249초 동안 오프셋 0.00**.
        #   막은 것이 내 진로 한복판에 선 **사람**이라 `overtake._own_lane_block` 이 그걸
        #   추월 대상에서 빼고(사람은 앞지르기 대상이 아니다) FSM 이 FOLLOW 에서 안 나갔다.
        #   -> 추월기가 FOLLOW 인 동안에는 탈출 폭을 **nudge 로** 직접 준다. 뒤차 확인은 그대로.
        _hold_lat = (s.speed < self.CW_STOP_V and s.tl_state in (TL_RED, TL_YELLOW)
                     and tl_stop_dist is not None)
        _esc_force = self._esc_want if (escape_dead and ov_state == "FOLLOW") else None
        if _esc_force is not None:
            _hold_lat = False                       # 사고현장 탈출은 신호 정지 중에도 민다
        nudge = self._nudge_offset(rf_objs, d_ego - base, dt, plan, ped_bypass,
                                   ego_speed=s.speed, hold_lat=_hold_lat, force=_esc_force, bi=bi)
        # ★★**추월 복귀(RETURN) 중에는 nudge 를 쌓아 두지 않는다**(2026-09-11 코스 H TRV (1476,522)).
        #   nudge 는 FOLLOW 에서만 쓰이는데 계산은 매 프레임 돈다. 추월 중 옆에 있던 휠체어 몫으로 3.46 이
        #   쌓였다가 휠체어를 지나며 1.2m/s 로 줄던 중, 복귀가 끝나 FOLLOW 가 되자 **남은 2.27 이 그대로
        #   명령**이 됐다 — 차로에 거의 다 돌아온(+0.43) 차가 +20° 로 다시 옆 차로로 꺾었다.
        #   복귀하는 동안 0 으로 두면 FOLLOW 는 추월기가 준 0 에서 이어받는다(끊김 없음). 비킬 게 있으면
        #   거기서부터 다시 민다. 사람이 원래 차로에 남아 있으면 추월기가 애초에 복귀하지 않는다
        #   (`overtake._vru_in_return_path`).
        if ov_state == "RETURN":
            self._nudge = 0.0
            nudge = 0.0
        # ★차로를 지키는 동안(`_lc_hold`)은 그 사람 옆으로 차선을 바꾸는 게 아니다 — 사람 우회 래치를 끈다.
        #   안 끄면 사람을 지나 붙잡기가 풀리는 순간 래치(뒤 4m 까지 유지)가 7km/h 캡을 걸어
        #   18.6km/h 에서 a=-5 로 급감속한다(모의 `lc_hold_for_vru` x=117.5 실측).
        if self._lc_hold is not None:
            self._ped_latch = False
            ped_bypass = False
        esc_nudging = _esc_force is not None and abs(nudge) > 0.05
        nudging = ov_state == "FOLLOW" and abs(nudge) > 0.05
        if ov_state != "FOLLOW":
            off = base + off_ov
            # ★WAIT 에 들어가면 off_ov 가 0 이라 **그동안의 횡회피가 통째로 취소된다**.
            #   그러면 차로 중심으로 되돌아가는데 거기 막는 차가 서 있다 — 스스로
            #   들이받으러 간다. 실측 2026-08-17(공식 v3, EV_LEADBRAKE 동일):
            #     t=35.1 OVT:WAIT 물체 7.21m 앞(차로 중심), 내 위치 +2.45m, clr +1.26
            #     t=36.4 off=+0.00 로 무너짐,        내 위치 +1.98m, clr -0.73  <- 접촉
            #     이후 994초 정지(후진이 없다)
            #   nudge 는 `ov_state == "FOLLOW"` 일 때만 도니 그 사이에 아무도 안 벌린다.
            #   막는 차가 앞에 있는 동안은 **지금 벌려둔 만큼을 유지**한다.
            if (ov_state == "WAIT" and _blk is not None
                    and abs(self._off_prev) > abs(off)):
                off = self._off_prev
        elif nudging:
            off = base + nudge
        else:
            off = base
        fwd_off = offset_path(fwd, off)

        # 행동(도교법)
        if plan is not None and plan.get("lim"):
            self.beh.speed_limit = plan["lim"]      # 노면표시 기준(경로점별)
        elif self.sc:
            self.beh.speed_limit = self._plan_limit(s.x)
        # (tl_stop_dist · tl_passed 는 위 추월기 게이트에서 이미 구했다 — 같은 값)
        # ★기동 중이면 **아직 남은 횡이동**을 알려준다. 안전망이 '지금 자리'만 보고
        #   완전정지를 주면, 옆으로 다 빠지기 전에 굳어 영영 못 나간다(2026-08-26
        #   EV_COMBO_PASSPED: 목표 -6.34m 인데 -5.41m 에서 굳어 260초).
        ego_yaw = s.heading - self.rf.heading_at(i_ego)
        ego_yaw = (ego_yaw + math.pi) % (2 * math.pi) - math.pi
        # 무신호 좌회전에서 가로지르는 차를 기다릴 때 교차로 입구만 목표로 삼으면,
        # 그 앞의 횡단보도를 지나친 뒤 본체 위에 멈출 수 있다(코스 A t=620, 3.2초).
        # 보호구역 여부와 무관하게 가장 가까운 무신호 횡단보도의 안전 정지점을 함께
        # 넘긴다. Behavior는 실제 YIELD_CROSS가 필요한 경우에만 더 앞쪽 목표를 고른다.
        _yield_cw_key, _yield_cw = self._crosswalk_ahead(
            s, self.beh.speed_limit, s_ego, zone_only=False)
        yield_stop_dist = None
        if _yield_cw is not None:
            _yield_rest = _yield_cw[0] - _yield_cw[1]
            # 목표 부근 제어 오차 0.5m까지는 0으로 물어 그 자리에서 기다린다. 그보다
            # 지나쳤다면 횡단보도 위 급정지를 새로 만들지 않고 기존 교차로 규칙에 맡긴다.
            if _yield_rest >= -0.5:
                yield_stop_dist = max(0.0, _yield_rest)

        v_cmd, _off, turn_beh, reason = self.beh.plan(
            s, vp, tl_stop_dist=tl_stop_dist, lane_offset=off,
            lat_plan=off - d_ego, ego_yaw=ego_yaw,
            # ★RETURN 도 '우회 중'이다(2026-09-09) — 명령 오프셋은 0 이어도 차체는 아직 옆 차로에 있다.
            #   추월기가 차체 복귀까지 RETURN 을 유지하므로(overtake RETURN_DONE_D) 여기도 같이 본다.
            avoiding=(ov_state in ("PASS", "RETURN") or esc_nudging),
            rf_objs=self._lc_hold_view(rf_objs, bi),
            # 사고현장을 nudge 로 지나는 중이면 서 있는 사람 옆도 서행으로 지난다(안 그러면
            # YIELD_PED 가 앞범퍼 5m 에서 세워 그 자리에서 다시 굳는다).
            # ★차로를 **지키는** 중(`_lc_hold`)은 사람 옆으로 차선을 바꾸는 게 아니다 — 7km/h 캡을 안 건다.
            ped_bypass=((ped_bypass and nudging and self._lc_hold is None) or esc_nudging
                        or (self.overtaker.escaping and ov_state == "PASS")),
            route_turn=self._upcoming_turn(bi), tl_passed=tl_passed,
            junc_dist=junc_soon,                 # 위에서 계산한 값 재사용(같은 bi)
            yield_stop_dist=yield_stop_dist,
            rtor_point=self._rtor_point(s, s_ego),
            path_ahead=self._path_ahead(s, s_ego, s.speed),
            tl_unseen_dist=tl_unseen)

        # ★어린이보호구역의 무신호 횡단보도 = **보행자 유무와 관계없이 일시정지** [법 제27조⑦].
        #   한 번 서면 그 횡단보도는 통과시킨다(안 그러면 영원히 못 간다).
        #   ⚠️ 신호등 판단과 **별개**로 건다 — 신호 있는 횡단보도는 DB 에서 이미 빠져 있고,
        #      여기 걸리는 건 신호가 아예 없는 곳뿐이다.
        # ★★**횡단보도 위에 사람이 있으면 정지선 앞에 선다** [법 제27조①] (CW_PED_* 주석).
        #   보호구역이 아니어도, 신호가 있어도 마찬가지다 — 사람이 길 위에 있으면 못 간다.
        #   ⚠️ 그 사람이 CW_PED_MAX_S 넘게 **가만히** 서 있으면 교착이다(우리 때문에 멈춘
        #      것일 수 있다). 그때는 풀어 주고 기존 서행 통과(PED_STILL_ASIDE)에 맡긴다.
        cwp_key, cwp = self._crosswalk_ahead(s, self.beh.speed_limit, s_ego,
                                             zone_only=False, all_cw=True)
        #   ⚠️ **사고현장 우회 중에는 걸지 않는다**(2026-09-11 코스 H (1462,934)). 우회로 이미
        #      옆 차로에 반쯤 나가 있는데 여기서 세우면 그 자리에 굳고, VTD 가 경로이탈로 보고
        #      **리스폰**시켰다(사용자: "여기서 왜 리스폰함?"). 사람 보호는 YIELD_PED·CPA 가
        #      그대로 하고, 우회는 2.5m/s 이하 서행이다.
        _cwp_body = (self._cw_body_fwd(s, cwp_key)
                     if (cwp_key is not None and not self._esc_latch) else None)
        if cwp_key is not None and cwp[0] > 0.0 and _cwp_body is not None:
            _on = self._ped_on_crosswalk(rf_objs, _cwp_body, d_ego, plan=plan)
            if _on:
                # ⚠️ 12초는 **그 사람이 가만히 있기 시작한 때**부터 센다. 우리가 기다리기
                #    시작한 때부터 세면, 적신호에 서서 기다리는 동안 시계가 다 흘러 녹색이
                #    되는 순간 바로 지나가 버린다(2026-09-10 코스 H 실측).
                if all(_on):
                    t0 = self._cw_ped_wait.setdefault(cwp_key, now)
                else:
                    self._cw_ped_wait.pop(cwp_key, None)
                    t0 = now
                _dead = all(_on) and now - t0 > self.CW_PED_MAX_S
                if not _dead:
                    _tgt = self._cw_stop_fwd(s, cwp_key, _cwp_body)
                    _d_tgt = _tgt if _tgt is not None else cwp[0] - cwp[1]
                    v_cwp = self.beh._stop_target_speed(_d_tgt)
                    # ★★**사람이 횡단보도 위에 있는 동안은 다시 빨라지지 않는다**(2026-09-11 코스 A).
                    #   이 상한은 '정지선까지 감속 곡선'이라 선이 5m 남았으면 13km/h 를 허용한다.
                    #   그 사이 YIELD_PED(7.2 -> 0)가 한 프레임 빠지면 최소값이 이 곡선으로 뛰어
                    #   **6.6 -> 13.2km/h 로 재가속**했다가 다시 급제동한다. 실측 (986.9,253.6)·
                    #   (1104.8,365.5) 두 곳 다 그 꼴이었다. 사용자: "횡단보도 사람 보고도 안 멈추고
                    #   계속 움직여". 사람이 위에 있는 동안 명령 속도는 **내려가기만** 한다 —
                    #   어떤 규칙이든 한 번 0 을 냈으면 그 사람이 내려갈 때까지 0 이다.
                    #   가만히 선 사람의 교착(`_dead`)은 종전대로 풀어 준다.
                    v_hold = self._cwp_latch(cwp_key, v_cmd, v_cwp, _d_tgt)
                    if v_hold < v_cmd:
                        v_cmd, reason = v_hold, "CROSSWALK_PED"
                else:
                    self._cwp_hold.pop(cwp_key, None)
            else:
                self._cw_ped_wait.pop(cwp_key, None)
                self._cwp_hold.pop(cwp_key, None)

        cw_key, cw = self._crosswalk_ahead(s, self.beh.speed_limit, s_ego)
        if cw_key is not None:
            rest = cw[0] - cw[1]                   # 정지 목표점까지 남은 거리
            if s.speed < self.CW_STOP_V and rest <= self.CW_DONE_NEAR:
                self._cw_done.add(cw_key)          # **그 앞에서** 섰다 -> 통과 허용
                self._cw_done |= self._cw_shares_line(s, bi)
            else:
                v_cw = self.beh._stop_target_speed(rest)
                if v_cw < v_cmd:
                    v_cmd, reason = v_cw, "CROSSWALK_STOP"

        # ★교차로 꼬리물기 금지 [도교법 제25조⑤]. **빠져나올 자리가 없으면 안 들어간다.**
        #   실측 2026-08-25 코스 H: 녹색(tl163)에 진입했는데 5m 앞에 정지차가 있어
        #   **교차로 한복판에서 153초** 섰다. 교차 교통을 통째로 막았고, 교차로 안이라
        #   앞지르기도 금지(제22조)라 스스로 못 풀었다 — 들어가기 전에 물어봤어야 했다.
        #   ★밖에서 기다리면 추월 FSM 이 살아 있어서 스스로 풀 여지도 남는다.
        jspan = self._junction_span(bi)
        if (jspan is not None and jspan[0] is not None and jspan[1] is not None
                and jspan[0] > 0.5):
            j_in, j_out = jspan
            # ★★검사 구간은 **출구 근처만**이다. 교차로 전체를 보면 안 된다 —
            #   실측 2026-08-26 코스 E: 그 교차로가 **길이 49.6m** 라(H 는 18.1m)
            #   `j_in ~ j_out+여유` 로 잡으면 **56m 구간**이 되고, 주변교통 50대면
            #   그 안에 늘 누군가 서 있어 **영영 진입 못 한다**(294초 정지).
            #   법이 묻는 건 "출구 너머에 내가 설 자리가 있나"지 "56m 가 다 비었나"가 아니다.
            lo = j_out - self.JAM_LEN
            hi = j_out + self.JAM_LEN + self.JAM_MARGIN
            jam = None
            t_go = min(self.JAM_LOOK_S, max(j_out, 1.0) / max(s.speed, 2.0))
            for ds, d, sp, *_ in rf_objs:
                if abs(d + d_ego) > self.JAM_LANE_HALF:
                    continue                                 # 내 진로가 아니다
                ds_p = ds + sp * t_go            # 우리가 출구에 닿을 때쯤 그 차가 있을 자리
                if lo < ds_p < hi and (jam is None or ds_p < jam):
                    jam = ds_p
            if jam is not None:
                v_jam = self.beh._stop_target_speed(j_in - self.beh.front_overhang - 1.0)
                if v_jam < v_cmd:
                    v_cmd, reason = v_jam, "JUNCTION_JAM"

        # ★고아 정지선 일시정지 [시행규칙 별표6 노면표시 530]
        # ★★**교차로 안에서는 걸지 않는다.** 도색 정지선은 접근로마다 있어서, 회전
        #   중이면 **다른 접근로의 정지선**이 우리 앞에 잡힌다. 이미 적법하게 들어온
        #   교차로 한복판에 다시 서는 것은 [법 제32조] 위반이고 뒤차에게도 위험하다.
        #   실측 2026-08-30 코스 E (1464,909): 좌회전 화살표를 받고 들어가 회전하는
        #   도중에 `STOPLINE_STOP` 이 걸려 14.9 -> 0.8km/h 로 **또** 섰다.
        #     t=176.5 NOSIG_LEFT 출발 -> t=178.3~180.0 STOPLINE_STOP -> t=180.4 재출발
        #   사용자: "정지선에 가까이 가서 멈추지도 않고 혼자 멈췄다가".
        in_junc_now = bool(plan) and plan.get("j")
        sl_key, sl_fwd = (None, None) if in_junc_now else self._stopline_ahead(s)
        if sl_key is not None:
            rest = sl_fwd - self.SL_GAP
            if s.speed < self.SL_STOP_V and rest <= self.SL_DONE_NEAR:
                # ⚠️ 도색 정지선은 **차로마다 하나씩** 있다. 옆차로 것이 3m 옆에 같이
                #    잡혀서(실측 코스 A: 6곳이 9곳으로 보였다) 3m 가다 또 선다.
                #    같은 자리로 보고 함께 소진한다.
                for q in self.sl:
                    if math.hypot(q[0] - sl_key[0], q[1] - sl_key[1]) <= self.SL_SAME:
                        self._sl_done.add((round(q[0], 1), round(q[1], 1)))
                # ★정지선에서 선 것도 **그 앞 횡단보도의 정지**로 친다.
                #   실측: 고아 정지선(1097,350) -> 24m 뒤 교차로 안 횡단보도에서 재정지.
                self._cw_done |= self._cw_shares_line(s, bi)
            else:
                v_sl = self.beh._stop_target_speed(rest)
                if v_sl < v_cmd:
                    v_cmd, reason = v_sl, "STOPLINE_STOP"

        # 추월 중 서행 (오프셋을 주는 순간 장애물이 '내 차선 밖'이 되어 구속이 사라짐)
        if ov_state in ("WAIT", "PASS", "RETURN"):
            v_cmd = min(v_cmd, self.PASS_SPEED_CAP)
            if self.overtaker.escaping:
                v_cmd = min(v_cmd, self.DEAD_ESCAPE_V)      # 사고현장은 기어서 지난다
        elif nudging:
            v_cmd = min(v_cmd, self.PED_BYPASS_V if (ped_bypass and self._lc_hold is None)
                        else self.NUDGE_SPEED_CAP)

        # 무한정지 방지 (대회 5분 제한 — 오탐 하나로 완주 실패가 최악)
        #  ⚠️ 적신호 대기는 정상이라 타이머를 멈추지만, '보행자 정지 + 그 지점 신호가 주기적으로
        #     적색'인 조합에선 타이머가 매번 리셋돼 영원히 못 빠져나온다(2026-08-15 실측).
        #     그래서 '적신호와 무관한 총 정지시간'도 따로 세고, 신호주기(~18s)를 훨씬 넘으면 탈출.
        # ★'정상적인 대기'는 탈출 타이머를 멈춘다. 적신호가 그렇고, **비보호 좌회전에서
        #   대향차를 보내는 중**도 그렇다. 실측 2026-08-20 HL_FMA_LEFTONC: 양보 중에
        #   30초 탈출이 물려 0.4초·0.8km/h 로 교차로 쪽으로 기어갔다. 그때는 여유 15m 라
        #   무해했지만, 탈출 금지 판정(`_escape_blocked`)은 **앞만 보고 옆에서 오는 차는
        #   못 본다** — 차가 끊이지 않으면 교차로로 밀어넣는다.
        #   ⚠️ 대신 FROZEN_SEC(75초) 는 그대로 둔다. 영원히 못 가는 것도 실패다.
        # ★★적신호가 '정상 대기'인 것은 **신호가 진짜로 나를 막고 있을 때**뿐이다.
        #   내 차로에 **정지선보다 가까이 선 차**가 있으면 나를 막는 건 신호가 아니라 그 차다.
        #   그런데도 red_hold 로 치면 스톨 타이머가 매 적색마다 리셋돼 **우회 시도를
        #   영영 안 한다**(신호는 주기적으로 빨개지므로 영구 정지가 된다).
        #   실측 2026-08-30 코스 E (1164.6,-481.5) — 사용자: "앞에 정지한 차가 있고
        #   여유 공간도 널널한데 왜 차선을 안 바꿔??":
        #     앞 15.0m 에 정지차(경로횡 -0.10, v=0.0) · 정지선은 17.1m (tl 114)
        #     왼쪽에 같은 방향 한 차로가 통째로 비어 있고 중앙선은 점선(xl 7.26)
        #     그런데 tl 114 가 1(적)->5->2->1 로 돌 때마다 `_stalled_since` 가 지워져
        #     `stuck_long`(20초)이 영영 안 서고 **105초** 정지.
        #   ⚠️ 앞지르기 금지는 '앞지르기'지 **장애물 회피 진로변경**이 아니다(제22조).
        blocker_near = False
        if tl_stop_dist is not None:
            for ds, d, sp, *_ in rf_objs:
                if (sp < self.JAM_STOPPED_V and 0.0 < ds < tl_stop_dist
                        and abs(d + d_ego) < self.JAM_LANE_HALF):
                    blocker_near = True
                    break
        red_hold = ((s.tl_state == TL_RED and tl_stop_dist is not None
                     and not blocker_near)
                    or reason in self.HOLD_REASONS)
        # ★★적·황색에 정지선 안쫽 앞차 뒤에 서 있는 동안은 스톨 타이머를 **멈춘다**(리셋도 진행도
        #   아님, 2026-09-06). 예전엔 진행시켰다 — 그래서 코스 E 신호114 에서 적색 17초 대기가
        #   '갇힘 20초'를 채워 녹색이 되자 **절박 모드**로 오른쭉(여유 1.36m = 인도)으로 나가
        #   리스폰. 사용자: "아무리 앞차가 멈춰도 인도로 추월은 아니지... 신호 기다리는 건데".
        #   리셋하지 않는 이유는 그대로다(2026-08-30, 신호 앞 **고장차** 105초) — 녹색마다
        #   누적돼 결국 우회 시도가 열린다.
        red_wait_blocked = (s.tl_state in (TL_RED, TL_YELLOW) and tl_stop_dist is not None
                            and blocker_near)
        if s.speed < 0.3 and not goal_reached:
            if self._stopped_since is None:
                self._stopped_since = now
            # ⚠️ '켜지 않는다'로는 부족하다 — 정상 대기 **전에** 이미 켜져 있으면 그대로
            #    물린다. 실측 2026-08-20: 녹색에서 화살표를 기다리는(WAIT_LEFT_ARROW)
            #    8초 동안 타이머가 켜지고, 이어진 적신호·양보 내내 살아남아 30초에 물렸다.
            #    정상 대기 중에는 **되돌린다.** 영원히 못 가는 건 FROZEN_SEC(75초)가 잡는다.
            if red_hold:
                self._stalled_since = None
            elif red_wait_blocked:
                if self._stalled_since is not None:
                    self._stalled_since += dt                  # 멈춤(위 주석)
            elif self._stalled_since is None:
                self._stalled_since = now
        else:
            self._stalled_since = self._stopped_since = None
        # ★★동결 판정은 **속도가 아니라 변위**로 한다.
        #   `_stopped_since` 는 한 프레임이라도 0.3m/s 를 넘으면 리셋된다. 정체에서 차가
        #   조금씩 꿈틀대면 75초 감시가 **영영 안 걸린다** — 실측 2026-08-25 코스 E:
        #   150초에 0.41m 밖에 안 움직였는데 `[교착]` 로그가 한 번도 안 찍혔다.
        #   "제자리에 있다"는 건 속도의 순간값이 아니라 **얼마나 안 갔나**로 재는 게 맞다.
        if (self._frozen_ref is None
                or math.hypot(s.x - self._frozen_ref[0], s.y - self._frozen_ref[1])
                > self.FROZEN_MOVE):
            self._frozen_ref = (s.x, s.y, now)
        frozen_disp = (not goal_reached
                       and now - self._frozen_ref[2] > self.FROZEN_SEC)
        if not goal_reached and s.speed < 0.3:
            stalled = self._stalled_since is not None and now - self._stalled_since > self.STALL_SEC
            # ⚠️ **탈출 판정에는 변위 기준을 쓰지 않는다.** 변위로 바꾸면 긴 적신호
            #    (75초 초과)에서도 동결로 잡혀 **적신호에 교차로로 기어들어간다.**
            #    변위 기준은 **진단 로그에만** 쓴다 — 관측은 넓게, 행동은 보수적으로.
            frozen = self._stopped_since is not None and now - self._stopped_since > self.FROZEN_SEC
            # ⚠️ 앞을 막고 있는 게 있으면 절대 전진하지 않는다. 탈출용 서행이라도 기어가면
            #    결국 닿는다(2026-08-15 실측: 보행자 앞 2.02m -> 145초에 걸쳐 -0.39m).
            if (stalled or frozen) and not self._escape_blocked(rf_objs):
                v_cmd = max(v_cmd, self.STALL_V)
                reason = "STALL_ESCAPE"
            elif frozen or frozen_disp:
                # ★★탈출이 **막힌 채로** 얼어붙은 상태 = 스스로 못 푸는 교착이다.
                #   실측 2026-08-25 코스 G(주변교통 50대): t=41s 에 정차 차량의 뒷모서리에
                #   옆구리가 물려(clr -0.40m) 멈춘 뒤 **503초 동안 이동량 0.00m**.
                #   그동안 뒤로 차가 쌓여 obj 13->30, t=240 이후로는 주변에 움직이는 것이
                #   하나도 없었다 — 우리가 정체를 만들었다.
                #   ⚠️ 직진 서행은 여기서 **틀린 답**이다(그 차의 앞끝이 범퍼보다 3.1m 앞이라
                #      곧장 기면 옆구리를 3m 더 긁는다). 제대로 풀려면 반대쪽으로 조향하며
                #      빠져나와야 하는데, 그건 예전에 되돌린 NARROW_CREEP 계열이다.
                #   지금은 **조용히 죽지 않게** 한다 — 30초마다 크게 남긴다.
                if (self._frozen_log is None
                        or now - self._frozen_log > 30.0):
                    self._frozen_log = now
                    blk = min(((abs(d) - self.HALF_WIDTH - owid / 2.0, ds, d, owid)
                               for ds, d, _sp, _ol, owid, *_ in rf_objs
                               if -1.0 < ds < self.ESCAPE_WATCH), default=None)
                    print(f"[교착] {now - self._frozen_ref[2]:.0f}초째 못 움직임 "
                          f"({s.x:.1f},{s.y:.1f}) reason={reason} "
                          f"막은 것={blk}", flush=True)

        # 자차 회전율(평활). 회전 지시등 래치가 '회전이 끝났나'를 이걸로 본다.
        if self._prev_hd is not None and dt > 1e-3:
            raw = math.degrees((s.heading - self._prev_hd + math.pi)
                               % (2 * math.pi) - math.pi) / dt
            self._yaw_rate = self._yaw_rate * 0.7 + raw * 0.3
        self._prev_hd = s.heading

        # ★경로에 박힌 차선변경 — 목표 차로 뒤차에 양보 [법 제19조③] (위 LC_REAR_* 주석).
        #   추월기가 도는 중이면 그쪽이 이미 목표차선 후방을 보므로 건드리지 않는다.
        if ov_state == "FOLLOW" and not nudging:
            _lc_v = self._lc_rear_yield(plan, rf_objs, d_ego, s.speed)
            if _lc_v is not None and _lc_v < v_cmd:
                v_cmd = _lc_v
                if reason in ("LANE_KEEP", "TURN_LANE"):
                    reason = "LC_REAR_YIELD"

        # ★방향지시등 — 추월기가 켜는 건 그대로 두고, **그 외의 모든 횡이동**(지정차로 진입,
        #   장애물 회피 차로변경, 복귀)과 **회전**에도 켠다. 안 켜고 차로를 바꾸면 진로변경 위반.
        self._beh_left_age = (self._beh_left_age + max(dt, 0.0)) if turn_beh == TS_LEFT else 0.0
        turn = self._final_turn(turn_av, turn_beh, s_remain)
        self._pend_sig = self._pending_lat_signal(off, d_ego, ov_state, plan)
        if turn == 0:
            d_off = (off - self._off_prev) / max(dt, 1e-3)
            # ★★차로를 **지키는** 중(`_lc_hold`)의 오프셋 변화는 차선변경이 아니다(2026-09-11 코스 H TRV).
            #   경로가 옆 차로로 옮겨 가는 만큼 오프셋을 되돌려 주는 것이라, 차는 제 차로에 그대로 있다.
            #   이걸 d_off 로 보면 좌측 지시등이 켜졌다 꺼졌다 한다(경로점이 바뀔 때마다 목표가 계단으로
            #   올라간다). 사용자: "왼쪽 깜빡이는 왜 계속 켰다껐다하면서 고민이여". 그동안은 아래 경로
            #   차선변경(sig)만 켠다 — 사람을 지나면 실제로 그쪽으로 옮긴다.
            if abs(d_off) > self.SIGNAL_RATE and self._lc_hold is None:
                turn = TS_LEFT if d_off > 0 else TS_RIGHT
            elif self._pend_sig != 0:
                # ★★**명령은 벌써 끝났는데 차체가 아직 옮기는 중**이면 그쪽을 켠다(2026-09-11 v6).
                #   위 d_off 는 '명령이 변하는 중'만 본다. 추월 복귀가 적신호에 **선 채로** 끝나
                #   (명령 3.10 -> 0 을 0.3초에, RETURN 은 15초 뒤 시간초과로 FOLLOW) 녹색에 차가
                #   실제로 3.1m 를 돌아올 때는 명령이 이미 0 이라 **지시등 없이** 차로를 옮겼다.
                #   사용자: "아무리 급박하게 차선변경해도 깜빡이는 켜야지".
                turn = TS_LEFT if self._pend_sig > 0 else TS_RIGHT
            else:
                # ★경로 자체에 박힌 차선변경(주최측 차량이 실제로 옮긴 구간)도 켠다.
                #   그 구간은 우리가 오프셋을 주지 않고 선만 따라가므로 위 d_off 로는 안 잡힌다
                #   (실측 2026-08-15: s≈95~125 에서 한 차로 왼쪽으로 옮기는데 깜빡이 없었다).
                # ★차로계획의 `sig` 를 그대로 쓴다. 생성 단계에서 이미 법 30m·대회 3초
                #   선행을 깔아 두므로(build_lane_plan.sig_lead_for) 런타임에서 다시
                #   늘릴 필요가 없다 — 계획기 차선변경 35곳 전부 지시등이 있는 걸 확인했다.
                rsig = (plan or {}).get("sig", 0)
                up = self._signal_turn(bi, s.speed)   # 점등 선행 = max(30m, 3초×속도)
                # ★★**회전이 끝날 때까지 켠다** [시행령 별표2: "그 행위가 끝날 때까지"].
                #   `_upcoming_turn` 은 35m 앞을 보므로 **회전에 깊이 들어가면 남은 각도가
                #   문턱 아래로 떨어져 0 이 된다** — 핸들은 아직 꺾여 있는데 꺼진다.
                #   실측 2026-08-28 코스 A: 회전 12개 중 **11개가 1.2~2.3초 일찍** 꺼졌다
                #   (예 (1090,0) -97.2° 회전: 끝 t=266.8 인데 264.6 에 소등).
                #   사용자가 화면에서 먼저 봤다: "steer -24.4° 인데 sig 가 꺼져 있다".
                #   -> 한 번 켜지면 **자차 회전율이 잦아들 때까지** 방향을 물고 있는다.
                #   ⚠️ 경로 곡률로 풀면 부족하다 — **경로는 이미 직선인데 차가 아직**
                #      돌고 있다(전방주시 지연). 실측: 곡률 기준은 12개 중 11개가
                #      여전히 0.8~1.4초 일찍 꺼졌다. 차 자신의 회전율로 봐야 맞다.
                #   ⚠️ 래치는 `_upcoming_turn`(=교차로 회전)이 켠 경우만이다. 교차로가 없는
                #      완만한 커브에서는 애초에 안 켜지므로 여기 걸릴 일이 없다 —
                #      "커브에서 깜빡이 켠다"는 지적을 다시 만들지 않으려는 조건이다.
                if up:
                    self._turn_latch = up
                elif self._turn_latch and abs(self._yaw_rate) <= self.TURN_SIG_HOLD_DPS:
                    self._turn_latch = 0
                # ★★**실제 회전이 차선변경 예고를 이긴다.** 차로계획의 sig 는 교차로를
                #   넘어서까지 켜지므로(build_lane_plan 주석), 교차로를 도는 동안
                #   회전 방향과 반대 등이 켜질 수 있다. 몸이 도는 방향과 등이 다른 게
                #   제일 나쁘다 — 회전이 끝나면 자연히 sig 로 넘어간다.
                t_dir = up or self._turn_latch or rsig
                if t_dir:
                    turn = TS_LEFT if t_dir > 0 else TS_RIGHT
        self._off_prev = off

        if ov_state != "FOLLOW" and reason != "STALL_ESCAPE":
            reason = f"OVT:{ov_state}"
        elif nudging:
            # 다른 이유(적신호 접근 등)와 겹쳐도 비키는 중인 게 보이도록 표시한다.
            tag = "PED_LANECHANGE" if ped_bypass else "NUDGE"
            reason = tag if reason == "LANE_KEEP" else reason + "+N"
        elif self._po_on and reason == "LANE_KEEP":
            reason = "GOAL_PULLOVER"
        elif abs(base) > 0.1 and reason == "LANE_KEEP":
            reason = "TURN_LANE"

        # ★★급커브에서 전방주시를 줄인다 — **코너컷으로 차선을 밟고 있었다.**
        #   실측 2026-08-27 공식 WP9: 지시 오프셋 0 인데 경로에서 **최대 2.70m** 벗어나고
        #   `|d_ego| > 0.5m` 가 프레임의 **8.1%**. 차로폭 2.40m 짜리 굽은 구간에서
        #   0.8m 벗어나 **차선을 5.2초 물었다**(차폭 1.886m 라 중앙에 놓아도 좌우 0.26m).
        #   v1~v7 은 같은 조건에서 최대 0.22m · 0.0% 로 멀쩡하다 — 문제는 추종 자체가
        #   아니라 **급커브**다(WP9 만 10m 방위변화가 20°를 넘는 구간이 11.2%,
        #   v1·v7 은 최대 4.8° 라 이 게이트에 **원리상 안 걸린다**).
        #   ⚠️ 값은 새로 만들지 않고 추월 때 쓰는 2.2m 를 그대로 쓴다 — 이미 검증된 값이다.
        #   ★대회 코스는 WP9 모양일 가능성이 크다 — [강의 103:02] "좌회전, 우회전해야
        #     되는 구간에 (경유지를) 하나씩 찍어드릴 거예요".
        if ov_state in ("PASS", "RETURN") or nudging:
            self.pp.lmin = self.LOOKAHEAD_PASS
        elif self._local_curve(bi) > self.TURN_CURVE_DEG:
            self.pp.lmin = self.LOOKAHEAD_TURN
        else:
            self.pp.lmin = self.LOOKAHEAD_NORMAL
        steer = self.pp.steer(s.x, s.y, s.heading, s.speed, fwd_off)
        # ★속도 추정 지연 보상(2026-09-06): 0.30초 창 이동거리 추정은 가속 중 실제보다 낮게 읽힌다.
        #   실측 코스 E t=78: 15->48 가속 끝에 추정 48.7 / 채점기(참) 52 -> 항목1 -3.
        #   직전 가속 명령 × V_LAG_S 를 더해 제어기가 미리 놓게 한다. 감속 중엔 보상하지 않는다.
        accel = self.lon.accel(v_cmd, s.speed + max(0.0, self._a_prev) * self.V_LAG_S, dt)
        self._a_prev = accel
        self._sig_dir_age = (self._sig_dir_age + max(dt, 0.0)) if (turn != 0 and turn == self._turn_prev) else (max(dt, 0.0) if turn != 0 else 0.0)
        self._turn_prev = turn

        return Command(steer=steer, accel=accel, turn=turn, v_cmd=v_cmd, reason=reason,
                       cap_by=getattr(self.beh, "cap_by", "SPEED_LIMIT"),
                       goal_reached=goal_reached, lane_offset=off, ov_state=ov_state,
                       rf_objs=rf_objs, d_ego=d_ego, base=base, nudge=nudge)
