"""
도로교통법 준수 행동 계획 (감점 항목 대응).

우선순위 arbiter로 목표속도(v_cmd)/차선오프셋/방향지시등을 결정한다.
  긴급정지(임박충돌) > 보행자 양보 > 적신호 정지 > 앞차 ACC(안전거리) > 장애물 정지 > 차선유지
인지는 9910 DataPacket의 objects[30] GT를 그대로 사용(라이다/YOLO 불필요).

⚠️ 신호등 정지선 좌표는 패킷에 없음 -> 경로(라우트)/xodr에서 계산한 tl_stop_dist를 넘겨줄 것.
   미제공 시 적신호는 보수적 감속(RED_SLOW)만 수행.
⚠️ 객체 '종류' 필드 없음 -> 원본 치수 기반 `vtd_io.is_vru()`로 취약대상을 추정.
   org 모듈이 종류를 제공하면 그 값으로 교체.
"""
import math

from run_logger import EGO_CX, oriented_gap
from vtd_io import (TL_RED, TL_YELLOW, TL_GREEN, TL_LEFT, TL_GREEN_LEFT, TL_UNSET,
                    TL_FLASH, TS_OFF, is_vru, VRU_MAX_SPEED)


def to_ego_frame(ego, o):
    """월드 객체 -> 자차 좌표(전방 x+, 좌 y+)."""
    dx, dy = o.x - ego.x, o.y - ego.y
    fx = dx * math.cos(-ego.heading) - dy * math.sin(-ego.heading)
    fy = dx * math.sin(-ego.heading) + dy * math.cos(-ego.heading)
    return fx, fy


class Behavior:
    def __init__(self, speed_limit=8.33, react=0.8, lane_half=1.75,
                 comfort_dec=3.0, plan_dec=1.5, follow_gap=10.0, scan_range=90.0,
                 stop_margin=5.4, front_overhang=3.81, rtor=True, rtor_v=4.0,
                 rt_approach=30.0, rt_stop_v=0.3,
                 flash_stop_v=0.3, flash_near=2.0):
        self.speed_limit = speed_limit      # m/s (8.33=30km/h)
        self.react = react                  # 반응시간 s
        self.lane_half = lane_half          # 자차선 반폭 m
        self.a = comfort_dec                # 안전거리 계산용
        self.a_plan = plan_dec              # 정지계획 감속(작게=일찍 제동)
        self.follow_gap = follow_gap
        self.scan = scan_range
        # ★정지선 여유는 '뒷축 중심' 기준이다(공식 egoReferencePoint=rear_axle_center).
        #   아이오닉6는 뒷축→앞범퍼 3.81m라, 여유 4m면 **앞범퍼가 정지선에 걸친다**(실측 캡처 확인).
        #   실제 여유 = stop_margin - front_overhang.
        # ★★6.0 이었다. **대회 채점 기준이 2m 다** [대회 안내문 2026-08-27 · 평가항목 7·9]:
        #     "범퍼 기준 정지선 2.0m 미만에서 0.5초 이상 정지(정상) /
        #      정지 위치가 정지선에서 **2m 이상: 경미(-3)** / 정지선 통과·무정차: 중대(-6)"
        #   6.0 이면 범퍼~정지선 = 6.0-3.81 = **2.19m** 로 0.19m 차이로 경미다.
        #   실측 2026-08-30 코스 E 전 구간(신호 정지 10회): 2.18~2.31m, **10회 전부 초과**.
        #   정지 정확도 자체는 ±0.07m 로 훌륭하다 — 목표만 당기면 된다.
        #   5.0 -> 범퍼 1.19m. 2m 안이면서 선을 넘지도 않는다(넘으면 중대).
        #   ⚠️ 더 줄이지 않는다. 정지선 좌표는 우리가 xodr 에서 계산한 값이라 오차가
        #      있고, 넘는 쪽(중대 -6)이 못 미치는 쪽(경미 -3)보다 두 배 비싸다.
        self.stop_margin = stop_margin      # 뒷축(기록 좌표)이 정지선에서 이만큼 앞에 서게 한다 [m]
        # ★5.0 -> 5.4 (2026-09-06). 실주행 129회 정지에서 뒷축~도색선 중간값 5.02m(IQR 4.91~5.08)로
        #   제어는 목표를 정확히 맞춘다. 문제는 **기준점 가정**이다: 좌표가 뒷축이고 앞범퍼가 3.81m
        #   앞이면 여유 1.2m 지만, 좌표가 그보다 1m 뒤(뒷범퍼 기준 4.85)라면 여유 0.17m = 선 위다.
        #   사용자가 화면으로 본 것은 "계속 정지선을 밟는다". 어느 쪽이든 안전하게 0.4m 물린다 —
        #   3.81 가정 1.6m(심판 2m 창 안) / 4.85 가정 0.55m(선 앞). 기준점은 실차 측면 뷰로 확정할 것.
        # ★우회전 전 일시정지(도교법 제25조). 신호가 녹색이어도 한 번 서고 간다 —
        #   서지 않으면 횡단보도 보행자를 확인할 시간이 없고, 그건 통과와 다름없다.
        #   ⚠️ '적색에서 일시정지 후 우회전'까지는 하지 않는다. 법적으로는 가능하지만
        #      채점기가 '적신호 통과'로 볼 위험이 있어, 적색이면 녹색까지 기다린다.
        self.rt_approach = rt_approach      # 이 거리 안으로 들어오면 정지 대상
        self.rt_min_arm = 0.5               # 정지점이 이보다 가까우면 '교차로 안' — 무장 안 함[m]
        self.rt_stop_v = rt_stop_v          # 이 아래로 떨어지면 '섰다'고 인정
        self._rt_armed = False              # 지금 우회전 접근 중인가
        self._rt_done = False               # 이번 우회전에서 이미 섰나
        # ★적신호 우회전 [시행규칙 별표2 '적색의 등화' 단서]
        #   > 차마는 정지선, 횡단보도 및 교차로의 직전에서 정지하여야 한다. **다만**
        #   > 신호에 따라 진행하는 다른 차마의 교통을 방해하지 아니하고 **우회전할 수 있다.**
        #   전엔 '채점기가 적신호 통과로 볼 위험'이 있어 녹색까지 기다렸다. 그 대가를
        #   실측해 보니 **코스 B 34.0초 · 코스 E 28.9초**(전체 459s/755s)였다 —
        #   5분 제한이 사실이라면 감당 못 할 크기다. 2026-08-25 사용자 판단으로 켠다.
        #   ⚠️ 우회전 삼색등이 적색이면 금지인데, **9910 프로토콜에는 우회전 화살표
        #      상태가 아예 없다**(UNSET/RED/YELLOW/GREEN/LEFT/GREEN_LEFT/FLASH 7종).
        #      즉 이 인터페이스에서는 그 경우가 표현되지 않는다.
        #   끄려면 `rtor=False` 한 줄. 세 조건을 다 만족할 때만 간다:
        #     ① 이미 일시정지했다(`_rt_done`) ② 보행자 캡이 따로 걸린다(_cap 최솟값)
        #     ③ 신호 따라 오는 차가 없다(`_oncoming_clear`, CPA)
        self.rtor = rtor
        self.rtor_v = rtor_v
        self.rt_green_v = 4.2               # 녹색 우회전은 서지 않고 이 속도로 돈다(15km/h) — 아래 2026-09-06
        # ★우회전 양보 판정은 **좌회전과 값이 달라야 한다.** 좌회전은 교차로 한복판을
        #   8초 점유하지만 우회전은 3초면 끝나고, 들어가는 자리도 바로 옆 차로 하나다.
        #   좌회전 값(9초·25m)을 그대로 쓰면 실주행에서 **거의 항상 막힌다** —
        #   실측 2026-08-25: 양보 A 29.9s·H 18.0s 인데 통과는 A 3.1s·H 0s 였다.
        #   막은 것들을 열어 보니 **뒤차와 왼쪽으로 우리를 지나쳐 간 차**였다
        #   (코스 H t=345.4: 왼쪽 3.1m 의 1.4m/s 짜리 차 하나에 18초를 섰다).
        #   그런 차는 우리 우회전에 방해받지 않는다.
        self.rtor_gap = 5.0              # 우회전 노출시간[s] (정지->가속->합류)
        self.rtor_radius = 5.0           # 합류 지점 이 안으로 들어오면 방해[m]
        self._rtor_go = False               # 적신호 우회전을 시작했다(회전 중 재정지 방지)
        self._red_clear_tl = None           # 적색인데 앞범퍼가 이미 정지선을 넘어 '빠져나가는 중'인 신호
        # ★적신호 우회전은 **정지선 직전에서 선 것**만 인정한다. 단서의 문언이 그렇다
        #   ("정지선, 횡단보도 및 교차로의 직전에서 정지하여야 한다. 다만 ...").
        #   실측 2026-08-25 코스 A tl130: 정체 뒤 **43.7m** 지점에서 완전정지한 걸
        #   '일시정지 완료'로 치고 정지선은 14.8km/h 로 통과했다 — 그건 그냥 신호위반이다
        #   (회전 자체는 -98.4° 로 멀쩡했다. 문제는 어디서 섰느냐다).
        #   우회전 일시정지(제25조)는 접근 어디서 서도 목적을 이루지만, 적신호 통과는 아니다.
        self.rtor_bumper_gap_max = 2.0       # 앞범퍼~정지선 유효 구간의 배타적 상한[m]
        # ★★**선을 조금 넘어 선 것도 인정한다**(2026-09-11 코스 H 신호155). 녹색에 우회전하러
        #   들어가다 앞차가 회전 중 멈춰 JUNCTION_JAM 으로 **앞범퍼가 선을 0.73m 넘은 자리**에 섰고,
        #   그대로 황->적. 일시정지는 끝났는데(`_rt_done`) 적신호 우회전 조건이 '선 **전** 0~2m' 라
        #   적색 정지를 한 번도 못 셌다 — 앞차가 빠진 뒤 녹색까지 **15초**를 더 섰다.
        #   사용자: "왜 우회전때 바로 안 함?? 기다리다가 신호가 바뀌면 멈추는 듯".
        #   ⚠️ **적색이 켜질 때 이미 선을 넘어 있던 경우만**이다(`_rtor_pos_ok`). 적색에 선을 넘어
        #      선 것(과주행)은 예전대로 안 된다(09-08 규칙·테스트 그대로) — 채점기도 그건 '적색에 다가가
        #      정지 없이 넘었다'로 본다. 녹색에 이미 넘어 있던 선은 적색 중에 '다가간' 적이 없다.
        #   43.7m 뒤 정체에서 선 것(08-25 A tl130)은 여전히 안 된다.
        self.rtor_past_max = 2.0             # 적색 전부터 선을 넘어 있었으면 이 안의 정지도 인정[m]
        self._red_onset_dline = {}           # id -> 적색이 켜진 순간 앞범퍼~정지선[m](음수 = 이미 넘음)
        self.red_clear_pad = 0.3             # 앞범퍼가 정지선을 이만큼 넘었으면 '이미 들어갔다'[m]
        self._rt_done_bumper_gap = None      # 그 정지 때 앞범퍼~정지선 거리[m]
        self._rt_tl = None                  # 그 일시정지·적신호우회전 허가가 **어느 신호등** 것인가
        self._rt_done_red = False           # 그 정지가 **적색 중**이었나 — 적신호 우회전(RTOR) 인정 조건
        self._rt_green_tl = None            # 녹색으로 서행 통과한 우회전의 신호 id(그 회전에선 다시 무장 안 함)
        self._yellow_stop_tl = None         # 황색에 '서기로 한' 신호 id — 반응시간 없이 재판정(래치)
        self._rt_red_still = None           # 적색으로 바뀐 뒤 다시 서 있는 시각(황색에 선 경우)
        self._rt_still_since = None         # 우회전 일시정지에서 '완전히 선' 시각
        self._left_go = False               # 좌회전 화살표를 받고 들어갔다
        self._left_tl = None                # 그 화살표가 어느 신호등 것인가
        # ★점멸 신호(적색 점멸)용 1회 정지 기억. 신호 id 단위.
        self.flash_stop_v = flash_stop_v
        # ★★9.0 이었다. 적색점멸도 **범퍼 기준 2.0m 미만 · 0.5초 이상** 이다
        #   [안내문 평가항목 9·공식 답변]: "범퍼 기준 정지선 2.0m 미만에서 0.5초 이상 1회 이상
        #   정지 필요 / 무정차 통과 시 중대(-6)". 9m 에서 선 것을 인정하면
        #   **정지선에서 9m 뒤에 서고 그냥 통과**하는 그림이 된다.
        self.flash_near = flash_near        # 앞범퍼~정지선 [0, flash_near)에서만 인정[m]
        # ★★**심판의 문턱보다 엄하게 잡는다.** 안내문은 "1km/h 이하 0.5초 이상" 인데
        #   우리 문턱을 거기 딱 맞추면 못 넘긴다 — 실측 2026-08-30 코스 E:
        #     신호 117(적색점멸)  1km/h 이하 **0.32초**   -> 항목 9 중대 -6
        #     신호 200(적색 우회전) 1km/h 이하 **0.40초**  -> 항목 7 중대 -6
        #   왜 짧았나: ① 우리 판정 문턱이 0.3m/s(=1.08km/h)라 심판의 1km/h 창보다
        #   **넓다** — 0.5초를 세는 동안 일부는 1.0~1.08km/h 구간이다. ② 우리 `speed` 는
        #   0.30초 창 이동거리라 실제보다 **늦게 0 에 닿는다.**
        #   그래서 속도는 더 낮게, 시간은 더 길게 잡는다. 정지 0.3초 더 서는 비용은
        #   판당 1초 남짓이고 중대 감점은 -6 이다.
        self.stop_hold_v = 0.15             # '완전히 섰다'로 볼 속도[m/s] = 0.54km/h
        # ★VTD 가 신호를 보고하기 시작하는 **최소** 전방거리(앞범퍼 기준) — 실측 6코스 로그(2026-09-09):
        #   대부분 60~300m 앞에서 보고되지만 신호 130·167 은 **9m**, 100 은 12m, 213 은 14m 다.
        #   그 앞까지 모르는 채 달리면 못 선다: 코스 B 신호167 을 37km/h 로 9m 앞에서 적색으로
        #   처음 봐 1.9m 지나쳤다(항목7 -6). D 는 같은 신호를 13km/h 로 와서 무사했다.
        self.tl_report_d = 9.0
        self.stop_hold_s = 0.8              # 그 상태를 이만큼 유지해야 인정[s]
        self.ped_leave_pad = 2.0            # 멀어지는 사람은 내 차로 반폭 + 이만큼 밖이어야 '갔다'[m]
        self.yellow_go_min_v = 2.0          # 이보다 느리면 황색 통과(딜레마존) 판정을 안 한다[m/s] = 7.2km/h
        self.flash_hold = 0.5               # (구버전 호환. 실제로는 stop_hold_s 를 쓴다)[s]
        self._flash_since = {}              # id -> 그 신호 앞에 멈춘 시각
        self._flash_done = set()
        self.front_overhang = front_overhang
        self.a_max = 4.5                 # 황색 정지가능 판정용 최대 감속
        self.unknown_slow_dist = 45.0    # 상태불명 신호에 이 거리부터 감속
        self.unknown_slow_v = 4.0        # 상태불명 신호 통과 속도(≈14km/h)
        self.obstacle_gap = 15.0         # 정지 장애물 앞 정지 간격(정지 후 추월할 횡이동 거리 확보)
        self.head_on_margin = 1.5        # 내 속도보다 이만큼 빠르게 좁혀오면 정면 접근[m/s]
        self._prev_fx = {}
        self._close_sm = {}
        self.side_clear = 2.8            # 차선 경계 밖이라도 이 안이면 '걸친 차'로 보고 서행
        # ★옆 차선에 **나보다 느린 차**가 앞에 있으면 나란히 서기 **전에** 미리 줄인다.
        #   실측 2026-08-16(제한을 50km/h 로 올린 직후 v1): 우측 차선 18km/h 차가 10.8m 앞에
        #   있는데 50.5km/h 로 접근했다. 그 차가 가속하며 끼어들어 급제동했지만 **-0.41m 접촉**.
        #   기존 근접통과(side_clear 2.8m)는 그 차가 3.29m 에서 시작해 늦게야 발동했고,
        #   상한이 '상대속도+1.5' 라 상대가 가속하자 같이 올라가 제동이 안 걸렸다.
        #   30km/h 로 달릴 땐 애초에 이 상황을 안 만들었을 뿐, 규칙이 없던 것이다.
        self.adj_watch = 4.5             # 이 횡거리 안의 옆차선 차를 본다
        self.adj_range = 30.0            # 이 앞거리 안
        self.adj_ttc = 3.0               # 나란히 되기까지 이 시간 안이면 미리 감속
        self.adj_margin = 3.0            # 그때 허용 속도 = 상대속도 + 이만큼[m/s]
        self.side_range = 18.0           # 걸친 차 서행 적용 종거리
        self.side_pass_v = 5.0           # 걸친 차 옆 통과 속도(18km/h)
        self.cutin_window = 3.2          # 끼어드는 차 조기인지 폭(차선 경계 밖이라도 접근중이면 대응)
        # ★★끼어듦은 **속도**로 판정한다. 예전엔 '직전 프레임보다 1cm 라도 가까워지면'
        #   이었는데, 그건 25Hz 에서 0.25m/s 상당이라 **측정 잡음과 구분이 안 된다.**
        #   실측 2026-08-30 코스 E (1260,-355), 사용자 지적 "옆차선 차 보고 브레이크":
        #     옆 차로 차 dpath -3.07 -> -3.02 (**2초에 5cm**, 중앙 접근속도 0.000 m/s),
        #     방위차 -4°(나란함), 속도 14m/s 로 **앞서 나가는 중**. 완벽히 자기 차로다.
        #     그런데 ±0.01m 양자화 잡음이 cutin 으로 읽혀 앞차 취급 -> FOLLOW_BRAKE 가
        #     `sqrt(2*1.5*(10.6-10.0))` = 1.34m/s 상한을 걸어 30 -> 24.5km/h 급제동.
        #   실제 끼어들기는 3.5m 를 3초 안에 좁힌다(~1.2m/s). 차선변경도 ~0.7m/s.
        #   곡선 표류·잡음은 0.1m/s 아래다. 그 사이에 문턱을 둔다.
        self.cutin_rate = 0.35           # 이 속도 이상으로 좁혀와야 '끼어든다'[m/s]
        self.cutin_hold = 2              # 그 상태가 이만큼 연속이어야 한다[프레임]
        self.ped_watch = 4.5             # 보행자 감시 폭
        self.STRAIGHT_LAT = 3.0          # 직선 전방거리를 쓰려면 직선 횡거리도 이 안(+ped_watch)이어야 한다[m]
        self.ped_pass_v = 2.0            # 서 있는 사람 옆을 차선 바꿔 지날 때 속도(7km/h)
        self.ped_still_margin = 1.0      # 내 차로 반폭 + 이만큼 밖이면 '진로 밖'[m]
        self.ped_aside_clr = 0.4         # 서 있는 사람 옆 실여유(차체~사람)가 이만큼이면 서행 통과[m]
        self.ped_edge_pass_v = 2.0       # 그때의 통과 속도(7km/h)
        # ★★**평평한 속도 캡은 가까울 때만 건다.** PED_LEAVING·PED_STILL_ASIDE·
        #   PED_LANECHANGE 는 거리와 무관한 고정값(5.0/…)이라, 멀리 있는 사람 하나가
        #   그대로 18km/h 캡이 된다. YIELD_PED 는 `_stop_target_speed` 로 거리에
        #   비례하니 멀면 저절로 무해한데, 이 셋만 거리 검사가 없었다.
        #   실측 2026-08-28 코스 A t=480.0 (1183,-505): **앞범퍼 64m 앞** 이륜차가
        #   멀어지는 중이라 PED_LEAVING -> vcmd 18km/h, 30.9km/h 에서 **a=-5.00**.
        #   사용자가 "유령 보고 멈춤"이라 한 게 이것이다.
        self.ped_side_near = 20.0        # 이 안(앞범퍼 기준)일 때만 고정 캡을 건다[m]
        # ★사람 앞 정지간격은 **앞범퍼 기준**이다. 예전엔 `fx - 6.0` 이었는데 fx 는 뒷축
        #   기준이라 실여유가 6.0-3.81=**2.19m** 밖에 안 됐다(신호등 쪽은 이 보정을
        #   `stop_margin` 주석에 적어놨으면서 사람 쪽만 빠져 있었다).
        self.ped_gap = 5.0               # 앞범퍼 ~ 사람 정지간격[m]
        # ★'미리 줄이기'. 위 양보는 |fy|<4.5 에 들어와야 발동하는 **반응**이라,
        #   3m/s 로 뛰어드는 사람은 창에 들어온 뒤 1.4초밖에 안 남는다(50km/h 기준).
        #
        # ⚠️⚠️ 여기 처음 넣었던 판정이 **자기모순이었다**(실측 2026-08-19 v6, 녹색신호):
        #      `reach = 차선반폭 + 사람속도 × 자차도달시간` 으로 '닿을 수 있는 폭'을 쟀는데,
        #      가까워질수록 도달시간이 줄어 **감시 폭이 사람보다 빨리 좁아진다.**
        #        t=13.02  사람 28.3m 앞·옆 11.4m  reach 7.0   -> 발동 안 함
        #        t=14.50  사람  8.5m 앞·옆  5.0m  reach 2.8   -> 발동 안 함
        #      그래서 **8.5m 앞까지 50km/h** 로 갔고 실여유 **1.09m** 로 스쳤다.
        #      → 폭은 자차 속도에 **줄지 않는** 고정값이어야 한다. 대신 '다가오는 중'
        #        (closing)만 보고, 실제 감속은 아래 거리 프로파일이 매끄럽게 만든다.
        #      ⚠️ 폭을 넓히면 **인도를 걷는 사람마다** 걸린다. 그래서 방향이 아니라
        #         '내 진로 쪽으로 다가오는 **속도**'를 재서 거른다 — 나란히 걷는 사람은
        #         횡속도가 0 이라 안 걸리고, 가로지르는 사람만 걸린다.
        self.ped_caution_lat = 15.0      # 움직이는 사람을 이 횡거리부터 본다[m]
        self.ped_close_v = 0.5           # 이만큼 빠르게 진로 쪽으로 와야 대상[m/s]
        self.ped_caution_a = 2.5         # 미리 줄일 때 상정하는 감속[m/s^2]
        self.ped_caution_min = 2.5       # 예측만으로는 이 아래로 안 내린다(서다 갇히는 것 방지)
        # ★비보호 좌회전(화살표가 없는 교차로) — 아래 TL 절 참고
        # ★★75초였다. **대회 채점이 그걸 중대 위반으로 본다**
        #   [안내문 2026-08-27 · 평가항목 8]: "녹색신호 통과 — 정지선 30m 이내에서
        #   이유 없는 정차 / **5초 이상: 경미(-3) / 10초 이상: 중대(-6)**".
        #   화살표를 기다리는 건 우리에겐 이유가 있지만 채점기에는 '녹색인데 서 있음'이다.
        #   원래 판정('녹색을 두 번 봐야 화살표 없음 확정')은 **한 주기를 통째로 버린다** —
        #   첫 녹색 15~30초를 서 있다는 뜻이고 그것만으로 -6 이다.
        #   -> **이 녹색에서** green_probe 초 안에 화살표가 안 나오면 비보호로 본다.
        #      틀려도(사실은 화살표가 곧 나올 신호였어도) 비보호 좌회전은 대향차를
        #      양보하며 가므로 위험하지 않다. 반면 기다리면 감점이 확정이다.
        self.green_probe = 3.5           # 녹색 진입 후 이만큼 화살표가 없으면 비보호[s]
        self.left_arrow_wait = 75.0      # 안전망(이 신호를 통째로 이만큼 봤으면)[s]
        self._green_since = {}           # id -> 이번 녹색이 시작된 시각
        self.unprotected_left_v = 4.0    # 비보호 좌회전 통과 속도(14km/h)
        # ★대향차 양보 — 비보호 좌회전은 마주오는 직진차가 우선이다(도교법 제26조).
        #   ⚠️ 9초는 넉넉해 보이지만 **좌회전 자체가 오래 걸린다.** 실측(오프라인 판):
        #      정지 상태에서 출발해 대향 차로를 다 건너기까지 6.5초, 완전히 비우기까지
        #      8초 남짓이다. 처음에 5초로 뒀다가 **그대로 충돌했다**(대향차 79m/8m/s
        #      = 9.9초로 보였는데, 우리가 마주 다가가니 실제 여유는 그보다 훨씬 짧다).
        # ★신호 없는 교차로에서의 회전 양보 [법 제26조④]. 값이 신호 있는 비보호 좌회전
        #   (9초·25m)보다 **훨씬 좁다** — 그 값을 여기 쓰면 무신호 교차로마다 걸려 못 간다.
        #   여기서 잡아야 하는 건 '내가 들어갈 자리를 지금 가로지르는 차' 하나뿐이다.
        # ★★**닿을 코스면 누구 우선인지 따지기 전에 선다.** 아래 CPA 충돌 가드.
        #   양보 규칙(제26조)은 '교차로에서 좌회전할 때'처럼 조건이 붙는데, 실제로 스친
        #   것들은 그 조건 밖이었다(우회전 중 · 뒤에서 추월해 파고든 차 · 직진 중 횡단).
        #   그래서 **규칙이 아니라 기하로** 한 겹 더 깐다 — 최근접(CPA)이 차체 안으로
        #   들어오면 우선순위와 무관하게 감속한다.
        self.cpa_horizon = 2.5           # 이 시간 안에 최근접이 오는 것만 본다[s]
        self.cpa_margin = 0.5            # 차체 반폭 + 상대 반폭 위에 이만큼 더[m]
        self.cpa_min_v = 2.0             # 이보다 느린 물체는 여기서 안 본다[m/s]
        self.cpa_vru_v = 3.0             # 취약대상도 이보다 빠르면 **차처럼** 본다[m/s] = 10.8km/h
        self.cpa_unc = 0.8               # 최근접까지 1초당 반경에 더하는 예측 불확실성[m/s]
        # ★★**나란히 가는 차는 보지 않는다.** 방위차가 작으면 제 차로를 가고 있는 것이고,
        #   그걸 직선 외삽하면 **작은 방위차가 몇 초 뒤 큰 횡이동으로 부풀어** 옆차로 차마다
        #   급제동한다. 실측 2026-08-28 코스 A(사용자 지적 "옆차선에 차 지나간다고 브레이크"):
        #       t=570.2 옆 fy-3.4 · 49km/h · **dh +3°**    -> CPA_BRAKE (a=-5.00)
        #       t=573.0 옆 fy-2.5 · 51km/h · **dh +1°**
        #       t=579.7    fy-1.0 · 54km/h · **dh +163°**(대향차)
        #     3° 면 2.5초에 1.8m 라 문턱을 넘긴다. 곡선에서는 더 심하다.
        #   ⚠️ 이 가드가 잡아야 하는 **진짜** 접촉 3건의 방위차는 -89° · -17° · -99° 였다.
        #      끼어들기는 각도가 **생긴 뒤에** 닿는다(코스 B 뒤차 -17°). 문턱을 그 사이에 둔다.
        #   ⚠️ 대향차는 `HEADON_STOP` 과 차체 안전망이 따로 본다. 여기서 또 볼 일이 아니다.
        self.cpa_parallel_deg = 12.0     # 방위차가 이보다 작으면 '나란히'[도]
        self.cpa_headon_deg = 150.0      # 이보다 크면 '마주 나란히'(대향차)[도]
        # ★★**자차가 도는 중이면 '지금 나란히'는 거짓말이다.** 위 CPA 가드는 두 차가
        #   모두 직진한다고 보고 최근접을 잡는데, 회전 중에는 자차 궤적이 원호라
        #   같은 방향으로 나란히 달리던 차를 **내가 가로질러 들어간다**.
        #   실측 (1456,939) 무신호 73° 좌회전 — 같은 자리에서 세 번 스쳤다:
        #     2026-08-26 코스 E -0.65m · 코스 H -0.19m · 2026-09-06 코스 E -0.19m
        #   2026-09-06 판 로그: t=146.0 에 이미 **뒤 27.6m·왼쪽 3.5m 에 52km/h** 로
        #   따라오는 차가 보였다. 그런데 그때 방위차는 1°(='나란히')라 CPA 가 통째로
        #   건너뛰었고, 자차가 25° 돌아 방위차가 12° 를 넘긴 t=149.9 에야 CPA_BRAKE 가
        #   걸렸다 — 접촉 0.2초 전이다. 4초 동안 보이던 차였다.
        #   -> 회전 중에는 **자차를 원호로 굴려** 닿는지 본다(상대는 직진 외삽 그대로).
        #   ⚠️ 차로변경(2~3°/s)과 교차로 회전(15~20°/s)을 요레이트로 가른다.
        self.turn_yaw_min = 0.12         # 이보다 빨리 돌 때만 본다[rad/s] (~7°/s)
        self.turn_cut_horizon = 2.5      # 원호를 이만큼 앞까지 굴린다[s]
        self.turn_cut_step = 0.1         # 격자[s]
        self.turn_cut_deg = 20.0         # 경로가 이만큼 돌아간 구간에서만 본다[도]
        self.turn_cut_path_t = 4.0        # 경로를 아는 경우의 예측 지평[s]
        # ★★**차선을 지키는 차는 보지 않는다 — 커브에서 자차 헤딩 기준은 무너진다.**
        #   사용자 지적 2026-08-30: "꺾인 도로에서 옆차선 차량 보고 브레이크 잡지 마.
        #   차선이 뭔지 모름? 일반적인 경우는 다 차선 따라가지. 핸들을 갑자기 트는
        #   경우에만 브레이크 잡아라." — 맞는 말이고 데이터도 그렇다.
        #   실측 코스 E (776,566) t=282~288, 옆차로 이륜차 하나:
        #     경로기준 횡위치  -3.48 -> -3.11 -> -3.05 -> -2.98 -> -2.72 -> -3.06
        #                      (내내 옆차로. 제 차선을 그대로 간다)
        #     자차기준 fy      -3.10 -> -1.90 -> -1.00 -> +1.10 -> +0.30
        #                      (**우리가 커브를 도니까** 앞으로 쓸려오는 것처럼 보인다)
        #   위 parallel/headon 필터는 `o.heading - ego.heading` 을 쓰는데, 커브에서는
        #   자차 헤딩이 계속 돌아서 나란히 가는 차도 문턱 밖으로 나간다.
        #   -> **경로 기준 횡위치가 내 차로 밖이고 다가오지 않으면** 보지 않는다.
        #      진짜 끼어들기·횡단은 횡위치가 빠르게 줄어들므로 그대로 걸린다.
        self.cpa_keep_lane_m = 0.5       # 내 차로 반폭 + 이만큼 밖이면 '옆차로'[m]
        self.cpa_close_v = 0.6           # 횡으로 이보다 빨리 다가올 때만 본다[m/s]
        self.nosig_near = 25.0           # 교차로가 이 안에 있으면 무장[m]
        # ★★**정지점이 0m 면 무장하지 않는다** — 그건 '여기서 서라'가 아니라
        #   **이미 교차로 안**이라는 뜻이다(`_junction_ahead` 는 교차로 안에서 0.0 을 준다).
        #   우회전 일시정지(rt_min_arm)에서 이미 고친 버그가 여기만 안 들어가 있었다.
        #   실측 2026-08-30 코스 E (1454,940) — jx=1 교차로 **한복판**:
        #     t=143.2 YIELD_CROSS -> _stop_target_speed(0 - 5.0) = 0 -> 14.7km/h 에서 완전정지
        #     t=143.2~145.6 **2.5초 정지**(방위 114°->151°, 회전 도중이었다)
        #   d_ego 는 내내 0.00 이라 경로 추종은 완벽했다 — 판정이 틀린 것이다.
        #   [법 제32조] 교차로 내 정차이고, 대회 채점으로는 그 2.5초가 차로 판정 밖이라
        #   항목 4(중앙선 0.6m·0.6초 중대)·5(보도 0.5m·0.3초 중대)에 걸린다 = -12점.
        #   ⚠️ 이미 들어갔으면 **서행으로 빠져나가는 게 맞다.** 실제 충돌 위험은
        #      CPA_BRAKE 와 차체 안전망이 따로 본다 — 그게 이 규칙보다 아래 겹이다.
        self.nosig_min_arm = 0.5         # 정지점이 이보다 가까우면 '교차로 안' — 무장 안 함[m]
        self.nosig_gap = 5.0             # 이 시간 안에 내 자리를 지날 차를 본다[s]
        self.nosig_radius = 8.0          # 그 자리 이 반경 안이면 양보[m]
        self.nosig_wait = 12.0           # 이만큼 기다렸으면 서행으로 진입(영구 대기 방지)[s]
        self._nosig_since = None         # 무신호 교차로 양보 시작 시각
        self.oncoming_gap = 9.0          # 이 시간 안에 스쳐 지나갈 차가 있으면 안 꺾는다[s]
        self.oncoming_radius = 25.0      # 그 차가 내 자리 이 반경 안으로 들어오면 위험[m]
        self.nominal_dt = 0.04           # ego.t 를 못 믿을 때 쓸 프레임 간격[s]
        self._prev_d = {}                # id -> 직전 횡오프셋(끼어듦 판정용)
        self._cutin_v = {}               # id -> 평활한 횡접근 속도[m/s]
        self._cutin_n = {}               # id -> 연속으로 문턱을 넘은 프레임 수
        # ★굽은 길에서 옆차선 차가 투영 오차로 **한 프레임에 몇 m 씩** 내 차로 안으로
        #   뛰어 들어와 보인다. 실제 차의 횡이동은 ~3m/s 가 한계라 프레임당 0.25m 안팎
        #   — 그보다 큰 점프는 물리가 아니라 투영 잡음이다. 그런 프레임과, 방금
        #   튀어 들어온(연속으로 차로 안에 있지 않은) 차는 앞차로 잡지 않는다.
        #   실측 2026-08-30 코스 E (763,555): 갈래 2개 모퉁이(93° 굽이)에서 옆차선
        #   차를 보고 FOLLOW vcmd=0 급제동 — LANE_KEEP 과 1~2초 간격으로 번갈아
        #   나타났다(투영이 차로 경계를 들락거린 것). 정지물(스폰 포함)은 필터에서
        #   뺀다 — 서 있는 건 늦게 서는 쪽이 더 위험하다.
        self.lat_jump = 0.6              # 프레임당 이보다 크게 점프한 횡값은 못 믿는다[m]
        self.lat_jump_rate = 6.0         # 안전망용 같은 판정, 시간 기준[m/s]
        self.lane_streak_n = 3           # 이만큼 연속으로 차로 안+연속적이어야 앞차다
        self._lane_streak = {}           # id -> 연속으로 '차로 안 & 연속 투영' 프레임 수
        self._prev_ped = {}              # id -> 직전 |횡오프셋|(보행자 이탈 판정용)
        # ★차체 안전망의 **예측**용 이력. 지금 여유만 보면 늦는다 —
        #   실측 2026-08-25 코스 G: 오른쪽으로 추월하던 차가 급정거해 앞우측 6.9m 에
        #   섰고, 우리는 최대제동으로도 2.2m 밀려 -0.40m 로 물렸다(그 뒤 503초 정지).
        #   그 차는 그 전 1초 동안 이미 우리 쪽으로 좁혀오고 있었다.
        self._lat_hist = {}              # id -> (|횡오프셋|, 시각)
        self.pred_horizon = 1.5          # 이 시간 뒤까지 내다본다[s]
        self.pred_min_rate = 0.3         # 이보다 느리게 좁혀오면 잡음으로 본다[m/s]
        self._ped_fy = {}                # id -> 평활한 |횡오프셋|(횡속도 추정용)
        self._ped_t = None               # 직전 프레임 시각
        self._cpa_lat = {}               # id -> 직전 프레임의 경로기준 |횡위치|
        self._fast_ids = set()           # 차량 속도로 달린 적 있는 id (drive.py 가 같은 set 을 물린다)
        self._prev_h = None              # 직전 프레임 heading (요레이트용)
        self._yaw_rps = 0.0              # 평활된 요레이트[**rad**/s]
        #   ⚠️ drive.DrivingStack 쪽 같은 이름의 값은 **도/s** 다(지시등 래치용).
        #      단위가 달라 헷갈리기 쉬워 이름을 나눠 뒀다.
        # ★★**트랙이 끊긴 id 의 이력은 버린다.** 위의 per-id 이력들(_prev_fx·_prev_d·
        #   _cutin_v…)은 물체가 감시 반경(140m)을 나갔다 돌아와도 남아 있다. 그러면
        #   몇 초 전 위치와 지금 위치의 차이를 **한 프레임의 이동**으로 읽는다.
        #   제일 위험한 건 `_prev_fx` 다: 20초 전 fx=80 이던 차가 fx=30 으로 다시
        #   보이면 close=(80-30)/dt_f(0.04) = **1250 m/s** -> head_on 오판 ->
        #   HEADON_STOP 급제동. (dt_f 는 0.5s 넘는 공백이면 공칭값으로 떨어져
        #   시간으로도 안 걸러진다.) 끊겼다 돌아온 id 는 첫 관측으로 취급한다.
        self.track_gap = 1.0             # 이보다 오래 안 보였으면 그 id 의 이력은 못 믿는다[s]
        self._seen = {}                  # id -> 마지막으로 본 시각
        self._tl_has_left = set()        # 좌회전 화살표를 **본 적 있는** 신호 id
        self._tl_green_n = {}            # id -> 녹색을 몇 번 봤나(주기가 한 바퀴 돌았나)
        self._tl_prev = {}               # id -> 직전 상태(녹색 진입 시점 검출용)
        self._red_since = {}             # id -> 이번 적색이 시작된 시각(적색 우회전 정지 인정용)
        self._left_wait_since = {}       # id -> 그 신호 앞에 선 시각
        self.reason = "LANE_KEEP"

        # ★최후 안전망 — 차선 계산과 **무관하게** 내 차체가 지나갈 통로만 본다.
        #   왜 필요한가(2026-08-15 EV_CUTIN 실측): 위 판정들은 `rel = fy - lane_offset` 을 쓰는데
        #   fy(rf_objs 의 d)는 이미 ego 기준 상대값이라 오프셋을 **두 번** 뺀다. 연료통 회피로
        #   1.62m 비켜 있을 때 오른쪽 1.62m 의 차가 3.24m 밖으로 보여 차선판정(1.75)도
        #   근접통과(2.8)도 전부 빠져나갔고, 23km/h 로 가속해 옆구리를 스쳤다(실여유 -0.98m).
        #   오프셋 부기(簿記)가 어긋나도 이 검사만은 살아있게 둔다.
        self.half_width = 0.943          # 차폭 1.886m 의 절반
        self.body_gap_min = 0.35         # 이보다 좁으면 서행(검증된 연료통 통과 0.45m 는 통과)
        # 자전거·이륜차 판정(원본 치수). 라바콘(0.46x0.15x0.61)은 키에서,
        # 승용차(1.8m)·트럭(2.2m)은 폭에서 걸러진다.
        self.two_wheel_w = 1.0           # 이보다 좁고
        self.two_wheel_l = 3.0           # 이보다 짧으며 키가 크면 이륜차
        # ★★**도로를 달리는 이륜차는 '차'로 본다.** 사람 취급을 벗기는 게 아니라
        #   **양보(정지) 규칙의 대상에서만** 뺀다 — 옆여유 1.2m 안전망은 그대로 간다.
        #   왜(실측 2026-08-28 코스 A 라이브, 사용자 지적 2건):
        #     t=289.2 (1186,93)  경로횡 **+3.4m**(옆차로) · 2.0x0.60x1.70 · **16.8m/s(60km/h)**
        #                        -> YIELD_PED, 30.3 -> 10.4km/h. 보행자는 60km/h 로 못 간다.
        #     t=688.4 (1188,-574) 경로횡 **+2.7m**(대향차로) · 방위차 **-177°** · 7.0m/s
        #                        -> YIELD_PED **완전정지**(a=-5.00). 마주 오는 이륜차다.
        #   제27조는 '횡단하거나 횡단하려는' 보행자 보호다. 나란히·마주 달리는 이륜차는
        #   횡단자가 아니라 교통이고, 그건 ACC·CPA·차체 안전망이 담당한다.
        #   ⚠️ **내 차로 안**의 이륜차는 그대로 사람처럼 다룬다(느린 자전거 뒤를 따라간다).
        self.two_wheel_ride_v = 8.0      # 이보다 빠른 이륜차는 '주행 중인 차'(29km/h)
        self.two_wheel_par_deg = 25.0    # 방위차가 이 안이면 나란히, 180±이 안이면 마주
        self.two_wheel_out_m = 0.3       # 내 차로 경계에서 이만큼 밖일 때만 적용
        self.body_gap_ped = 1.2          # 사람 옆은 이만큼 벌어질 때까지 붙지 않는다
        self.body_stop_gap = 1.5         # 막힌 물체의 **뒷모서리** 앞 이만큼에 선다[m]
        self.body_sweep_block = 0.5      # 비스듬한 정지차가 경로 폭으로 이보다 깊이 침범하면 SAT 무시(멈춘다)[m]
        self.body_crawl = 2.0            # 좁은 통로 통과 속도(7km/h)
        self.body_watch = (-4.0, 25.0)   # 이 종거리 범위만 본다(앞 25m ~ 옆·살짝 뒤)
        # per-id 이력 전부(위 track_gap 주석). 새 이력 dict 를 추가하면 여기도 넣을 것.
        self._tracks = (self._prev_d, self._cutin_v, self._cutin_n, self._lane_streak,
                        self._prev_ped, self._ped_fy, self._prev_fx, self._close_sm,
                        self._lat_hist, self._cpa_lat)

    def _purge_stale_tracks(self, rf_objs, now):
        """트랙이 끊겼다 돌아온 id 의 이력을 버린다(위 track_gap 주석)."""
        for ob in (rf_objs or ()):
            oid = ob[6] if len(ob) > 6 else None
            if oid is None:
                continue
            last = self._seen.get(oid)
            if last is not None and now - last > self.track_gap:
                for d in self._tracks:
                    d.pop(oid, None)
            self._seen[oid] = now
        # 메모리: 오래 안 보인 id 는 통째로 버린다(긴 판에서 무한히 자라지 않게).
        if len(self._seen) > 64:
            for k in [k for k, t0 in self._seen.items() if now - t0 > 10.0]:
                self._seen.pop(k, None)
                for d in self._tracks:
                    d.pop(k, None)

    def _cap(self, v, limit, name):
        """속도 상한 하나를 **기록하면서** 적용한다.

        ★왜: 속도를 깎는 곳이 18군데인데 `reason` 은 규칙마다 제각각(어떤 건 무조건
          덮어쓰고 어떤 건 LANE_KEEP 일 때만) 이라, 로그의 reason 이 **실제로 속도를
          잡은 규칙과 다를 수 있다**. 실주행 CSV 로 "왜 느리지"를 역추적하는 데
          오늘 하루 많은 시간을 썼다(유령 브레이크·영구정지·갇힘 전부 이 문제였다).
          여기서 전부 모아두고 `cap_by` 로 **이긴 제약**을 따로 내보낸다.
        ⚠️ reason 의미는 건드리지 않는다 — 테스트 166개와 채점기 분석이 그걸 쓴다.
        """
        self._caps.append((limit, name))
        return limit if limit < v else v

    def _oncoming_clear(self, ego, at=None, gap=None, radius=None):
        """양보해야 하나 — 내가 들어갈 자리를 스쳐 갈 차가 있나.

        `at` = 판정 기준점(자차 기준 상대좌표). 없으면 **자차 위치**로 본다.
        ★왜 기준점이 필요한가(2026-08-25): 적신호 우회전에 이걸 자차 위치로 쓰면
          **뒤차와 왼쪽으로 지나가는 차까지** 충돌 상대로 잡힌다. 실측 코스 H:
          왼쪽 3.1m 에 있던 1.4m/s 짜리 차 하나 때문에 18초를 서 있었다.
          우회전의 상대는 **회전을 마친 뒤 내가 있을 자리**를 지나갈 차다.

        ⚠️⚠️ 처음엔 `heading 차이가 120° 이상이면 대향차` 로 짰다가 **실주행에서
           한 대도 못 걸렀다**(2026-08-20 HL_FMA_LEFTONC). 그 교차로는 T자로라
           충돌 상대가 정면이 아니라 **옆 90°** 에서 온다 — ego 헤딩 308°, 상대 218°.
           좌회전의 충돌 상대는 '마주오는 차'만이 아니라 **내가 들어갈 자리를 지나갈 차**
           전부다. 방향으로 가르면 교차로 모양마다 틀린다.

        -> 최근접 접근(CPA)으로 본다. 상대속도로 예측해서, 내 자리 `oncoming_radius`
           안으로 `oncoming_gap` 초 안에 들어오면 기다린다. 기하에 무관하다.
        ⚠️ 사람은 여기서 안 본다 — 보행자 규칙이 따로 있고, 여기 넣으면 횡단보도
           보행자 때문에 좌회전이 영영 안 된다.
        """
        gap = self.oncoming_gap if gap is None else gap
        radius = self.oncoming_radius if radius is None else radius
        px, py = at if at else (0.0, 0.0)
        for o in getattr(ego, "objects", ()) or ():
            if max(o.length, o.width) < 1.2 or o.speed < 0.5:
                continue
            fx, fy = to_ego_frame(ego, o)
            dh = o.heading - ego.heading
            # ★★**내 뒤에서 같은 방향으로 오는 차는 상대가 아니다**(2026-09-06 코스 E 신호108).
            #   우리 차로 뒤 78m 에서 40km/h 로 다가오는 차의 CPA 는 정확히 **내 자리**다
            #   (내 뒤에서 설 테니까). 그걸 '내 자리를 스쳐 갈 차'로 봐서 비보호 좌회전을
            #   녹색 내내 양보했다 — 11.5초 양보 중 5초가 이것, 결국 한 주기(36초)를 놓쳤다.
            #   뒤차는 좌·우회전과 무관하다 — 제26조의 상대는 교차로에 **들어오는** 차다.
            if fx < 0.0 and math.cos(dh) > 0.7:
                continue
            fx, fy = fx - px, fy - py            # 기준점 기준으로 옮긴다
            vx, vy = o.speed * math.cos(dh), o.speed * math.sin(dh)
            vv = vx * vx + vy * vy
            if vv < 0.25:
                continue
            t_cpa = -(fx * vx + fy * vy) / vv
            if t_cpa < 0.0 or t_cpa > gap:
                # 이미 지나갔거나 한참 뒤 -> 지금 결정과 무관. 단 **당장 코앞**이면 본다.
                if math.hypot(fx, fy) > radius:
                    continue
                t_cpa = 0.0
            cx, cy = fx + vx * t_cpa, fy + vy * t_cpa
            if cx < -2.0:                 # 내 뒤로 지나간다 -> 좌회전과 무관
                continue
            if math.hypot(cx, cy) < radius:
                return False
        return True

    def _cpa_conflict(self, ego, rf_objs=None, dt_f=0.04, lane_offset=0.0):
        """**닿을 코스인 움직이는 물체**의 최근접점까지 거리[m]. 없으면 None.

        `_oncoming_clear` 와 같은 CPA 계산이지만 쓰임이 다르다 — 저건 '회전해도 되나'를
        묻는 **양보** 판정이고(반경 8~25m 로 넓게), 이건 '이대로 가면 닿나'를 묻는
        **충돌 가드**다. 그래서 반경이 **실제 차체 크기**다(자차 반폭 + 상대 반폭 + 0.5m).

        왜 필요한가(실측 2026-08-27, 주변교통 3판이 같은 날 같은 식으로 스쳤다):
          코스 A t=653  우회전 중 왼쪽에서 90° 로 34km/h 접근 -> 여유 -0.71m
          코스 B t=363  뒤에서 33km/h 로 추월해 들어옴(-39°->-15°) -> 여유 -1.03m
          코스 D t=186  직진 중 왼쪽에서 -99° 로 28km/h 횡단 -> 여유 -1.45m
        셋 다 **1.1~2.0초 전부터 보이던 차**인데 그 시간에 우리는 가속하고 있었다.
        양보 규칙으로는 못 잡는다 — 하나는 우회전, 하나는 뒤차, 하나는 직진 중이라
        `YIELD_CROSS`(무신호 좌회전)의 조건 밖이다. 그래서 기하로 깐다.

        ⚠️ 사람은 안 본다(보행자 규칙이 따로 있다). 느린 물체도 안 본다 — 정지차·서행차는
           차체 안전망과 ACC 담당이고, 여기까지 넣으면 정체마다 급정거한다.
        """
        # ★경로 기준 정보를 id 로 찾을 수 있게 해 둔다(횡위치·접근속도 판정용).
        lat_of = {}
        for ob in (rf_objs or ()):
            oid = ob[6] if len(ob) > 6 else None
            if oid is not None:
                lat_of[oid] = ob[1]                 # d = 경로기준 상대 횡위치
        best = None
        for o in getattr(ego, "objects", ()) or ():
            if o.speed < self.cpa_min_v:
                continue
            # ★★제 차선을 지키는 차는 보지 않는다(위 cpa_keep_lane_m 주석).
            oid_ = getattr(o, "id", None)
            lat = lat_of.get(oid_)
            if lat is not None:
                lat_abs = abs(lat + lane_offset)
                prev = self._cpa_lat.get(oid_)
                self._cpa_lat[oid_] = lat_abs
                closing = 0.0 if prev is None else (prev - lat_abs) / max(dt_f, 1e-3)
                if (lat_abs > self.lane_half + self.cpa_keep_lane_m
                        and closing < self.cpa_close_v):
                    continue                        # 옆차로에서 제 갈 길 가는 중
            # 사람·이륜차·콘은 제외 — 판정은 **behavior 의 다른 자리와 같은 식**이어야 한다.
            # ⚠️ `max(길이,폭) < 1.2` 만으로는 **VTD 보행자(2.0x0.6x1.7)가 새 나간다.**
            #    오늘 아침 추월 FSM 에서 똑같이 당했다(3a446d7).
            #  사람·이륜차·휠체어는 `is_vru`, 낮은 소형물(콘)은 크기로 뺀다.
            #   ★속도도 넘긴다(2026-09-05, vtd_io.is_vru 주석). 62km/h 이륜차가 '사람'으로
            #     분류돼 CPA 가드에서 **빠지면** 충돌코스인데도 안 본다. 한 번 빠르게 달린
            #     id 는 느려져도 차량으로 둔다(`_fast_ids`, drive.py 와 공유).
            _sp = getattr(o, "speed", 0.0) or 0.0
            _id = getattr(o, "id", None)
            if _sp > VRU_MAX_SPEED and _id is not None:
                self._fast_ids.add(_id)
            _vru = (_id not in self._fast_ids) and is_vru(o.length, o.width, o.height, _sp)
            # ★★**자전거·킥보드는 차처럼 다닌다 — 여기서도 차로 본다**(2026-09-11 코스 A TRV).
            #   보행자 규칙(YIELD_PED·CROSSWALK_PED)은 '가까이 있는 느린 사람'을 보는 자다.
            #   25km/h 로 옆에서 가로질러 오는 자전거는 그 자로는 안 잡힌다 — 실측 t=41.6 에
            #   25m 앞 27m 오른쪽에서 7m/s 로 보이기 시작했는데 우리는 적신호 출발 가속 중이었고,
            #   t=45.4 에 접촉했다(여유 -1.07m). 이 판을 오프라인 재생하니 취약대상 제외 규칙
            #   때문에 CPA 가 **한 프레임도** 안 걸린다(실제 판에서 걸린 건 그 자전거가 한 번
            #   40km/h 를 넘겨 `_fast_ids` 에 들어간 우연이었다 — 운에 기대면 안 된다).
            #   ⚠️ `cpa_min_v`(2.0) 위에 문턱을 하나 더 둔다. 걷는 사람(1.4)·느린 조깅(2.5)은
            #      그대로 보행자 규칙 몫이다. 반경은 실제 차체라 '닿을 때'만 걸린다.
            #   실측 대조: 코스 H 주변교통 409초·H TRV 593초 판에서 이 규칙으로 늘어난
            #   프레임은 **0개**다.
            if _vru and _sp > self.cpa_vru_v:
                _vru = False
            if _vru or max(o.length, o.width) < 1.2:
                continue
            fx, fy = to_ego_frame(ego, o)
            dh = o.heading - ego.heading
            # ★나란히 가는 차(같은 방향/마주 방향)는 제 차로를 가는 것이다 — 보지 않는다.
            adh = abs(math.degrees((dh + math.pi) % (2 * math.pi) - math.pi))
            if adh < self.cpa_parallel_deg or adh > self.cpa_headon_deg:
                continue
            # 상대속도 = 물체속도 - 자차속도(자차기준 +x)
            vx = o.speed * math.cos(dh) - ego.speed
            vy = o.speed * math.sin(dh)
            vv = vx * vx + vy * vy
            if vv < 1.0:
                continue                       # 나란히 간다 -> 최근접이 안 정해진다
            t_cpa = -(fx * vx + fy * vy) / vv
            if not (0.0 < t_cpa < self.cpa_horizon):
                continue
            cx, cy = fx + vx * t_cpa, fy + vy * t_cpa
            if cx < -2.0:                      # 내 뒤에서 만난다 -> 내가 할 게 없다
                continue
            # 반경은 **실제 차체**다. 상대 폭은 자차 기준 옆폭(비스듬하면 커진다).
            half = (abs(o.length * math.sin(dh)) + abs(o.width * math.cos(dh))) / 2.0
            # ★★**멀리 내다본 예측일수록 반경을 넓힌다**(2026-09-11 코스 A TRV).
            #   이 CPA 는 둘 다 **직진**한다고 보고 최근접을 잡는다. 도는 물체에는 틀린다 —
            #   실측 그 자전거는 상대방위가 83° -> 118° 로 2초에 35° 돌며 우리 앞을 파고들었다.
            #   직진 가정으로는 t=43.6 에 4.5m 비껴 가는 것으로 나왔고(문턱 2.45m), 실제로
            #   걸린 건 접촉 1.0초 전인 t=44.4 였다. 그때 최근접점은 앞범퍼 **1.2m** 앞이라
            #   전제동으로도 못 선다. 게다가 t=44.8 에 추정치가 4cm 흔들리며 **브레이크가
            #   0.4초 풀렸다**.
            #   남은 시간에 비례해 불확실성을 더하면 t=43.6 부터 **끊기지 않고** 걸린다
            #   (재생: 44.20~44.76 · 45.16~45.76 두 토막 -> 43.64~45.76 한 줄).
            #   실측 대조: 코스 H 409초 판 +1프레임, H TRV 593초 판 +15프레임뿐이다.
            if (math.hypot(cx, cy)
                    >= self.half_width + half + self.cpa_margin + self.cpa_unc * t_cpa):
                continue
            if best is None or cx < best:
                best = cx
        return best

    def _turn_cut_conflict(self, ego, path_ahead=None):
        """**회전 중** 자차 원호가 물체와 닿나. 닿으면 그 지점까지 호 길이[m], 아니면 None.

        `_cpa_conflict` 는 두 차가 **모두 직진**한다고 보고 최근접을 잡는다. 그래서
        같은 방향으로 나란히 달리는 차를 `cpa_parallel_deg` 로 통째로 건너뛴다 —
        똑바로 갈 때는 맞다. 하지만 **내가 도는 중이면 그 차를 내가 가로지른다.**

        실측 (1456,939) 무신호 73° 좌회전(연결로 3164, 길이 95m) — 같은 자리 3회 접촉:
          2026-08-26 코스 E -0.65m · 코스 H -0.19m · 2026-09-06 코스 E -0.19m
        마지막 판 로그: t=146.0 에 뒤 27.6m·왼쪽 3.5m 에 52km/h 차가 이미 보였는데
        방위차 1° 라 CPA 가 건너뛰었다. 자차가 25° 돌아 12° 를 넘긴 t=149.9 에야
        CPA_BRAKE — 접촉 0.2초 전이다. 직선 CPA 로는 4.24m 나 비껴 가는 것으로 나온다.

        여기서는 자차를 **등속·등요레이트 원호**로 굴리고 상대는 직선 외삽한다.
        반경·최소속도·사람 제외는 `_cpa_conflict` 와 같은 기준을 쓴다.
        ⚠️ 차로변경(2~3°/s)에는 안 켜진다 — `turn_yaw_min` 7°/s 가 문턱이다.
        """
        v0 = max(getattr(ego, "speed", 0.0) or 0.0, 0.5)
        # 예측 궤적 [(t, x, y)] — **경로를 알면 경로가 답이다**(회전 전에도 보인다).
        if path_ahead:
            # 누적 **경로 길이**를 같이 들고 간다 — 직선거리로 재면 회전에서 짧게 나와
            # 필요 이상으로 세게 밟는다. 걸러내기 전에 전부를 훑어야 길이가 맞는다.
            traj, run, px, py = [], 0.0, 0.0, 0.0
            for (t, x, y, h) in path_ahead:
                run += math.hypot(x - px, y - py)
                px, py = x, y
                if abs(math.degrees(h)) >= self.turn_cut_deg:
                    traj.append((t, x, y, run))
        else:
            w = self._yaw_rps
            if abs(w) < self.turn_yaw_min:
                return None
            traj = []
            t = 0.0
            while t < self.turn_cut_horizon:
                t += self.turn_cut_step
                th = w * t
                traj.append((t, (v0 / w) * math.sin(th),
                             (v0 / w) * (1.0 - math.cos(th)), v0 * t))
        if not traj:
            return None
        best = None
        for o in getattr(ego, "objects", ()) or ():
            sp = getattr(o, "speed", 0.0) or 0.0
            if sp < self.cpa_min_v:
                continue
            _id = getattr(o, "id", None)
            if sp > VRU_MAX_SPEED and _id is not None:
                self._fast_ids.add(_id)
            _vru = (_id not in self._fast_ids) and is_vru(o.length, o.width, o.height, sp)
            if _vru or max(o.length, o.width) < 1.2:
                continue
            fx, fy = to_ego_frame(ego, o)
            dh = o.heading - ego.heading
            # ★대향차는 뺀다 — `_cpa_conflict` 와 같은 기준(cpa_headon_deg). 마주 오는 차를
            #   직선 외삽하면 굽은 길에서는 **반대 차로를 따라 오는 차가 내 원호를 가로지르는**
            #   것으로 나온다. 실측 2026-09-09 코스 E: 굽은 구간마다 대향차(방위차 ~179°)에
            #   TURN_CUT 이 걸려 코너를 기어갔다. 사용자: "코너 왜이리 느리게 돌아".
            #   교차로에서 진짜로 대향 직진차를 가로지르는 좌회전은 YIELD_ONCOMING 이 맡는다.
            adh = abs(math.degrees((dh + math.pi) % (2 * math.pi) - math.pi))
            if adh > self.cpa_headon_deg:
                continue
            ovx, ovy = sp * math.cos(dh), sp * math.sin(dh)
            # 상대 옆폭은 CPA 와 같은 식(비스듬하면 커진다)
            half = (abs(o.length * math.sin(dh)) + abs(o.width * math.cos(dh))) / 2.0
            rad = self.half_width + half + self.cpa_margin
            for (t, ex, ey, run) in traj:
                ox, oy = fx + ovx * t, fy + ovy * t
                if math.hypot(ox - ex, oy - ey) <= rad:
                    if best is None or run < best:
                        best = run
                    break
        return best

    def _rtor_pos_ok(self, d_line, tl_id):
        """적신호 우회전 일시정지로 인정되는 자리인가(앞범퍼~정지선 `d_line`, 음수 = 넘음).

        기본은 선 **전** [0, rtor_bumper_gap_max). 적색이 켜질 때 **이미 선을 넘어 있었다면**
        (녹색에 들어가다 앞이 막혀 선 경우) 넘은 쪽 rtor_past_max 안도 인정한다(위 주석).
        """
        if d_line is None:
            return False
        if 0.0 <= d_line < self.rtor_bumper_gap_max:
            return True
        onset = self._red_onset_dline.get(tl_id) if (tl_id and tl_id > 0) else None
        return onset is not None and onset < 0.0 and -self.rtor_past_max < d_line < 0.0

    def _stop_target_speed(self, dist):
        """dist[m] 앞에서 멈추기 위한 현재 허용속도(완만한 계획감속 -> 일찍 제동)."""
        if dist <= 0.3:
            return 0.0
        return math.sqrt(2 * self.a_plan * dist)

    SWEEP_NOW = 0.6            # 지금 이 자리 실여유가 이보다 커야 서행을 허용[m]

    def _sweep_ok(self, ob, ego_yaw, lat_plan, need_gap, now_min=None):
        """옆으로 빠지는 중 — 완전정지 대신 서행해도 되나. **둘 다** 참이어야 한다.

        now_min: ① 의 문턱을 이 값으로 낮춘다(None 이면 SWEEP_NOW). 차선을 바꿔 **서 있는**
                 사람·자전거 옆을 지나기로 한 때(ped_bypass) 쓴다 — 목표 여유가 0.67m 인
                 궤적은 모서리가 도는 동안 그보다 좁게(0.56~0.60m) 스친다. 0.6 을 고집하면
                 8.8m 앞에서 시작한 회피가 매번 여기서 굳는다(2026-09-09 모의 accident_junction,
                 헤딩 27° 에서 clr 0.598). 물리적으로는 닿지 않는 값이다.

        ① **지금 실제로 안 닿아 있다** — 상자 대 상자(SAT)로 잰다. 이 판정을 경로축
           외접상자로 하면 비스듬한 차의 옆폭이 부풀어 없는 접촉이 나온다
           (`run_logger` 가 같은 이유로 SAT 를 쓴다 — `oriented_gap` 하나를 같이 쓴다).
        ② **다 빠지고 나면 옆을 지나갈 수 있다** — 그때는 경로에 정렬해 있으므로
           경로축 폭(`owid`)으로 재는 게 맞다. 종방향 간격은 여기서 빼고 **옆폭만** 본다
           — 안 그러면 '앞에 멀리 있는 차'가 SAT 거리 덕에 통과돼 그대로 들이받는다.

        ⚠️ 옛 NARROW_CREEP 은 ②가 없어서 **물체 쪽으로 기어가기만 하고 못 나갔다**
           (2026-08-16 되돌림). ②가 그 재발을 막는다.
        """
        if abs(lat_plan) < 0.2:
            return False                       # 기동 중이 아니다
        # ② 다 빠진 뒤의 옆 통로
        if abs(ob[1] - lat_plan) - self.half_width - ob[4] / 2.0 < need_gap:
            return False
        # ① 지금 이 자리의 실여유(상자 대 상자)
        raw_w = ob[7] if len(ob) > 7 else ob[4]
        raw_l = ob[8] if len(ob) > 8 else ob[3]
        c, sn = math.cos(ego_yaw), math.sin(ego_yaw)
        rx = ob[0] - EGO_CX * c                # 경로기준: 차체 중심 -> 객체
        ry = ob[1] - EGO_CX * sn
        return oriented_gap(rx * c + ry * sn, -rx * sn + ry * c,   # -> 차체 기준
                            ob[9] - ego_yaw, raw_l / 2.0, raw_w / 2.0) >= (
            self.SWEEP_NOW if now_min is None else min(self.SWEEP_NOW, now_min))

    def plan(self, ego, v_prof, tl_stop_dist=None, lane_offset=0.0, avoiding=False,
             rf_objs=None, ped_bypass=False, route_turn=0, junc_dist=None,
             tl_passed=False, rtor_point=None, lat_plan=0.0, ego_yaw=0.0,
             path_ahead=None, yield_stop_dist=None, tl_unseen_dist=None):
        """
        ego: vtd_io.State
        v_prof: 현재 지점 경로 기반 목표속도[m/s]
        tl_stop_dist: 전방 신호등 정지선까지 거리[m] (없으면 None)
        rtor_point: 적신호 우회전 양보 판정의 **기준점**(자차 기준 상대좌표) —
                    '회전을 마친 뒤 우리가 있을 자리'. drive.py 가 경로에서 뽑아 준다.
        lane_offset: 현재 주행 차선의 횡오프셋[m] (추월기가 결정) — 이 차선 기준으로 장애물 판단
        avoiding: True면 옆차선으로 우회 중 -> 정지 대신 서행 통과
        junc_dist: 전방 교차로 진입까지 거리[m] (신호 없는 교차로의 우회전 정지점)
        yield_stop_dist: 무신호 좌회전 양보 때 넘지 말아야 할 횡단보도 앞 정지점까지
                         거리[m]. None이면 기존 교차로 진입점만 쓴다.
        tl_passed: **아는 신호인데 그 정지선을 이미 지났다** = 지금 교차로 안이다.
                   (tl_stop_dist 는 그때도 None 이라 '정지선 미상'과 구별이 안 된다)
        tl_unseen_dist: 앞의 **아직 보고되지 않은** 신호 정지선까지 거리[m](뒷축 기준). 없으면 None.
                   그 신호가 보이는 순간(tl_report_d) 정지선 앞에 설 수 있는 속도까지 미리 줄인다.
        rf_objs: [(ds, d, speed, length, width)] 경로기준 투영 객체(권장).
                 ⚠️ 곡선에서 ego좌표 fy는 곡률에 오염되므로 rf_objs를 쓰는 것이 정확하다.
        반환: (v_cmd[m/s], lane_offset[m], turn_signal, reason)
        """
        self._caps = [(self.speed_limit, "SPEED_LIMIT"), (v_prof, "PROFILE")]
        v = min(self.speed_limit, v_prof)
        reason = "LANE_KEEP"
        turn = TS_OFF
        offset = lane_offset

        if rf_objs is None:
            rf_objs = []
            for o in ego.objects:
                fx, fy = to_ego_frame(ego, o)
                rf_objs.append((fx, fy, o.speed, o.length, o.width, o.height))

        # ★사람 앞 정지거리 계산에 쓸 **직선** 전방거리(id -> ego 기준 fx).
        #   rf_objs 의 ds 는 경로를 따라 잰 호 길이라 직선거리보다 길다 — 곡선에서
        #   낙관적이다. 사람 앞에서만은 둘 중 **짧은 쪽**을 쓴다(공짜인 보수적 선택).
        # ★★단, **직선 횡거리도 가까울 때만**(ped_watch+STRAIGHT_LAT) 쓴다(2026-09-06 코스 G 미완주).
        #   (1320,159) 우회전 직후: 오른쪽 62m 에 서 있는 0.2x0.4x1.4 물체(볼라드류)가 경로가
        #   77m 뒤에 그 옆(1.6m)을 지나가서 rf 횡 1.6m 로 투영됐고, 정지거리는 직선 fx=9.9m 를
        #   써서 '코앞 6m 의 사람' 이 됐다 -> YIELD_PED 완전정지 **248초**, 종점 127m 남기고
        #   미완주. 사용자: "사람도 없는데 뭘 보고 이러는 건지". 경로 옆 1.6m 는 77m 뒤의
        #   얘기고 지금 옆 62m 는 아무 위협도 아니다 — 두 좌표계를 섞으면 안 된다.
        straight = {}
        for o in getattr(ego, "objects", ()) or ():
            oid_ = getattr(o, "id", None)
            if oid_ is not None:
                _sfx, _sfy = to_ego_frame(ego, o)
                if abs(_sfy) <= self.ped_watch + self.STRAIGHT_LAT:
                    straight[oid_] = _sfx

        # 보행자 횡속도 추정용 프레임 간격. ego.t 가 안 움직이면(테스트·모의) 공칭값을 쓴다.
        _now = getattr(ego, "t", 0.0) or 0.0
        _el = _now - self._ped_t if self._ped_t is not None else 0.0
        dt_f = _el if 0.0 < _el < 0.5 else self.nominal_dt
        self._ped_t = _now

        # ★요레이트(자차가 도는 속도). 회전 중 CPA 보정(_turn_cut_conflict)이 쓴다.
        #   heading 은 [-pi,pi] 로 감기므로 차이를 반드시 정규화한다.
        _h = getattr(ego, "heading", 0.0) or 0.0
        if self._prev_h is not None and dt_f > 1e-3:
            _dh = (_h - self._prev_h + math.pi) % (2 * math.pi) - math.pi
            _w = _dh / dt_f
            if abs(_w) < 3.0:                      # 리스폰·튐은 버린다
                self._yaw_rps += 0.35 * (_w - self._yaw_rps)
        self._prev_h = _h
        self._purge_stale_tracks(rf_objs, _now)

        for ob in rf_objs:
            fx, fy, ospeed, olen, owid = ob[:5]
            ohgt = ob[5] if len(ob) > 5 else 1.7
            if fx <= 0 or fx > self.scan:
                continue
            # ★사람/사물 구분은 '높이'로 (2026-08-14 실측): 보행자 H≈1.7m.
            #   L0.15×W0.46×**H0.61** 정지물(라바콘)을 보행자로 보고 영구정지 -> 시간초과 위험이었음.
            small = (max(olen, owid) < 1.2)
            # ★**자전거·이륜차도 보호 대상이다.** 실측 2026-08-24 주변교통 판:
            #   2.0x0.6x1.7m 자전거를 `max(L,W)=2.0 > 1.2` 라 **승용차로 취급**해
            #   옆여유를 0.35m(차량)만 두고 30.8km/h 로 스쳐 **실여유 -0.20m 접촉**했다.
            #   사람이었으면 1.2m 를 뒀을 자리다.
            #   ⚠️ 판정에는 **원본 폭**을 쓴다 — 경로축 외접상자(owid)는 비스듬한 자전거를
            #      0.6m -> 1.8m 로 부풀려 승용차와 구분이 안 된다.
            raw_w = ob[7] if len(ob) > 7 else owid
            raw_l = ob[8] if len(ob) > 8 else olen
            # ★판정은 `vtd_io.is_vru` 하나로 한다(그 함수 주석에 카탈로그 근거).
            # ★★속도로도 가르고, 한 번 차량 속도로 달린 id 는 기억한다(2026-09-05).
            #   실측 코스 A t=465~478: 62km/h 이륜차가 앞에서 2km/h 까지 줄었다가 다시
            #   50 으로 — 치수만 보면 '사람' 이라 YIELD_PED(앞범퍼 5m 정지)가 걸리고,
            #   느려질 때마다 사람/차량이 뒤집혀 그게 곧 12초 요동이었다.
            _oid = ob[6] if len(ob) > 6 else None
            if ospeed > VRU_MAX_SPEED and _oid is not None:
                self._fast_ids.add(_oid)
            is_ped = (_oid not in self._fast_ids) and is_vru(raw_l, raw_w, ohgt, ospeed)
            #  '탄 이륜차' 인가 — 아래 주행중 강등 판정에만 쓴다
            two_wheel = is_ped and not small
            # ★도로를 달리는 이륜차(위 two_wheel_ride_v 주석)는 양보 대상에서 뺀다.
            if two_wheel and not small:
                _dh = ob[9] if len(ob) > 9 else 0.0
                _adh = abs(math.degrees((_dh + math.pi) % (2 * math.pi) - math.pi))
                riding = (ospeed > self.two_wheel_ride_v
                          or _adh < self.two_wheel_par_deg
                          or _adh > 180.0 - self.two_wheel_par_deg)
                if riding and abs(fy - lane_offset) >= self.lane_half + self.two_wheel_out_m:
                    is_ped = False               # 교통이다 — ACC/CPA/차체 안전망이 담당
            if small and not is_ped and ospeed < 0.3:
                continue                       # 낮은 소형 정지물(콘/볼라드) = 무시
            o = type("O", (), {"speed": ospeed, "length": olen, "width": owid})
            # ★끼어들기(cut-in) 조기인지: 차선 경계 밖이라도 '내 차선 쪽으로 좁혀오는' 차량은
            #   미리 앞차로 취급한다. 경계(1.75m)를 넘은 뒤 반응하면 제동거리가 모자람.
            oid = ob[6] if len(ob) > 6 else None
            rel = fy - lane_offset
            cutting_in = False
            lane_ok = True                  # '앞차'로 믿어도 되는 횡판정인가
            if oid is not None and not is_ped and ospeed > 0.5:
                prev = self._prev_d.get(oid)
                #  물리적으로 가능한 횡이동인가(위 lat_jump 주석). 첫 관측은 통과 —
                #  스캔에 처음 들어온 차가 내 차로 안이면 즉시 앞차다.
                step_ok = prev is None or abs(abs(rel) - prev) < self.lat_jump
                # ★끼어듦은 **접근 속도**로 본다(위 cutin_rate 주석). 1cm 차이는 잡음이다.
                if prev is not None and step_ok and dt_f > 1e-3:
                    raw = (prev - abs(rel)) / dt_f          # +면 내 쪽으로 좁혀오는 중
                    sm = self._cutin_v.get(oid)
                    rate = raw if sm is None else 0.5 * sm + 0.5 * raw
                    self._cutin_v[oid] = rate
                    if rate >= self.cutin_rate and abs(rel) < self.cutin_window:
                        self._cutin_n[oid] = self._cutin_n.get(oid, 0) + 1
                    else:
                        self._cutin_n[oid] = 0
                    cutting_in = self._cutin_n.get(oid, 0) >= self.cutin_hold
                elif not step_ok:
                    self._cutin_v[oid] = 0.0                # 투영 점프 프레임은 버린다
                    self._cutin_n[oid] = 0
                #  연속으로 차로 안에 있는 프레임 수. 점프 프레임은 streak 를 끊는다.
                if abs(rel) < self.lane_half and step_ok:
                    self._lane_streak[oid] = self._lane_streak.get(oid, 0) + 1
                else:
                    self._lane_streak[oid] = 0
                lane_ok = (prev is None                      # 처음부터 내 차로
                           or self._lane_streak.get(oid, 0) >= self.lane_streak_n
                           or cutting_in)
                self._prev_d[oid] = abs(rel)
            if is_ped:
                # ★사람까지의 거리는 **앞범퍼 기준 직선거리**로 잰다(뒷축 기준 fx 그대로 쓰면
                #   앞범퍼 3.81m 만큼 낙관적이고, 호 길이는 직선보다 길어 또 낙관적이다).
                sfx = straight.get(oid)
                fwd = min(fx, sfx) if (sfx is not None and sfx > 0.0) else fx
                d_body = fwd - self.front_overhang
                #  ⚠️ **이미 우리 차선을 지나 멀어지는 중**이면 계속 기다리지 않는다.
                #     (실측: 다 건너간 보행자를 4.5m 창 밖으로 나갈 때까지 기다리다 시간초과)
                leaving = False
                vy_close = 0.0            # +면 내 진로 쪽으로 다가오는 중[m/s]
                if oid is not None:
                    prev_p = self._prev_ped.get(oid)
                    # ★★**'멀어진다'가 곧 '다 건넜다'는 아니다**(2026-09-11 코스 A TRV).
                    #   문턱이 `lane_half + 1.0`(2.6m)이라, 횡단 중인 사람이 우리 차로만 지나면
                    #   바로 풀려 **찔끔 출발**했다. 실측 t=71.3: 사람이 옆 3.0m 인데 출발했고,
                    #   1.2초 뒤 그 사람이 5.0m 에서 멈춰 서자 다시 급정거했다(3.0 -> 0.07km/h).
                    #   사용자: "왜 사람 보고도 멈추지를 않고 찔끔찔끔 움직여".
                    #   차로 하나만큼 더 기다린다(1.3m/s 로 1.2초). `ped_watch`(4.5m) 상한은 그대로라
                    #   다 건너간 사람을 붙잡아 교착되지는 않는다.
                    if (prev_p is not None and abs(fy) > prev_p + 0.005
                            and self.lane_half + self.ped_leave_pad < abs(fy) < self.ped_watch):
                        leaving = True
                    self._prev_ped[oid] = abs(fy)
                    # ★평활계수 0.5 면 정상상태에서 (평활값-현재값)/dt 가 **실제 횡속도**가 된다
                    #   (c = r·dt·(1-α)/α, α=0.5 -> c = r·dt). 미분 잡음 없이 속도를 얻는다.
                    sm = self._ped_fy.get(oid)
                    if sm is not None:
                        vy_close = (sm - abs(fy)) / dt_f
                    self._ped_fy[oid] = abs(fy) if sm is None else sm + 0.5 * (abs(fy) - sm)
                # 자차선 옆 4.5m 안(또는 내 진로 위) = 무조건 양보 대상
                inside = abs(fy) < self.ped_watch or abs(rel) < self.lane_half + 0.5
                # ★**내 진로 쪽으로 다가오는 사람**이면 아직 멀어도 미리 줄인다.
                #   서는 게 아니라 접근속도만 깎는다 — 창에 들어왔을 때 설 수 있게.
                #   폭은 고정(15m)이다. 자차 속도로 줄이면 위 ⚠️ 의 자기모순이 된다.
                coming = (ospeed > 0.5 and vy_close > self.ped_close_v
                          and abs(fy) < self.ped_caution_lat)
                near = d_body < self.ped_side_near      # 고정 캡은 가까울 때만(위 참고)
                if leaving and near:
                    v = self._cap(v, self.side_pass_v, "PED_LEAVING")   # 멀어지는 중 -> 서행 통과
                    if reason == "LANE_KEEP":
                        reason = "PED_LEAVING"
                elif near and inside and ped_bypass and ospeed < 0.5:
                    # ★서 있는 사람 옆으로 **차선을 바꿔서** 지나가는 중이면 서지 않는다.
                    #   (움직이는 사람=횡단 중은 아래로 빠져 그대로 정지 — 진로가 겹친다)
                    v = self._cap(v, self.ped_pass_v, "PED_LANECHANGE")
                    reason = "PED_LANECHANGE"
                elif (near and inside and ospeed < 0.5
                      and (abs(rel) >= self.lane_half + self.ped_still_margin
                           or abs(rel) - self.half_width - owid / 2.0 >= self.ped_aside_clr)):
                    # ★★**가만히 서 있고 내 진로 밖**인 사람은 세워서 될 일이 아니다.
                    #   실측 2026-08-26 코스 E: 옆 2.9m 에 정지한 사람 하나가 **215초**를
                    #   세웠다(차체 여유 +3.63m 인데도). 그 사이 뒤로 정체가 쌓였다.
                    #   구조적 모순이었다 — 정지 판정은 `ped_watch`(4.5m)까지 거는데
                    #   **우회 검토는 2.5m 까지**라(`drive.py::ped_block`) 그 사이 대역의
                    #   정지 보행자는 **세우기만 하고 비켜갈 검토는 영영 안 된다.**
                    #   제27조는 '횡단하거나 횡단하려는' 보행자 보호다. 멈춰 서 있고
                    #   내 진로 밖이면 **서행 통과**가 맞다.
                    #   ⚠️ 다가오는 사람은 아래 `coming` 과 차체 안전망(사람 1.2m)이 잡고,
                    #      오늘 넣은 횡방향 예측이 움직이기 시작하는 순간 다시 건다.
                    # ★차로 가장자리에 **서 있는** 사람은 실여유(ped_aside_clr)가 있으면 서행 통과한다
                    #   (2026-09-06). 코스 G (1348,103): 경로 옆 1.6m 에 선 1.4m 짜리 물체 — 차로 반폭
                    #   1.75 기준으론 '차로 안' 이라 YIELD_PED 완전정지 -> 움직이지 않는 사람이라 영원히.
                    #   STALL_ESCAPE 도 30초마다 10cm 기다가 다시 잡혀 미완주가 된다. 실여유 0.46m 면
                    #   기어서(ped_edge_pass_v) 지난다. 차로 밖으로 넉넉히 비킨 사람은 예전 속도 그대로.
                    _tight = abs(rel) < self.lane_half + self.ped_still_margin
                    v = self._cap(v, self.ped_edge_pass_v if _tight else self.side_pass_v, "PED_STILL_ASIDE")
                    if reason in ("LANE_KEEP", "SIDE_CAUTION"):
                        reason = "PED_STILL_ASIDE"
                elif inside:
                    v = self._cap(v, self._stop_target_speed(d_body - self.ped_gap), "YIELD_PED")
                    reason = "YIELD_PED"
                elif coming:
                    cap = math.sqrt(2 * self.ped_caution_a * max(0.0, d_body - self.ped_gap))
                    v = self._cap(v, max(self.ped_caution_min, cap), "PED_CAUTION")
                    if reason == "LANE_KEEP":
                        reason = "PED_CAUTION"
            elif ((abs(rel) < self.lane_half
                   #  ★움직이는 차는 lane_ok(연속성)까지 요구한다. 굽은 길의 투영
                   #    점프가 만든 유령 앞차에 급제동하지 않기 위해서다(위 주석).
                   #    정지물(ospeed<0.5)은 그대로 첫 프레임부터 선다.
                   and (ospeed < 0.5 or lane_ok))
                  or cutting_in):
                # ★**정면으로 다가오는 차**(역주행·중앙선 침범)를 '앞차'로 보면 안 된다.
                #   앞차 ACC 는 같은 방향으로 멀어지는 차를 전제한 식이라, 접근속도가
                #   두 속도의 합인 정면 상황에서는 너무 늦게 선다.
                #   실측 2026-08-22 hz_wrongway: 8m/s 로 마주 오는 차를 ACC 로 따라붙어
                #   **6.72m 앞에 정지**했다. 접촉은 없었지만 추월 개시에 필요한
                #   7.008m(front_overhang 3.808 + 차길이/2 2.2 + margin 1.0)에
                #   **0.29m 가 모자라 영구히 OVT:WAIT**(85초 확인, 미완주).
                #   정지 장애물과 같이 obstacle_gap(15m)을 두고 서면 나갈 자리가 남는다.
                #   ⚠️ 여기 `o` 는 경량 객체라 heading 이 없다(2026-08-22 실측: AttributeError).
                #      **접근속도**로 가른다 — 정지물은 접근속도 = 내 속도, 같은 방향
                #      앞차는 그보다 느리다. 내 속도보다 **빠르게** 좁혀오면 마주 오는 것이다.
                head_on, close = False, 0.0
                if oid is not None and ospeed > 0.5:
                    pf = self._prev_fx.get(oid)
                    if pf is not None:
                        close = (pf - fx) / dt_f
                        sm = self._close_sm.get(oid)
                        close = close if sm is None else 0.5 * sm + 0.5 * close
                        self._close_sm[oid] = close
                        head_on = close > ego.speed + self.head_on_margin
                    self._prev_fx[oid] = fx
                if o.speed < 0.5 or head_on:
                    if not avoiding:
                        # 정지 장애물 -> 그 앞에 정지. 간격을 넉넉히(15m) 둬야 이후 차선변경
                        # 으로 우회할 공간이 생긴다(8m에서 시작하면 코너 접촉, 2026-08-14 실측).
                        # ★정면 접근이면 남은 거리를 **내 몫**으로 나눠 쓴다.
                        #   상대가 다가오는 만큼 내가 쓸 수 있는 거리는 줄어든다:
                        #   없애야 할 접근거리 중 내가 기여하는 비율 = 내속도/접근속도.
                        #   이걸 안 하면 '15m 앞에서 정지'를 목표로 잡아도 상대가 계속
                        #   다가와 실제로는 8m 에 선다 — 그러면 **원리상 못 빠져나간다**.
                        avail = fx - self.obstacle_gap
                        if head_on and close > 0.1:
                            avail *= max(0.0, ego.speed) / close
                        _name = "HEADON_STOP" if head_on else "OBSTACLE_STOP"
                        # ★★**상한을 쥐고 있을 때만 그렇게 부른다.** 예전엔 무조건
                        #   `reason` 을 덮어써서, 45m 앞 정지물처럼 **아직 감속할 필요도
                        #   없는** 프레임이 `OBSTACLE_STOP` 으로 찍혔다. 로그로 "왜 느리지"
                        #   를 역추적할 때 그게 사람을 헷갈리게 한다 — `cap_by` 는 이미
                        #   이긴 제약을 정확히 내보내는데 `reason` 만 거짓말을 했다.
                        #   ⚠️ `<` 가 아니라 `<=` 다. 이미 0 으로 서 있는데 장애물 목표도
                        #      0 이면 **그 장애물이 나를 잡고 있는 게 맞다**.
                        _target = self._stop_target_speed(avail)
                        _before = v
                        v = self._cap(v, _target, _name)
                        if _target <= _before:
                            reason = _name
                    # avoiding이면 회피기가 옆으로 빼므로 정지 대신 서행(측면 접촉 방지)
                    else:
                        v = self._cap(v, 3.0, "AVOID")
                        reason = "AVOID"
                else:
                    # 앞차: 동적 안전거리 ACC + 급정지 대비 정지가능 안전속도(둘 중 낮은 쪽)
                    safe = ego.speed * self.react + ego.speed ** 2 / (2 * self.a) + self.follow_gap
                    if fx < safe:
                        v = self._cap(v, max(0.0, o.speed), "FOLLOW")
                        reason = "FOLLOW"
                    v = self._cap(v, self._stop_target_speed(fx - self.follow_gap), "FOLLOW_BRAKE")
            elif abs(rel) < self.side_clear and fx < self.side_range and ospeed > 0.5:
                # ★차선 경계 바로 밖에 '걸쳐 있는' 차: 끼어드는 중이거나 막 빠져나간 차다.
                #   실측(2026-08-14 끼어들기 시험): d=2.5m에 있던 차 옆을 30km/h로 지나가
                #   실제 여유가 0.6m밖에 안 됐다. 이런 경우는 속도를 낮춰 통과한다.
                v = self._cap(v, max(self.side_pass_v, ospeed + 1.5), "SIDE_CAUTION")
                if reason == "LANE_KEEP":
                    reason = "SIDE_CAUTION"
            elif (self.lane_half <= abs(rel) < self.adj_watch and 0 < fx < self.adj_range
                  and not is_ped and ego.speed - ospeed > 2.0
                  and fx / max(0.1, ego.speed - ospeed) < self.adj_ttc):
                # ★옆 차선 느린 차에 빠르게 접근 중 -> 나란히 되기 전에 미리 줄인다.
                #   끼어들더라도 이미 느려져 있어야 대응할 수 있다.
                v = self._cap(v, ospeed + self.adj_margin, "ADJ_SLOW")
                if reason == "LANE_KEEP":
                    reason = "ADJ_SLOW"

        # 신호등.
        #  ⚠️ 실측 3가지 함정(2026-08-14):
        #    ① 정지선이 매핑 안 된 신호에서 '서행 통과'하면 그냥 신호위반 -> 모르면 선다.
        #    ② state=UNSET(0)인데 tl_id가 붙어있는 신호가 있음(예: tl80) -> 상태 불명이므로
        #       30km/h로 교차로를 지나가지 말고 감속해서 대비한다.
        #    ③ 황색 판단은 '앞범퍼가 정지선에 닿기까지 거리 vs 최대제동거리'로 해야 한다
        #       (뒷축 기준 거리에서 여유를 뺀 값으로 보면 너무 쉽게 '통과'로 판정됨).
        d_line = None if tl_stop_dist is None else tl_stop_dist - self.front_overhang
        d_brake = ego.speed * self.react + ego.speed ** 2 / (2 * self.a_max)

        # ★신호를 **관찰**한다 — 이 신호에 좌회전 화살표가 있기는 한지 알아내기 위해서다.
        #   실측 2026-08-20 새 코스 E: 마지막 교차로(tl 108)가 RED->GREEN->YELLOW 만 돌고
        #   **좌회전 화살표가 아예 없는데** 코스는 거기서 좌회전을 요구한다. 우리 규칙이
        #   '좌회전은 화살표에서만' 이라 **영원히 기다렸다**(목표 84m 앞에서 미완주).
        #   화살표 없는 교차로의 좌회전 = 비보호 좌회전이고, 녹색에 양보하며 가는 게 맞다.
        tl = ego.tl_id
        if tl is not None and tl > 0:
            if ego.tl_state in (TL_LEFT, TL_GREEN_LEFT):
                self._tl_has_left.add(tl)
            prev = self._tl_prev.get(tl)
            if ego.tl_state == TL_GREEN and prev != TL_GREEN:
                self._tl_green_n[tl] = self._tl_green_n.get(tl, 0) + 1
                self._green_since[tl] = _now          # 이번 녹색 시작
            elif ego.tl_state != TL_GREEN:
                self._green_since.pop(tl, None)
            if ego.tl_state == TL_RED:
                if tl not in self._red_since:
                    self._red_onset_dline[tl] = d_line
                self._red_since.setdefault(tl, _now)
            else:
                self._red_since.pop(tl, None)
                self._red_onset_dline.pop(tl, None)
            self._tl_prev[tl] = ego.tl_state
            if ego.speed < 1.0:
                self._left_wait_since.setdefault(tl, _now)

        # ★★허가는 **그 신호등 것**이다 — 다음 신호등으로 물고 가면 신호위반이다.
        #   실측 2026-08-28 코스 A: tl 181 적신호에서 적법하게 우회전(RIGHT_ON_RED)한 직후,
        #   불과 5m 앞의 **tl 213 이 적색**인데 `_rtor_go` 래치가 살아 있어 그대로 통과했다.
        #     x=1341.4 y=145.7  tl=213/1  [RIGHT_ON_RED]
        #     x=1343.4 y=130.1  tl=0/0    a=+2.00        <- 적신호 통과
        #   래치 해제 조건이 `route_turn >= 0 and d_stop > rt_approach*1.5` 였는데,
        #   **다음 정지선이 코앞이면 그 조건이 영영 안 맞는다.** 신호가 바뀌면 무효로 한다.
        #   ⚠️ 회전 도중 tl_id 가 0(미보고)으로 잠깐 비는 건 해제 사유가 아니다 —
        #      거기서 풀면 교차로 한복판에 다시 선다(제32조).
        #   ⚠️ `_rt_tl is not None` 을 조건에 넣었다가 **구멍이 났다**(실측 2026-08-28
        #      스윕 코스 A): 무신호 교차로에서 선 것은 `_rt_tl=None` 으로 남는데, 그
        #      크레딧이 다음 **신호** 교차로까지 살아남아 tl 130 적신호를 14.8km/h 로
        #      그냥 통과했다(t=260.6~263.0 (1101,-13)->(1093,-4), 그 앞 정지 없음).
        #      신호가 보이는데 그 신호 것이 아닌 크레딧이면 **전부 무효**다.
        if ego.tl_id and ego.tl_id > 0 and ego.tl_id != self._rt_tl:
            self._rtor_go = False
            self._rt_done = False
            self._rt_done_bumper_gap = None
            self._rt_tl = None
            self._rt_done_red = False
            self._rt_red_still = None
            self._rt_green_tl = None

        # ★★**화살표를 받고 들어간 좌회전은 신호 보고가 끊겨도 비보호로 강등하지 않는다.**
        #   실측 2026-08-30 코스 E (1464,900) — 사용자 지적 "정지선에 가까이 가서 멈추지도
        #   않고 혼자 멈췄다가 혼자 빨간불에 좌회전함". 로그가 그대로다:
        #     t=169~173.4 tl=221/GREEN_LEFT  25->23km/h 로 진입
        #     t=173.7     tl=0/UNSET         <- VTD 가 보고를 끊는다
        #     t=175.1     [YIELD_CROSS] a=-5.00  22.2 -> 0.8km/h  <- 교차로 **안**에서 급정지
        #     t=176.5     [NOSIG_LEFT]       <- 혼자 다시 출발
        #   `tl_stop_dist` 가 None 이 되면서 아래 '무신호 좌회전' 가지로 떨어진 것이다.
        #   화살표를 받고 들어간 회전은 대향차 양보 대상이 아니고, 교차로 안에 서는 것
        #   자체가 [법 제32조] 위반이며 뒤차에게도 위험하다.
        #   -> 회전을 마칠 때까지 허가를 물고 있는다(`_rtor_go` 와 같은 이유·같은 방식).
        if route_turn > 0 and ego.tl_state in (TL_LEFT, TL_GREEN_LEFT):
            self._left_go = True
            if ego.tl_id and ego.tl_id > 0:
                self._left_tl = ego.tl_id
        elif (ego.tl_id and ego.tl_id > 0 and self._left_tl is not None
              and ego.tl_id != self._left_tl):
            self._left_go, self._left_tl = False, None      # 다른 신호등 = 다른 교차로
        elif route_turn <= 0 and (junc_dist is None or junc_dist > self.nosig_near * 1.5):
            self._left_go, self._left_tl = False, None      # 교차로를 벗어났다

        # ★★**아직 보고되지 않은 신호**의 정지선이 앞에 있으면, 그 신호가 보일 지점(tl_report_d)에서
        #   RED_STOP 이 정지선 앞에 세울 수 있는 속도(_v_rep)까지 미리 계획감속한다. 보고되면(tl_stop_dist
        #   가 그 선을 가리키면) drive.py 가 None 을 넘겨 이 캡은 바로 풀린다 — 녹색이면 그냥 가속.
        if tl_unseen_dist is not None and tl_unseen_dist > 0.0:
            _rep_axle = self.tl_report_d + self.front_overhang              # 뒷축 기준 보고 지점
            _v_rep = self._stop_target_speed(_rep_axle - self.stop_margin)   # 그때 RED_STOP 이 허용할 속도
            _d_rep = max(tl_unseen_dist - _rep_axle, 0.0)                    # 보고 지점까지 남은 거리
            v = self._cap(v, math.sqrt(_v_rep ** 2 + 2.0 * self.a_plan * _d_rep), "TL_UNSEEN")
            if reason == "LANE_KEEP" and v < min(self.speed_limit, v_prof) - 1e-6:
                reason = "TL_UNSEEN"
        if ego.tl_state != TL_RED:
            self._red_clear_tl = None                   # 적색이 아니면 놓는다
        elif (ego.tl_id and ego.tl_id > 0 and self._red_clear_tl is not None
              and ego.tl_id != self._red_clear_tl):
            self._red_clear_tl = None                   # 다음 신호로 물고 가지 않는다
            # ⚠️ tl_id 0(미보고)은 해제 사유가 아니다 — 교차로 안에서 보고가 끊기는 일이 있다.
        if ego.tl_state == TL_RED:
            # ★적신호 우회전 — 일시정지를 마쳤고 교차 교통이 없으면 간다(위 self.rtor 참조).
            #   ⚠️ 회전에 들어가면 `route_turn` 이 0 이 되므로(_upcoming_turn 의 성질)
            #      **결정을 걸어둔다**. 안 그러면 회전 도중에 조건이 풀려 교차로 한복판에
            #      다시 선다 — 그건 제32조 교차로 내 정차다.
            # ★★적색 우회전의 '일시정지'는 **적색 중에** 한 것만 인정한다(2026-09-06).
            #   실측 코스 E 신호109·G 신호130: 녹색에 선 채 황->적으로 바뀌자 '정지 마쳤다'로
            #   적색 진입 0.4초 만에 출발했다. 심판 기준은 "적색에서 정지선 2.0m 미만 0.5초"라
            #   그건 **정지 없이 통과**다(항목7 중대 -6, 두 판). 황색에 섰다면 여기서 적색
            #   정지를 stop_hold_s 만큼 다시 채운다 — 비용 0.8초.
            if route_turn < 0 and self._rt_done and not self._rt_done_red:
                rtor_stop_position_ok = self._rtor_pos_ok(d_line, ego.tl_id)
                if rtor_stop_position_ok and ego.speed < self.stop_hold_v:
                    if self._rt_red_still is None:
                        self._rt_red_still = _now
                    elif _now - self._rt_red_still >= self.stop_hold_s:
                        self._rt_done_red = True
                        self._rt_done_bumper_gap = d_line
                else:
                    self._rt_red_still = None
            rtor_ok = (route_turn < 0 and self._rt_done and self._rt_done_red
                       and self._rtor_pos_ok(self._rt_done_bumper_gap, ego.tl_id))  # 어디서 섰나
            if self.rtor and (self._rtor_go or rtor_ok):
                # ⚠️ 기준점을 못 만들면(경로 끝 등 `point_at` 이 None) **보수적으로** 본다.
                #   우회전용 완화값(5초·5m)을 **자차 위치**에 그대로 쓰면, 기준점이 없다는
                #   이유로 오히려 판정이 헐거워진다. 그때는 좌회전용 기본값으로 떨어뜨린다.
                _g = self.rtor_gap if rtor_point else None
                _r = self.rtor_radius if rtor_point else None
                if tl_passed or self._rtor_go or self._oncoming_clear(
                        ego, at=rtor_point, gap=_g, radius=_r):
                    self._rtor_go = True
                    if ego.tl_id and ego.tl_id > 0:
                        self._rt_tl = ego.tl_id
                    v = self._cap(v, self.rtor_v, "RIGHT_ON_RED")
                    reason = "RIGHT_ON_RED"
                else:
                    # 신호 따라 오는 차가 있다 -> 정지선에서 계속 대기(방해 금지)
                    v = self._cap(v, 0.0, "RTOR_YIELD")
                    reason = "RTOR_YIELD"
            elif (d_line is not None and d_line < -self.red_clear_pad
                  and (self._red_clear_tl == ego.tl_id or ego.speed >= self.stop_hold_v)):
                # ★★**앞범퍼가 이미 정지선을 넘었는데 굴러가는 중이면 세우지 않는다**
                #   (2026-09-11 사전주행2 신호25). `tl_passed` 는 **뒷축**이 선을 넘어야
                #   참이라, 앞범퍼가 3.8m 앞선 우리 차는 '아직 안 지났다'로 판정돼 그 자리에
                #   섰다 — 그 자리가 **횡단보도 위**였고 녹색까지 18초를 서 있었다(항목12).
                #     t=95.5 우회전 감속 15.5km/h 로 정지선을 지나는 중 황색
                #     t=96.9 횡단보도 위 정지 -> t=97.5 YELLOW_GO 로 재출발
                #     t=98.5 적색. 앞범퍼가 선 3.0m 뒤(d_line -3.03) 인데 RED_STOP
                #     t=99.4~116.5 정지 유지. tl_stop_dist 가 0.07m 만 더 줄었어도
                #                  `tl_passed` 가 참이 돼 빠져나갔을, 칼날 위의 판정이었다.
                #   여기서 서는 것은 [법 제27조①·항목12] 위반이고 [법 제32조] 취지에도 어긋난다.
                #   앞범퍼로 재고, **이미 굴러가고 있을 때만** 빠져나간다 — 선 차를 적색에
                #   출발시키지는 않는다(그건 항목7 -6 이다). 한 번 정하면 그 신호 동안 문다.
                self._red_clear_tl = ego.tl_id
                reason = "RED_CLEARING"
            elif tl_stop_dist is not None:
                v = self._cap(v, self._stop_target_speed(tl_stop_dist - self.stop_margin), "RED_STOP")
                reason = "RED_STOP"
            elif tl_passed:
                # ★정지선을 **이미 지났다** — 여기서 서면 교차로 한복판에 선다.
                #   실측 2026-08-20(비보호 좌회전 판): 황색에 진입해 좌회전하는 중에
                #   적색으로 바뀌자 교차로 안에서 11초를 서 있었다. 위험하고,
                #   현실에서도 교차로 내 정차다. 빠져나가는 게 맞다.
                reason = "RED_CLEARING"
            else:
                v = 0.0                  # 정지선 미상 -> 통과 대신 정지(위반 방지)
                reason = "RED_STOP_BLIND"
        elif ego.tl_state == TL_YELLOW:
            if d_line is not None:
                # ★★이미 서기로 한 신호는 **반응시간 없이** 다시 판정한다(2026-09-06 코스 E 신호121).
                #   d_brake 는 '반응 0.8초 + 제동거리' 라 저속에선 반응거리가 지배한다: 14km/h
                #   에서 5.5m. 그래서 YELLOW_STOP 으로 잘 서던 차가 정지선 4.5m 앞에서 '못 선다'
                #   로 뒤집혀 **재가속**했고(13.8 -> 19.3km/h) 적색으로 바뀐 순간 범퍼가 정지선
                #   위였다 -> 항목7 -6. 이미 제동 중이면 반응할 게 없다 — 제동거리만 본다.
                _committed = self._yellow_stop_tl == tl
                _d_need = (ego.speed ** 2 / (2 * self.a_max)) if _committed else d_brake
                # ★★**서 있던 차는 황색에 출발하지 않는다**(2026-09-11 코스 B TRV 신호173).
                #   [시행규칙 별표2 황색등화] 차마는 정지선·횡단보도 직전에 정지해야 하며,
                #   **이미 교차로에 일부라도 진입한 경우에만** 신속히 빠져나간다.
                #   실측: 녹색 내내 횡단 보행자 때문에 정지선 0.96m 앞에 서 있다가, 보행자가
                #   비켜난 t=60.4 에 출발했고 0.4초 뒤 황색이 됐다. 그때 속도 0.75m/s ·
                #   정지선까지 0.53m 인데 `d_brake` 는 **반응 0.8초**가 들어가 0.66m — 13cm
                #   차이로 '못 선다'가 돼 `YELLOW_GO`. 그대로 적색에 교차로를 건넜다.
                #   사용자: "빨간불에 왜 지나가". 2.7km/h 짜리 차에 딜레마존은 없다.
                _creeping = (ego.speed < self.yellow_go_min_v
                             and (d_line is None or d_line >= 0.0))
                if _creeping or d_line > _d_need:     # 정지선 전에 멈출 수 있으면 반드시 정지
                    self._yellow_stop_tl = tl
                    v = self._cap(v, self._stop_target_speed(tl_stop_dist - self.stop_margin), "YELLOW_STOP")
                    reason = "YELLOW_STOP"
                else:
                    reason = "YELLOW_GO"  # 딜레마존: 이미 못 멈춤 -> 신속 통과
            else:
                v = self._cap(v, 3.0, "YELLOW_SLOW")
                reason = "YELLOW_SLOW"
        elif ego.tl_state == TL_GREEN and route_turn > 0:
            # ★좌회전은 좌회전 신호(화살표)에서만. 직진 녹색에 꺾으면 신호위반이다.
            #   지금까지는 우연히 GREEN_LEFT 에 도착해 통과했을 뿐, 규칙이 없었다(2026-08-15).
            #
            # ★★단, **화살표가 없는 교차로**라면 그건 비보호 좌회전이라 녹색에 가야 한다.
            #   판정: 이 신호에서 녹색을 **두 번** 봤는데(=한 주기를 다 봤는데) 그동안
            #   화살표가 한 번도 안 나왔으면 없는 것이다. 시간 상한(75초)은 안전망이다.
            #   ⚠️ 대향차 양보는 아래 `YIELD_ONCOMING` 가 한다(도교법 제26조). 단
            #      **아직 설 수 있을 때만** — 이미 교차로에 들어갔으면 멈추는 게 더 위험하다.
            waited = _now - self._left_wait_since.get(tl, _now)
            # ★이번 녹색에서 화살표를 얼마나 기다렸나(위 green_probe 주석).
            green_for = _now - self._green_since.get(tl, _now)
            if (tl not in self._tl_has_left
                    and (green_for > self.green_probe
                         or self._tl_green_n.get(tl, 0) >= 2
                         or waited > self.left_arrow_wait)):
                # ★마주오는 직진차가 우선이다(도교법 제26조). 단 **아직 설 수 있을 때만**
                #   기다린다 — 이미 교차로에 들어갔으면 멈추는 게 더 위험하다.
                can_stop = d_line is not None and d_line > d_brake
                if can_stop and not self._oncoming_clear(ego):
                    v = self._cap(v, self._stop_target_speed(tl_stop_dist - self.stop_margin),
                                  "YIELD_ONCOMING")
                    reason = "YIELD_ONCOMING"
                else:
                    v = self._cap(v, self.unprotected_left_v, "UNPROTECTED_LEFT")
                    reason = "UNPROTECTED_LEFT"
            elif d_line is not None and d_line > d_brake:
                v = self._cap(v, self._stop_target_speed(tl_stop_dist - self.stop_margin), "WAIT_LEFT_ARROW")
                reason = "WAIT_LEFT_ARROW"
            elif d_line is None:
                v = 0.0
                reason = "WAIT_LEFT_BLIND"
            else:
                reason = "LEFT_DILEMMA"       # 이미 못 멈춤 -> 신속 통과
        elif ego.tl_state == TL_LEFT and route_turn <= 0:
            # 좌회전 화살표만 켜진 상태 -> 직진은 대기
            if d_line is not None and d_line > d_brake:
                v = self._cap(v, self._stop_target_speed(tl_stop_dist - self.stop_margin), "WAIT_GREEN")
                reason = "WAIT_GREEN"
        elif ego.tl_state == TL_FLASH and tl not in self._flash_done:
            # ★점멸 신호. 2026-08-20 실측: 코스 A 의 tl 117 을 **30.0km/h 로 그냥 통과**했다.
            #   TL_FLASH 는 정의만 돼 있고 아무 데서도 안 쓰였다.
            #
            #   도로교통법 시행규칙 별표2
            #     · 적색 점멸: 정지선이나 횡단보도가 있을 때에는 그 직전이나 교차로의 직전에
            #                  **일시정지한 후** 다른 교통에 주의하면서 진행할 수 있다.
            #     · 황색 점멸: 다른 교통 또는 안전표지의 표시에 주의하면서 진행할 수 있다.
            #
            #   ⚠️ 9910 패킷은 **점멸의 색을 안 준다**(state 6 하나뿐). 지도로도 못 가른다 —
            #      전 신호가 type/크기가 같다(2026-08-20 확인: 1000008/1000012/1000020,
            #      전부 0.40x0.41m). 그래서 **서는 쪽**을 택한다. 서면 적색 점멸도 만족하고,
            #      황색 점멸이라도 '주의하며 진행'을 어기는 게 아니다. 반대로 안 서면
            #      적색 점멸일 때 그대로 위반이다.
            if (d_line is not None and 0.0 <= d_line < self.flash_near
                    and ego.speed < self.stop_hold_v):
                # ★**0.5초 이상** 서 있어야 인정한다(안내문 항목 9). 순간 속도만 보면
                #   감속 곡선의 최저점을 스치고 지나가도 '섰다'가 된다.
                #   문턱은 심판보다 엄하게 — 위 `stop_hold_v`/`stop_hold_s` 주석 참조.
                t0 = self._flash_since.setdefault(tl, _now)
                if _now - t0 >= self.stop_hold_s:
                    self._flash_done.add(tl)      # 섰다 -> 이 신호는 통과 허용
                else:
                    v = self._cap(v, 0.0, "FLASH_STOP")
                    reason = "FLASH_STOP"
            elif d_line is not None:
                # 유효 구간을 벗어났거나 다시 움직이면 처음부터 쌓는다.
                self._flash_since.pop(tl, None)
                v = self._cap(v, self._stop_target_speed(tl_stop_dist - self.stop_margin),
                              "FLASH_STOP")
                reason = "FLASH_STOP"
            else:
                v = self._cap(v, 3.0, "FLASH_BLIND")   # 정지선 미상 -> 서행
                reason = "FLASH_BLIND"
        elif (tl_stop_dist is None and route_turn > 0 and not self._left_go
              and junc_dist is not None
              and self.nosig_min_arm < junc_dist < self.nosig_near):
            # ★★**교통정리가 없는 교차로에서 좌회전 — 직진차가 우선이다** [법 제26조④].
            #   여기엔 규칙이 아예 없었다. 신호 있는 비보호 좌회전(`YIELD_ONCOMING`)만
            #   있었고, 그건 `tl_state == GREEN` 일 때만 도는 가지다.
            #   실측 2026-08-26 (1457,938) — **신호 없는 교차로(tl 0/UNSET)에서 73° 좌회전**:
            #     코스 E t=160.4  왼쪽 21m 에 47km/h 로 가로지르는 차 -> **가속**해서 진입
            #                     t=162.2 실여유 **-0.65m** 접촉
            #     코스 H t=307    같은 자리, 58km/h 짜리에 **-0.19m** 접촉
            #   두 코스가 **같은 교차로에서 같은 식으로** 스쳤다. 2.4초 전부터 보이는
            #   차였는데 `LANE_KEEP/PROFILE` 로 25km/h 까지 가속했다.
            #   ⚠️ 기준점은 **자차가 아니라 회전을 마친 뒤 있을 자리**(rtor_point)다.
            #      자차로 재면 위 실측에서 CPA 12.1m 로 안 걸린다(회전 뒤 자리로 재면 3.3m).
            #      뒤차·옆으로 지나가는 차를 상대로 잡지 않으려는 이유도 같다.
            #   ⚠️ 영구 대기 방지: `nosig_wait` 를 넘기면 서행으로 진입한다.
            if self._oncoming_clear(ego, at=rtor_point, gap=self.nosig_gap,
                                    radius=self.nosig_radius):
                self._nosig_since = None
            elif self._nosig_since is None:
                self._nosig_since = _now
            waited = 0.0 if self._nosig_since is None else _now - self._nosig_since
            if self._nosig_since is not None and waited < self.nosig_wait:
                stop_dist = junc_dist - self.stop_margin
                if yield_stop_dist is not None:
                    stop_dist = min(stop_dist, yield_stop_dist)
                v = self._cap(v, self._stop_target_speed(stop_dist),
                              "YIELD_CROSS")
                reason = "YIELD_CROSS"
            else:
                # 비었거나 충분히 기다렸다 -> 서행으로 지난다(가속 금지).
                v = self._cap(v, self.unprotected_left_v, "NOSIG_LEFT")
                if reason in ("LANE_KEEP", "SIDE_CAUTION"):
                    reason = "NOSIG_LEFT"

        # ★우회전 일시정지 — 신호 판단과 별개로 적용한다. **단 녹색 신호는 예외**(아래).
        d_stop = tl_stop_dist if tl_stop_dist is not None else junc_dist
        # ★★녹색(직진녹·좌회전동시)에는 서지 않고 **서행으로 돈다**(2026-09-06).
        #   예전엔 "녹색이어도 선다"였다. 실측 코스 E 신호109 · G 신호130:
        #     녹색에 3.6초 일시정지 -> 서 있는 동안 황->적 -> '정지 마쳤다'로
        #     적색 진입 0.4초 만에 RIGHT_ON_RED 출발
        #   심판 기준(적색에서 정지선 2.0m 미만 0.5초)에 못 미쳐 **항목7 중대 -6 이 두 판**.
        #   녹색 30m 내 정차는 항목8 위험이기도 하다. 법도 녹색 우회전에 정지를 요구하지
        #   않는다(적색일 때만 정지선 일시정지). 보행자는 YIELD_PED·CROSSWALK_STOP 이
        #   따로 본다. 사용자 지적 ①(2026-09-05 "초록불에서 우회전할 때는 정지 안 해도")
        #   이 맞았다 — 나는 "24초 절약뿐"이라 유지를 권했고 그 값이 -12점이었다.
        rt_green = (tl_stop_dist is not None
                    and ego.tl_state in (TL_GREEN, TL_GREEN_LEFT))
        if (rt_green and route_turn < 0 and d_stop is not None
                and d_stop < self.rt_approach):
            self._rt_armed = False                  # 접근 중 녹색으로 바뀌면 정지를 접는다
            self._rt_still_since = None
            self._rt_green_tl = ego.tl_id           # ★이 회전은 녹색 통과 — 아래 래치
            v = self._cap(v, self.rt_green_v, "RIGHT_TURN_SLOW")
            if reason in ("LANE_KEEP", "SIDE_CAUTION"):
                reason = "RIGHT_TURN_SLOW"
        # ★★녹색으로 서행 진입한 우회전은 **정지선을 지나 신호 보고가 끊겨도**(tl_id 0) 다시
        #   무장하지 않는다. 실측 2026-09-06 코스 G 신호198: RIGHT_TURN_SLOW 15km/h 로 정지선을
        #   지나자 tl_id=0 -> '무신호 우회전'으로 보고 교차로 안에서 RIGHT_TURN_STOP 급정지.
        #   사용자: "정지선 지나서 멈추던데 왜??". 새 신호가 보고되면 래치는 풀린다.
        rt_skip = (self._rt_green_tl is not None
                   and ego.tl_id in (0, self._rt_green_tl))
        # 무장: 우회전이 앞에 있고 정지점이 사정거리 안이면(녹색 신호·그 직후는 제외).
        if (route_turn < 0 and not rt_green and not rt_skip
                and not self._rt_armed and not self._rt_done):
            # ★★정지점이 **0m** 이면 무장하지 않는다 — 그건 '여기서 서라'가 아니라
            #   **이미 교차로 안**이라는 뜻이다(`_junction_ahead` 는 교차로 안에서 0.0 을 준다).
            #   실측 2026-08-28 코스 A 남행 (1339,157): 우회전은 35m 앞 **다음 교차로**
            #   (1343,124) 인데, 지금 있는 교차로 때문에 junc_dist=0.0 → 그 자리에서
            #   `_stop_target_speed(-6)`=0 → **29.9km/h 에서 a=-5.00 급정지**.
            #   게다가 선 자리가 교차로 한복판이라 [법 제32조] 위반이다.
            #     x=1339.2 y=157.5 a=-5.00 [RIGHT_TURN_STOP]
            #     x=1340.0 y=152.0 v=8.3   [RIGHT_TURN_STOP]   <- j=1 구간
            #   교차로를 빠져나오면(j=0) junc_dist 가 18.7m 로 살아나 제때 무장한다.
            if d_stop is not None and self.rt_min_arm < d_stop < self.rt_approach:
                self._rt_armed = True
        # ⚠️ **무장되면 route_turn 이 0 이 돼도 풀지 않는다.** _upcoming_turn 은 전방을
        #    내다봐 꺾임을 찾으므로 **회전에 진입하는 순간 0 이 된다**. 거기서 풀면
        #    서다 말고 그냥 통과한다 — 실측 2026-08-16 9경유지: 12회 중 2회가
        #    30->15km/h 로 줄이다가 재가속해 안 섰다(t=63s, t=98s).
        if self._rt_armed:
            # ★★**한 프레임 스친 것은 정지가 아니다.** 예전엔 `speed < rt_stop_v` 가
            #   참인 첫 프레임에 바로 풀었다 — 감속 곡선의 최저점을 스치기만 해도
            #   '섰다' 가 된다. 실측 2026-08-30 코스 E 신호 200: 1km/h 이하가
            #   **0.40초**밖에 안 돼 안내문 항목 7(0.5초)에 못 미쳤다.
            # 적신호 우회전 허가용 정지는 앞범퍼가 정지선을 넘지 않은
            # 2m 미만 구간에서만 시간을 쌓는다. 먼 곳의 정지는 `_rt_done`을
            # 만들지 않으므로, 정지선 앞으로 이동한 뒤 다시 정지할 수 있다.
            rtor_stop_position_ok = (ego.tl_state != TL_RED
                                     or self._rtor_pos_ok(d_line, ego.tl_id))
            if rtor_stop_position_ok and ego.speed < self.stop_hold_v:
                if self._rt_still_since is None:
                    self._rt_still_since = _now
            else:
                self._rt_still_since = None
            if (self._rt_still_since is not None
                    and _now - self._rt_still_since >= self.stop_hold_s):
                self._rt_armed, self._rt_done = False, True     # 섰다 -> 통과 허용
                self._rt_still_since = None
                self._rt_done_bumper_gap = d_line               # **앞범퍼 기준** 어디서 섰나
                # ★적색 중에 선 것인가(RTOR 조건). **정지를 채운 stop_hold_s 가 통째로 적색**이어야
                #   한다. 실측 2026-09-09 코스 A 신호100: 황색에 서서 0.7초, 적색 0.1초에 정지가
                #   '완료' -> 그 순간 적색이라 적색 정지로 인정 -> 0.1초 뒤 RIGHT_ON_RED.
                #   채점은 "적색에서 0.5초 정지" 라 -6. 적색이 시작된 시각을 보고 가른다.
                _red_t = self._red_since.get(ego.tl_id) if (ego.tl_id and ego.tl_id > 0) else None
                self._rt_done_red = (ego.tl_state == TL_RED and _red_t is not None
                                     and _now - _red_t >= self.stop_hold_s)
                self._rt_red_still = None
                # 무신호 교차로에서 선 것이면 None — 나중에 엉뚱한 신호와 비교되지 않게.
                self._rt_tl = ego.tl_id if (ego.tl_id and ego.tl_id > 0) else None
            else:
                v = self._cap(v, self._stop_target_speed(
                    (d_stop if d_stop is not None else 0.0) - self.stop_margin), "RIGHT_TURN_STOP")
                reason = "RIGHT_TURN_STOP"
        elif route_turn >= 0 and (d_stop is None or d_stop > self.rt_approach * 1.5):
            self._rt_done = False                   # 교차로를 벗어남 -> 다음 우회전 대비
            self._rt_still_since = None
            self._rtor_go = False
            self._rt_done_bumper_gap = None
            self._rt_tl = None
            self._rt_done_red = False
            self._rt_red_still = None
            self._rt_green_tl = None

        if (ego.tl_state == TL_UNSET and ego.tl_id > 0
                and d_line is not None and d_line < self.unknown_slow_dist):
            # 상태를 못 받은 신호 -> 적색일 수 있으니 정지선 앞에서 기어간다.
            #
            # ⚠️ **정지선을 알 때만** 그런다. 원래는 `d_line is None` 도 감속시켰는데,
            #    그게 이번 판 유령 브레이크의 원인이었다(2026-08-16 9경유지 실측):
            #    VTD 가 `tl_id=80`(항상 state=0)을 두 번, 합쳐 **52초** 보냈고 우리
            #    정지선 DB 214개 어디에도 없는 번호라 d_line 이 계속 None —
            #    (684,-538)~(753,-625) 110m 를 아무것도 없는데 13.5km/h 로 기었다.
            #    맵에 controller 80 도, 그 자리 근처의 signal 80 도 없다(signal 80 은
            #    (412,-196), 270m 밖). 즉 **정지선이 없는 신호**다. 정지선을 모르면
            #    설 자리도 없으니 기어봐야 시간만 버린다. 실제 신호는 전부 state 를
            #    같이 보내왔다(25·30·38·48·53·58·90·312 — 전부 DB 에 있고 state≠0).
            v = self._cap(v, self.unknown_slow_v, "TL_UNKNOWN_SLOW")
            reason = "TL_UNKNOWN_SLOW"

        # ★최후 안전망: 차선 부기와 무관하게 '내 차체가 지나갈 통로'만 본다.
        #   위 판정들이 오프셋 계산으로 어긋나도 여기서 막힌다(EV_CUTIN 옆구리 접촉 -0.98m).
        lo, hi = self.body_watch
        for ob in (rf_objs or []):
            fx, fy, ospeed, olen, owid, ohgt = ob[0], ob[1], ob[2], ob[3], ob[4], ob[5]
            if not (lo < fx < hi):
                continue
            # 차량과 **사람**만 본다. 낮은 소형물(콘·연료통)은 drive.py 의 nudge 가 비켜가므로
            # 여기서 세우면 연료통마다 정지한다.
            #  ⚠️ 사람을 빼놨다가, 옆으로 비켜 지나가는 도중 오프셋이 덜 벌어진 채
            #     실여유 0.11m 로 스쳐 지나갔다(2026-08-15 EV_PED 실측). 사람은 반드시 포함.
            rw = ob[7] if len(ob) > 7 else owid
            rl = ob[8] if len(ob) > 8 else olen
            is_vru_ = is_vru(rl, rw, ohgt)
            if max(olen, owid) < 1.2 and not is_vru_:
                continue
            # ★앞에서 **나보다 빨리 달아나는** 물체는 좁은 통로가 아니다(2026-09-06 코스 H).
            #   실측 (983,667): 10m 앞 우측 2.4m 의 이륜차(42km/h, 우리 35km/h)를
            #   '옆 여유 부족'으로 봐 35->6km/h 급제동(a=-5.0). 따라잡을 수 없는 물체는
            #   옆구리를 스칠 일이 없다. 사용자: "옆에 오토바이 지나가는 거 보고 쫄아서
            #   직진도 못함". 뒤에서 오거나(fx<0) 이미 옆에 있는 것은 그대로 본다 —
            #   코스 E 의 끼어든 이륜차(fx -3.7 -> 0.6)는 이 제외에 걸리지 않는다.
            if fx - olen / 2.0 > self.front_overhang:
                _vx = ospeed * math.cos(ob[9]) if len(ob) > 9 else ospeed
                if _vx > ego.speed + 0.3:
                    continue
            gap = abs(fy) - self.half_width - owid / 2.0
            lat_gap = gap                                  # 경로 폭 기준(휩쓸고 지나갈 때의) 실여유
            # ★★**서 있는 비스듬한 차**는 상자 대 상자(SAT)로 잰다(2026-09-06 코스 G 사고현장).
            #   경로축 외접상자(owid)는 139° 로 선 4.4m 차의 옆폭을 4.3m 로 부풀려, 옆 차로에
            #   서 있는 차를 '통로 -0.3m' 로 봤다 — 실여유는 3.0m. 33초 완전정지 뒤 절박 추월로
            #   나가 리스폰 두 번. 사용자: "사고난 건 옆차선인데 왜 너가 안 가고 멈춤???"
            #   비스듬한(|sin|>0.3) 정지물에만 쓴다 — 정렬된 차·움직이는 차는 예전 그대로
            #   (예측·좁혀옴 판정이 외접상자 기준으로 짜여 있고, 뒷모서리 정지 목표도 그렇다).
            if ospeed < 0.5 and len(ob) > 9 and abs(math.sin(ob[9] - ego_yaw)) > 0.3:
                _c, _sn = math.cos(ego_yaw), math.sin(ego_yaw)
                _rx, _ry = fx - EGO_CX * _c, fy - EGO_CX * _sn
                gap = max(gap, oriented_gap(_rx * _c + _ry * _sn, -_rx * _sn + _ry * _c,
                                            ob[9] - ego_yaw, rl / 2.0, rw / 2.0))
                # ★★그런데 SAT 는 **지금 이 순간** 상자 둘이 떨어져 있는가다 — 앞에 있는 물체는
                #   종방향 거리만으로 늘 '여유 있음'이 나온다. 실측 2026-09-09 코스 E (917,211)
                #   사고현장: 8.4m 앞·우 2.1m 에 118° 로 선 차(경로 폭 4.98m, 우리 차로를 1.3m
                #   침범). SAT 여유 2.72 -> 0.70(287.1s) -> 0.11 -> **-0.40(287.5s)**. 10km/h 로
                #   0.4초 만에 닿았다(항목14 -6, 채점 -0.22m). 사용자: "왜 얼굴부터 들이미냐".
                #   경로 폭으로 잰 침범이 body_sweep_block 보다 깊으면 지나갈 때 **반드시** 닿는
                #   것이니 SAT 로 덮지 않는다 — 옛 식대로 뒷모서리 앞 body_stop_gap 에 선다.
                #   G(2026-09-06)의 139° 차는 경로 폭 침범 0.26m 였다(문턱 아래) -> 그대로 SAT.
                if lat_gap < -self.body_sweep_block:
                    gap = lat_gap
            # ★차선을 바꿔 **서 있는** 사람·자전거 옆을 지나기로 한 때(ped_bypass)는 사람 여유(1.2m)
            #   대신 물건 여유(0.35m)로 잰다. 1.2m 는 '움직일 수 있는 사람' 몫인데, 그걸 옆으로
            #   빠지는 도중에도 요구하면 `_sweep_ok` ②가 늘 거짓이라 NARROW_BLOCK 으로 굳는다
            #   (2026-09-09 모의 accident_junction: +3.19 로 비키다 (44.1,0.9) 에서 정지, 자전거 옆
            #   계획 여유 0.5m). 움직이는 사람은 그대로 1.2m 다.
            need_gap = (self.body_gap_ped if (is_vru_ and not (ped_bypass and ospeed < 0.5))
                        else self.body_gap_min)
            # ★**좁혀오는 중이면 미리 본다.** 지금 여유만 보면 반응이 늦는다.
            #   내가 그 물체에 닿을 때까지의 시간(또는 상한 pred_horizon) 동안
            #   지금 속도로 계속 좁혀온다고 보고 그때 여유를 쓴다.
            #   ⚠️ **좁혀올 때만** 쓴다(벌어지는 쪽은 무시) — 예측으로 판정을 **느슨하게
            #      만들지 않는다.** 관측이 불확실하면 보수적으로.
            gap_use = gap
            oid_h = ob[6] if len(ob) > 6 else None
            if oid_h is not None:
                prev = self._lat_hist.get(oid_h)
                self._lat_hist[oid_h] = (abs(fy), _now)
                if prev is not None:
                    dt_h = _now - prev[1]
                    if 1e-3 < dt_h < 1.0:
                        rate = (prev[0] - abs(fy)) / dt_h          # +면 좁혀오는 중
                        # ★★굽은 길 투영 점프 폐기(위 lat_jump 주석과 같은 근거).
                        #   실제 차의 횡속도는 ~3m/s 가 한계다. 프레임 사이에 그보다
                        #   훨씬 빠르게(>6m/s) '순간이동'한 횡값은 측정이 아니라 투영
                        #   잡음이므로 **움직이는 차에 한해** 이번 프레임은 버린다.
                        #   ⚠️ 정지물은 그대로 본다 — 서 있는 걸 늦게 보는 쪽이 더
                        #      위험하고, 정지물의 횡점프는 어차피 ego 투영 문제라
                        #      다음 프레임에도 반복되면 결국 선다(안전 방향).
                        #   ⚠️ streak 요구는 여기엔 안 건다 — 이 안전망이 잡은
                        #      EV_CUTIN(차로 밖에서 좁혀와 옆구리 스침)은 차로 밖
                        #      이력이라 streak 를 걸면 그 회귀가 다시 뚫린다.
                        if ospeed >= 0.5 and abs(rate) > self.lat_jump_rate:
                            continue
                        if rate > self.pred_min_rate:
                            reach_h = max(0.0, fx - self.front_overhang)
                            tta = reach_h / max(ego.speed, 1.0)
                            gap_use = gap - rate * min(self.pred_horizon, tta)
            if gap_use >= need_gap:
                continue
            if gap_use < 0.0:                               # 이대로 가면 닿는다
                # ⚠️ fx 는 뒷축 기준이다. 앞범퍼가 3.81m 앞이므로 그만큼 빼야
                #    '범퍼가 닿기까지 남은 거리'가 된다(정지선 때와 같은 보정).
                reach = fx - self.front_overhang
                # ★★**이미 다 지나친 것은 세워서 피할 수 없다.**
                #   실측 2026-08-25 코스 D 주변교통 판: 옆으로 가로지른 차와 스친 뒤,
                #   이륜차 한 대가 **fx=-0.1m(뒷축 옆), 옆 1.4m** 에 와서 **멈췄다**.
                #   앞범퍼는 그것을 이미 3.9m 지나쳐 있었는데 gap<0 이라 BLOCK 이 걸렸고,
                #   `_stop_target_speed(-4.9)` = 0 이라 **1597프레임(64초) 완전 정지**로
                #   주행이 죽었다(x·y 이동량 0.000m). 상대 속도 0 · 자차 0 이라 영원히 그대로다.
                #
                #   ⚠️ 기준은 `fx`(중심)가 아니라 **물체의 앞끝**이다. 중심으로 자르면
                #      EV_CUTIN 회귀가 깨진다 — 그 판의 차는 중심 fx=3.0 이라 범퍼(3.81)보다
                #      뒤지만, 길이 4.39m 라 **앞끝이 5.2m** 로 우리를 종방향으로 물고 있다.
                #      그건 전진하면 옆구리를 긁는다. 반대로 앞끝이 범퍼보다 뒤면
                #      **전진해야 멀어진다** — 서면 붙어 있을 뿐이다.
                if fx + olen / 2.0 <= self.front_overhang:
                    v = self._cap(v, self.body_crawl, "NARROW_PASS")
                    if reason in ("LANE_KEEP", "SIDE_CAUTION"):
                        reason = "NARROW_PASS"
                    continue
                # ⚠️ 한때 '우회 중이면 기어서라도 나가게' 해봤다(NARROW_CREEP). 소용없었다 —
                #    8m 뒤에서 2.19m 옆으로 빠지려면 전진이 ~5m 필요한데(최소회전반경
                #    기준) 남은 게 2.1m 뿐이라 **차로는 원리상 불가능**하다. 물체 쪽으로
                #    기어가기만 하고 못 나간다(실측: 8.14 -> 6.9m). 답은 '더 뒤에 서기'다.
                # ★★정지 목표는 물체의 **뒷모서리**다. 중심이 아니다.
                #   실측 2026-08-25 코스 G: 옆으로 추월하던 차가 급정거해 앞우측 6.9m 에
                #   섰다. 그때 자차는 2.8km/h·여유 3.09m 로 **설 수 있었는데**, 목표가
                #   `중심 - 3.81 - 1.0` = 2.09m 전진이라 다시 5.0km/h 로 가속해 파고들었다.
                #   그 차의 뒷모서리는 중심보다 2.3m 앞이라, 그 목표는 **뒷모서리를 1.3m
                #   지나친 지점**이었다 — 정지 프로필이 우리를 옆구리로 밀어넣고 있었다.
                #   결과: clr -0.40 으로 물린 뒤 **503초간 이동량 0.00m**(재현됨).
                #
                # ★같은 변경이 **갇힘도 푼다.** 중심 기준으로 서면 fx ~ 4.8m 인데 추월 FSM 의
                #   최소 간격은 `front_overhang + 길이/2 + pass_margin` ~ 7.0m 다.
                #   즉 너무 붙어 서서 **추월이 영영 시작될 수 없었다.** 뒷모서리 기준으로
                #   서면 fx ~ 7.0m 라 추월이 열린다. README 의 "답은 '더 뒤에 서기'다"가 이것.
                # ⚠️ 여유를 1.0 이 아니라 1.5 로 둔다. 1.0 이면 추월 조건과 **정확히 같은 값**이
                #    되는데, 이 레포는 경계값에 세 번 물렸다(gap_min 0.29 · CW_PAIR 0.3 ·
                #    채점기 횡게이트 0.3). 문턱이 겹치는 자리는 벌려 둔다.
                # ★★**옆으로 빠지는 중이면 '지금 자리'로 세우지 않는다.**
                #   여기 판정은 물체를 **경로축 외접상자**로 보고 종방향 간격을 무시한다
                #   — 똑바로 갈 때는 맞지만, 비스듬히 옆으로 빠지는 중에는 틀린다.
                #   실측 2026-08-26 EV_COMBO_PASSPED: 목표 -6.34m 로 비껴 나가다
                #   -5.41m 에서 이 판정이 0 을 줬다. 그런데 **상자 대 상자 실여유는
                #   +2.93m** 로 닿지도 않았다. 한 번 0 이 되면 후진이 없어 옆으로
                #   마저 못 빠지고 260초 굳는다 — 미완주.
                #   -> 두 가지가 **모두** 참일 때만 완전정지 대신 서행으로 낮춘다:
                #      ① 지금 이 자리에서 **실제로** 안 닿아 있다(SAT ≥ SWEEP_NOW)
                #      ② 계획한 횡이동을 마치면 통로가 열린다(SAT ≥ 필요여유)
                #   ⚠️ 옛 NARROW_CREEP 은 ②가 없어서 **물체 쪽으로 기어가기만 하고
                #      못 나갔다**(2026-08-16 되돌림). ②가 그 재발을 막는다.
                #   ⚠️ 느슨해지는 방향이므로 조건을 못 세우면(방위 정보 없음) 옛 동작 그대로.
                _bypassing = ped_bypass and ospeed < 0.5
                if len(ob) > 9 and self._sweep_ok(ob, ego_yaw, lat_plan, need_gap,
                                                  now_min=need_gap if _bypassing else None):
                    v = self._cap(v, self.body_crawl, "NARROW_SWEEP")
                    if reason in ("LANE_KEEP", "SIDE_CAUTION"):
                        reason = "NARROW_SWEEP"
                    continue
                reach_edge = fx - olen / 2.0 - self.front_overhang
                v = self._cap(v, self._stop_target_speed(reach_edge - self.body_stop_gap),
                              "NARROW_BLOCK")
                reason = "NARROW_BLOCK"
            else:
                v = self._cap(v, self.body_crawl, "NARROW_PASS")
                if reason in ("LANE_KEEP", "SIDE_CAUTION"):
                    reason = "NARROW_PASS"

        # ★★**마지막 겹: 닿을 코스면 우선순위와 무관하게 선다**(CPA 충돌 가드).
        #   차체 안전망(위) 은 '지금 옆에 있는 것'을 보고, 이건 '**곧 여기로 올 것**'을 본다.
        #   실측 2026-08-27 주변교통 A·B·D 접촉 3건이 전부 이 구멍이었다 — 우회전 중 ·
        #   뒤에서 추월해 파고든 차 · 직진 중 횡단이라 어떤 양보 규칙에도 안 걸렸다.
        #   ⚠️ 반경이 실제 차체 크기(+0.5m)라 '지나갈 차'에는 안 걸린다. 옆 차로 병주·
        #      반대차로 대향차는 최근접이 차폭 밖이라 통과한다.
        cpa_x = self._cpa_conflict(ego, rf_objs=rf_objs, dt_f=dt_f, lane_offset=lane_offset)
        if cpa_x is not None:
            v = self._cap(v, self._stop_target_speed(cpa_x - self.front_overhang), "CPA_BRAKE")
            if reason in ("LANE_KEEP", "SIDE_CAUTION", "NOSIG_LEFT", "FOLLOW"):
                reason = "CPA_BRAKE"

        # ★★**회전 중에는 자차를 원호로 굴려 한 번 더 본다.** 위 CPA 는 둘 다 직진으로
        #   보기 때문에 '나란히 가는 차'를 건너뛰는데, 도는 중이면 그 차를 내가 가로지른다.
        #   (1456,939) 무신호 좌회전에서 같은 자리 3회 접촉 — 자세한 근거는 메서드 주석.
        cut = self._turn_cut_conflict(ego, path_ahead=path_ahead)
        if cut is not None:
            v = self._cap(v, self._stop_target_speed(cut - self.front_overhang), "TURN_CUT")
            if reason in ("LANE_KEEP", "SIDE_CAUTION", "NOSIG_LEFT", "FOLLOW", "CPA_BRAKE"):
                reason = "TURN_CUT"

        # ★이긴 제약 = 실제로 속도를 잡은 규칙. reason 과 다를 수 있고, 다를 때가 중요하다.
        self.cap_by = min(self._caps)[1] if self._caps else "SPEED_LIMIT"
        self.reason = reason
        return v, offset, turn, reason


# 차선변경 시 방향지시등은 필수(감점 항목). 회피/차선변경 결정 시:
#   turn = TS_LEFT / TS_RIGHT  를 최소 (변경 전 ~3s) 켜고 offset 적용.
# lattice 기반 회피는 777juwon/rwambangho 레포의 Bezier/cubic 후보 + 충돌비용으로 확장.
