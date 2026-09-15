"""
왕복 2차선 추월 상태기계 (실제 운전처럼).

앞 차선에 정지/저속 차량 -> 뒤에 안전정지 -> 반대(옆)차선에 오는 차 없으면
방향지시등 켜고 반대차선으로 넘어가 추월 -> 다 지나면 원래 차선 복귀.

⚠️ 블라인드 lattice 회피는 차선경계/주행가능영역을 몰라 옆으로 틀면 '인도'로 이탈
   -> VTD 리스폰 반복(2026-08-14 확인). 그래서 여기서는 '차선폭/추월방향'을 명시로 받아
   반대'차선'으로만(도로 위) 넘어간다. 우측통행(RHT) 기본: 추월방향=좌측(+, 반대차선).

상태: FOLLOW -> WAIT(정지·대기) -> PASS(반대차선 주행) -> RETURN(복귀) -> FOLLOW
반환: (lateral_offset[m], turn_signal, state, block)
"""
import math
import time
from vtd_io import is_vru

TS_OFF, TS_LEFT, TS_RIGHT = 0, 1, 2


def to_ego_frame(ego, o):
    """월드 객체 -> 자차좌표(전방 x+, 좌 y+)."""
    dx, dy = o.x - ego.x, o.y - ego.y
    fx = dx * math.cos(-ego.heading) - dy * math.sin(-ego.heading)
    fy = dx * math.sin(-ego.heading) + dy * math.cos(-ego.heading)
    return fx, fy



class Overtaker:
    def __init__(self, lane_width=3.5, side=+1.0, lane_half=1.75,
                 trigger=28.0, scan=90.0, pass_clear=6.0,
                 early_pass=15.0, min_pass_gap=9.0, stopped_hold=2.5,
                 stuck_relax=10.0, front_overhang=3.808, pass_margin=1.0):
        # side +1 = 좌측(반대차선, 우측통행 기본). W = 넘어갈 횡오프셋(좌+ 규약).
        self.W = lane_width * side
        self.lane_half = lane_half
        self.trigger = trigger          # 이 거리 안에 정지차 들어오면 추월 검토
        self.scan = scan
        self.pass_clear = pass_clear     # 이만큼 앞질러야 '지났다'
        self.early_pass = early_pass     # 이보다 멀리서 발견한 '원래 정지물'은 안 서고 미리 우회
        self.min_pass_gap = min_pass_gap # 이보다 가까우면 차선변경 시작 금지(접촉 위험)
        # ★단, 이미 서서 이만큼 기다렸으면 그 규칙을 pass_clear 까지 낮춘다. 9m 규칙은
        #   '달리다가 갑자기 틀지 마라'는 뜻이지 '갇혀 있어라'가 아니다 — 후진을 못 하므로
        #   가까이 붙어버리면 영원히 못 나간다(실측 2026-08-16 EV_COMBO_CHAIN:
        #   7.11m 앞 장애물, 8232프레임 OVT:WAIT, 464초/166m 미완주).
        self.stuck_relax = stuck_relax
        # ★fx 는 **뒷축 기준** 상대차 중심까지의 거리다(9910 egoReferencePoint).
        #   실제 범퍼 간격 = fx - 상대차 반길이 - 앞오버행. 완화 하한은 여기서 나온다.
        self.front_overhang = front_overhang
        self.pass_margin = pass_margin
        self.stopped_hold = stopped_hold # '움직이던 차'가 이 시간 이상 멈춰있어야 우회 검토
        self.front_watch = 70.0          # 목표차선 전방 감시 거리
        self.rear_watch = 50.0           # 목표차선 후방 감시 거리(뒤에서 오는 차)
        self.PASS_ROOM = 20.0            # 추월 구간: 막는 차 앞 이만큼까지가 목표차선에서 실제로 쓰는 길이[m]
        self.was_moving = set()          # 한 번이라도 달린 id = 교통차량(급정거 대상)
        self.still_since = {}            # id -> 멈춘 시각
        self.state = "FOLLOW"
        self.offset = 0.0
        self.turn = TS_OFF
        self.block_id = None
        self._pass_stop_since = None     # PASS 중 멈춰 있은 시각
        # ★객체 목록은 프레임마다 달라질 수 있다. PASS 중 막던 차가 **한 프레임**
        #   안 보였다고 '다 지나갔다'로 읽으면, 아직 옆에 있는 장애물을 가로질러
        #   원차로로 복귀한다. 이만큼 연속으로 안 보여야 사라진 것으로 친다.
        #   (test_lattice 브랜치에서 검증된 처방을 main 으로 가져옴, 2026-08-31)
        self.block_lost_hold = 0.4       # [s]
        self._block_lost_since = None
        self.abort_until = 0.0           # 중단 후 재시도 금지 시각
        self.abort_sec = 4.0             # 나가 있는 채로 이만큼 멈추면 복귀
        self.abort_cooldown = 8.0        # 복귀 후 이만큼은 다시 안 나간다
        self._go_since = None            # 나갈 조건이 갖춰진 시각 — PASS_SIG_LEAD 동안 지시등만 켠다
        # ★죽은 차 앞 마지막 탈출(drive.DEAD_*) 전용 — set_escape 주석. None 이면 평소 값.
        self._esc_half = None
        self._esc_pass_clear = None
        self._esc_pass_room = None

    target_clear = True          # 마지막 plan() 에서 목표차선이 비어 있었나(바깥이 읽는다)
    _wait_since = None           # WAIT 에 들어간 시각(갇힘 완화 판단용)
    _side_want = None            # 바꾸려는 방향(확정 전) — set_side 주석 참고
    _side_since = 0.0

    def _enter_pass(self):
        """PASS 진입. ★`_pass_stop_since` 를 반드시 지운다.

        ⚠️ 안 지우면 **이전 시도에서 남은 시각** 때문에 다음 PASS 가 첫 프레임에
           중단된다(실측 2026-08-16 EV_COMBO_CHAIN: PASS -> 바로 RETURN -> 8초 쿨다운
           -> 다시 대기 -> PASS -> 즉시 취소... 무한 루프). 서 있다가 나가는 상황이
           정상이므로(정지차 뒤에서 출발) 타이머는 여기서 새로 시작해야 한다.
        """
        self.state = "PASS"
        self._pass_stop_since = None
        self._block_lost_since = None
        self._go_since = None

    SIDE_HOLD = 0.8              # 새 방향이 이만큼 이어져야 실제로 바꾼다[s]
    RETURN_DONE_D = 0.5          # 차체가 차로 중심 이 안으로 돌아와야 복귀 완료[m]
    # ★★원래 차로에 **아직 안 지나간 서 있는 사람**이 있으면 돌아가지 않는다(아래 `_vru_in_return_path`).
    RETURN_VRU_AHEAD = 40.0      # 이 앞까지 본다[m] — drive.NUDGE_AHEAD 와 같다
    RETURN_VRU_BEHIND = -3.0     # 사람이 뒤로 이만큼 가면 돌아간다[m] — drive.NUDGE_BEHIND 와 같다
    RETURN_VRU_GAP = 1.2         # 돌아간 자리에서 몸 옆에 이만큼 안 남으면 '막는다'[m] — drive.FIT_GAP_PED
    EGO_HALF_W = 0.943           # 차폭 1.886m 의 절반(drive.HALF_WIDTH)
    RETURN_MAX_S = 15.0          # 이만큼 지나도 못 돌아오면 그냥 FOLLOW 로 넘긴다[s]
    _return_since = None
    PASS_SIG_LEAD = 3.0          # 나가기 전 지시등을 이만큼 먼저 켠다[s] — 대회 항목13 '3초 선행'

    def set_side(self, side, now=None):
        """추월 방향 변경(+1 좌 / -1 우). **기동 중에는 안 바꾼다** — 나가 있는 도중에
        방향이 뒤집히면 장애물을 가로질러 되돌아오게 된다.

        ★★**새 방향이 `SIDE_HOLD` 동안 이어져야 바꾼다.**
          호출부(`drive.py`)는 갇힘을 피하려고 **매 프레임** 좌우를 다시 고른다.
          거기 주석은 "WAIT 중에는 오프셋이 0 이라 차는 안 흔들린다" 인데, 맞다 —
          **차는 안 흔들린다. 깜빡이가 흔들린다.** WAIT 의 지시등은
          `TS_LEFT if W > 0 else TS_RIGHT` 라 `W` 부호가 뒤집히면 같이 뒤집힌다.

          실측 2026-09-05 코스 A t=264.8~289.8 (1389,-161), 적신호 앞 정지 중:
            off·nudge·d_ego 가 전부 0(차는 1cm 도 안 움직임)인데
            깜빡이가 **25초에 7번** L↔R 로 뒤집혔다. 주변 물체 6~7개라
            매 프레임 고르는 쪽이 달라진 것이다. 사용자 지적:
            "왜 직진 차선에서 잘 기다리면서 좌우 깜빡이가 난리가 남?"

          ⚠️ 매 프레임 재평가 자체는 **없애면 안 된다** — 2026-08-25 에 코스 E 가
             한쪽을 배제한 채 250초 갇힌 걸 그걸로 고쳤다. 그래서 재평가는 그대로 두고
             **확정만 늦춘다**. 탈출이 최대 0.8초 늦어지는 건 감수한다.
        """
        if self.state not in ("FOLLOW", "WAIT"):
            self._side_want = None
            return
        want = 1.0 if side > 0 else -1.0
        if want == (1.0 if self.W >= 0 else -1.0):
            self._side_want = None                  # 이미 그쪽이다
            return
        now = time.time() if now is None else now
        if self._side_want != want:
            self._side_want, self._side_since = want, now
            return
        if now - self._side_since >= self.SIDE_HOLD:
            self.W = abs(self.W) * want
            self._side_want = None

    @property
    def escaping(self):
        """죽은 차 앞 탈출 설정이 걸려 있나(바깥이 속도 상한에 쓴다)."""
        return self._esc_half is not None

    def set_escape(self, w, half, pass_clear, pass_room):
        """**죽은 차 앞 마지막 탈출**(사고현장 우회) 설정. FOLLOW/WAIT 에서만 받는다.

        왜 따로 있나(실측 2026-09-09 코스 E (915,216), 65초 교착·미완주):
          자전거(경로 폭 +0.0~+1.75)와 직각으로 선 승용차(-5.4~-1.0)가 내 차로와 우측
          차로를 같이 막았다. 통로는 왼쪽(대향 연결로)뿐이고 필요한 오프셋은 +3.19m.
          그런데 `_target_lane_clear` 는 목표 **차로**(반폭 1.51)가 비었나를 보므로,
          자전거 왼끝(1.75)이 목표차로(1.68~4.70)에 0.07m 걸쳐 '막힘'이 났다. 우리 **차**
          (반폭 0.943)는 안 닿는데도. 그리고 32m 앞 대향 정지차가 PASS_ROOM(20m) 안이라
          또 '막힘'. 사고현장은 좁고 짧다 — 차폭 기준·짧은 구간으로 봐야 빠진다.
          w          : 비킬 횡오프셋(막은 것 끝 + 차 반폭 + 여유)
          half       : 목표 통로 판정 반폭(차 반폭 + 약간) — `lane_half` 대신
          pass_clear : 막은 차 중심이 이만큼 뒤로 가면 복귀(평소 6m 보다 짧게)
          pass_room  : 막은 차 너머 목표 통로에 필요한 빈 길이(평소 PASS_ROOM 20m 보다 짧게)
        ⚠️ 바깥이 **매 프레임** 다시 판단한다 — `set_lane_width`(FOLLOW/WAIT 에서 매 프레임)
           가 이 설정을 지우고, 조건이 여전하면 바깥이 다시 건다. PASS 에 들어가면 복귀할
           때까지 유지된다(기동 중 평소 값으로 돌아가면 그 자리에서 '막힘'이 돼 굳는다).
        """
        if self.state not in ("FOLLOW", "WAIT") or not w or w <= 0.5:
            return False
        self.W = float(w) * (1.0 if self.W >= 0 else -1.0)
        self._esc_half = float(half)
        self._esc_pass_clear = float(pass_clear)
        self._esc_pass_room = float(pass_room)
        return True

    def clear_escape(self):
        self._esc_half = self._esc_pass_clear = self._esc_pass_room = None

    def set_escape_width(self, w):
        """**갇힘 탈출 전용** 비킬 거리[m]. `lane_half` 는 건드리지 않는다.

        `set_lane_width` 를 크게 불러서 대신할 수 없다 — 저건 `lane_half`(내 차로를
        막았나 판정하는 반폭)도 같이 바꾼다. 그걸 부풀리면 '내 차로가 막혔다' 판정이
        통째로 헐거워진다.

        왜 필요한가(실측 2026-09-06 코스 A t=467~656, **188초 이동 0.00m 미완주**):
          13m 짜리 버스가 역주행으로 마주 와 우리 앞 -0.19m 에 섰다. 그 버스는 우리
          차로와 **왼쪽 차로를 같이 물고** 있었다(경로기준 +0.25~+2.75m).
          한 차로치(3.0m) 만 나가는 추월은 목표차선이 그 버스라 영영 `OVT:WAIT` 였고,
          오른쪽은 1.5m 뿐이라 차폭 1.886m 가 못 들어간다.
          빠져나가려면 **버스 너머**까지, 즉 +4.0m 를 나가야 했다.
        ⚠️ 오래 갇혔을 때만 쓴다. 그리고 나가기 전에 `side_clear` 로 그 폭이 실제로
           비어 있는지 반드시 확인한다 — 이 함수는 '거기로 가라'가 아니라 '거기를 봐라'다.
        """
        if self.state in ("FOLLOW", "WAIT") and w and w > 0.5:
            self.W = float(w) * (1.0 if self.W >= 0 else -1.0)

    def set_lane_width(self, w):
        """그 지점 **실제 차로폭**으로 비킬 거리를 맞춘다. 기동 중에는 안 바꾼다.

        ★왜(2026-08-20 실측, 새 코스 G + 정지차): 폭이 생성자 기본값 3.5m 로 고정돼
          있었다. 그 자리 실제 차로폭은 3.03m 이고 좌측 여유가 4.38m 였는데,
          필요폭을 3.5+0.943=4.44m 로 계산해 **갈 수 있는데 못 간다**고 판단했다.
          결과: `OVT:WAIT` 로 54초 갇혀 **미완주(커버리지 20%)**. 리스폰보다 나쁘다.
          비킬 거리도 같은 값을 쓰므로, 좁은 차로에서 3.5m 를 나가면 그만큼 더 벗어난다.
        """
        if self.state in ("FOLLOW", "WAIT"):
            self.clear_escape()                  # 탈출 설정은 프레임마다 바깥이 다시 건다(set_escape 주석)
        if self.state in ("FOLLOW", "WAIT") and w and w > 0.5:
            self.W = float(w) * (1.0 if self.W >= 0 else -1.0)
            # ★★차로 반폭도 같이 맞춘다. 예전엔 **1.75m 고정**이었다. 이 맵의 실제
            #   차로폭은 2.4~3.6m(최빈 3.0~3.2m)라 반폭은 1.5~1.6m — **거의 모든
            #   지점에서 15~25cm 과대평가**였다(3.5m 넘는 지점은 1% 뿐).
            #   그 차이가 목표차선 판정을 뒤집는다. 실측 2026-08-26 EV_BOTHBLOCK:
            #
            #     좌측 목표차선 중심 W=+3.17 · **내 차로의** 정지차 dpath=+0.30 · 경로축 폭 2.4
            #       고정 1.75 -> 안쪽 경계 1.42 : 정지차 왼쪽 끝 1.50 이 넘는다 -> '막힘'
            #       실제 1.58 -> 안쪽 경계 1.59 : 안 넘는다 -> 비어 있다(정답)
            #
            #   **추월하려는 그 차가 또 자기 추월차선을 막았다** — 260초 정지·미완주.
            #   지도가 폭을 아는데 상수를 쓸 이유가 없다.
            self.lane_half = float(w) / 2.0

    def _is_vru(self, row):
        """사람·어린이·휠체어·이륜차인가 — 판정은 `vtd_io.is_vru` **하나**로 한다.

        예전엔 여기에 같은 식이 복사돼 있었고 주석에 "한쪽만 바뀌면 모순" 이라고
        적어두고도 실제로 어긋났다. 원본 치수(index 7,8)를 쓴다 — 경로축 치수로는
        못 가린다(비스듬히 선 사람은 경로축 폭이 2.1m 까지 부푼다).
        """
        olen, owid, ohgt = row[4], row[5], row[6]
        raw_w = row[7] if len(row) > 7 else owid
        raw_l = row[8] if len(row) > 8 else olen
        return is_vru(raw_l, raw_w, ohgt, row[3])          # row[3] = 속도(vtd_io 주석)

    def _vru_in_return_path(self, objs, d_ego=0.0):
        """원래 차로(오프셋 0)로 돌아가면 몸 옆에 `RETURN_VRU_GAP` 도 안 남는 **서 있는 사람**이
        아직 앞에 있나. objs 의 d 는 내 위치 기준이라 d_ego 를 더해 차로 중심 기준으로 본다.

        ★★(2026-09-11 코스 H TRV (1476,512)) 차로 안에 **정지차 → 9m 뒤 휠체어**가 줄지어 있었다.
          추월기가 정지차를 1차로로 넘어가 지나쳤고, 정지차를 뒤로 넘기자(pass_clear) 바로 RETURN —
          그런데 휠체어가 **2.8m 앞**이었다. 사람은 추월 대상이 아니라(`_own_lane_block`) 복귀 판정에
          안 보였다. 휠체어 쪽으로 −33° 꺾으며 2.1m 옆을 스쳤고, 복귀가 끝나자 그 사이 휠체어를 피하려고
          쌓여 있던 nudge(2.27)가 이어받아 **다시 1차로로** 꺾었다. 사용자: "왜 굳이 1차선으로 갔다가
          다시 2로 돌아왔다가 하는거임". 사람을 다 지나간 뒤 한 번에 돌아간다.
        """
        for row in objs:
            _oid, ds, d, spd, olen, owid = row[:6]
            if not (self.RETURN_VRU_BEHIND < ds < self.RETURN_VRU_AHEAD):
                continue
            if spd > 0.5 or not self._is_vru(row):
                continue                       # 걷는 사람은 행동층이 세운다
            if abs(d + d_ego) - self.EGO_HALF_W - owid / 2.0 < self.RETURN_VRU_GAP:
                return True
        return False

    def _own_lane_block(self, objs, d_ego=0.0):
        """**차로 중심 기준** 전방의 정지/저속 '차량'. objs=[(id,ds,d,spd,len,wid,hgt)]

        ⚠️ `d` 는 **내 현재 위치 기준** 상대 횡거리다. 그걸 그대로 쓰면 내가 옆으로 나가
           있을 때 차로 한복판의 차를 '옆 차선 차'로 보고 놓친다. `d_ego`(내가 차로 중심에서
           얼마나 벗어나 있나)를 더해 **차로 중심 기준**으로 되돌려 판정한다.

        실측 2026-08-16 EV_LEADBRAKE: 앞선 회피로 왼쪽 3.35m 에 나가 있었고, 정지차는
        경로 절대 d=+0.00(차로 정중앙)이었다. 상대 기준으로는 3.35m 옆이라 '안 막는다'로
        보고 그대로 차로 중심으로 복귀하다 **들이받았다**(접촉 4376프레임, 200초 정지).
        """
        best = None
        for row in objs:
            oid, fx, fy, spd, olen, owid, ohgt = row[:7]
            if fx <= 0 or fx > self.scan:
                continue
            if max(olen, owid) < 1.2 or spd > 1.0:       # 콘·작은 물건은 추월대상 아님
                continue
            # ★★**사람은 추월 대상이 아니다.** 앞의 `max(길이,폭) < 1.2` 로 거르려 했는데
            #   **VTD 보행자는 2.0 x 0.6 x 1.7** 이라 길이 2.0 에서 새 나갔다.
            #   실측 2026-08-26 코스 A (1391,-184): **교차로 안 신호 횡단보도 위**에 선
            #   보행자를 추월 대상으로 잡고, 20초 갇힘 탈출이 `allow_start` 를 뚫자
            #   8.7m 앞에서 우측 우회를 커밋했다. 옆으로 2.47m 를 벌어야 하는데 남은
            #   전진거리가 2.4m 뿐이라 0.59m 에서 굳었다 — **180초 정지 · 접촉 57프레임**.
            #   교차로 앞지르기(제22조)이자 보행자 옆 파고들기다. 애초에 대상이 아니다.
            #   ⚠️ 자전거도 같은 치수라 함께 빠진다. 그게 맞다 — 이 레포는 이미
            #      "자전거는 '차'지만 스치면 사람이 다친다"로 여유를 사람 기준(1.2m)에
            #      맞춰 뒀다(eval/scenarios/hz_cyclist.json). 달리는 이륜차는 위
            #      `spd > 1.0` 에서 이미 빠지고, 서 있는 것은 behavior 의
            #      PED_STILL_ASIDE / nudge 가 서행으로 지나간다.
            #   판정은 behavior 와 **같은 식**을 쓴다(원본 치수 + 높이).
            if self._is_vru(row):
                continue
            # ★여기도 **폭**을 본다. 비스듬히 선 차는 중심이 차로 밖이어도 몸통이 안이다.
            #   ⚠️⚠️ **폭을 빼면 여유 0.3 은 빼야 한다** — `_target_lane_clear` 와 똑같은
            #      이중계산이다. 0.3 은 '폭을 모르니 이만큼 봐준다'는 대체값이었다.
            #      한쪽만 고치고 이 형제 함수를 놓쳐서 EV_BOTHBLOCK 이 계속 죽었다.
            #      실측 2026-08-26(로그의 rw·dh 로 확인, 추측 아님):
            #        **옆 차로** 정지차 d=-3.16 · 경로축 폭 2.78(dh -14°) · lane_half 1.58
            #          3.16 - 1.39 = 1.77 < 1.58 + 0.3 = 1.88  -> '내 차로를 막는다'(오판)
            #      그 차 뒤에 선 채(OBSTACLE_STOP) 앞지르기도 못 했다 —
            #      `min_pass_gap` 이 그 차와의 거리 6.9m 를 막았기 때문(260초 미완주).
            #      폭을 빼고 나면 경계는 `lane_half` 그 자체다. 옛 판정(중심 기준
            #      `abs < lane_half + 0.3`)보다 여전히 넓게 막는다.
            if abs(fy + d_ego) - max(owid / 2.0, 0.3) < self.lane_half:   # 차로 중심 기준
                if best is None or fx < best[0]:
                    best = (fx, fy, oid)
        return best

    def blocker_was_moving(self, objs, d_ego=0.0):
        """지금 내 차로를 막고 있는 차가 **달리다 멈춘 차**인가.

        달리다 멈춘 차는 **신호를 기다리는 중일 수 있다** — 바깥(drive.py)의 회전
        게이트가 '20초 갇혔으니 나간다'를 적용해도 되는 상대인지 가르는 데 쓴다.
        원래부터 서 있던 것(공사 차량·죽은 NPC)은 기다려도 안 가므로 빨리 풀어야
        하고, 달리다 멈춘 차는 신호 한 주기를 기다려 보는 게 맞다.

        ★판정은 `_own_lane_block` 을 **그대로** 쓴다. 바깥에서 따로 '앞차'를 고르면
          게이트가 A 를 보고 추월기는 B 를 막는 어긋남이 생긴다 — 같은 차를 봐야
          같은 결정을 한다.
        ⚠️ `was_moving` 은 `plan()` 이 갱신한다. 이 함수를 `plan()` **전**에 부르면
           한 프레임 늦은 값이다. 이 판정이 걸리는 문턱은 20초·45초라 무해하다.
        """
        blk = self._own_lane_block(objs, d_ego)
        return blk is not None and blk[2] in self.was_moving

    def side_clear(self, objs, ego_speed, d_ego, side, block_fx=None):
        """**그 방향** 차로가 비었나. 바깥(drive.py)이 좌·우를 각각 물어보려고 쓴다.

        `block_fx` 를 주면 그 너머 PASS_ROOM(탈출 중엔 `_esc_pass_room`) 밖의 **정지물**은
        이번 기동과 무관한 것으로 걸러낸다(`_target_lane_clear` 주석). 안 주면 예전처럼
        전방 감시창 전체를 본다 — 사고현장 우회는 반드시 줘야 한다(2026-09-09 모의 실측:
        막은 것 너머 27m 의 대향 정지차가 '막힘'을 만들어 탈출이 아예 안 열렸다).

        ★왜 필요한가(2026-08-25 코스 E): 예전엔 바깥이 `target_clear`(마지막에 **선택된**
          쪽의 결과)만 보고 방향을 뒤집었다. 그래서 우측을 고른 상태에서 우측이 막히면
          '좌측을 배제하라'는 결론이 매 프레임 되풀이돼 **막힌 쪽에 영구히 묶였다**
          (t=332 에 좌측이 한순간 막힌 걸 보고 우측으로 뒤집은 뒤 250초 정지).
          고르기 전에 **양쪽을 각각** 물어보면 그 순환이 생기지 않는다.
        """
        keep = self.W
        try:
            self.W = abs(self.W) * (1.0 if side > 0 else -1.0)
            return self._target_lane_clear(objs, ego_speed, d_ego, block_fx=block_fx)
        finally:
            self.W = keep

    def _target_lane_clear(self, objs, ego_speed, d_ego=0.0, block_fx=None):
        """추월해 들어갈 차선이 비었나.

        ⚠️ 실측(2026-08-15): 이 코스의 좌측 차선은 '반대차선'이 아니라 **같은 방향 차선**이었다
           (대향차를 놓아도 도로 방향으로 정렬돼 우리와 같은 쪽으로 달림).
           따라서 진짜 위험은 정면충돌이 아니라 **뒤에서 빠르게 오는 차 앞으로 끼어드는 것**.
           기존엔 뒤 8m까지만 봐서 30m 뒤에서 접근하는 차를 놓쳤다 -> 후방 감시 확대.
        전방(대향 가능성 포함)은 넓게, 후방은 '나보다 느리지 않은 차'만 위험으로 본다.
        """
        for row in objs:
            oid, fx, fy, spd, olen, owid, ohgt = row[:7]
            # ⚠️ 크기만 보고 거르면 **사람이 빠진다**(2026-08-15 EV_COMBO_PASSPED 실측:
            #    목표차선에 서 있는 보행자를 못 보고 추월을 시작했다). 낮은 소형물(콘·연료통)만
            #    무시하고, 사람 높이 이상은 크기와 무관하게 차선을 막는 것으로 본다.
            if max(olen, owid) < 1.2 and ohgt < 0.8:
                continue
            # 목표차선도 **차로 중심 기준**이다(내 현재 위치 기준이 아니라).
            # ★★물체의 **폭**을 같이 본다. 중심만 보면 **비스듬히 선 차를 놓친다.**
            #   실측 2026-08-26 코스 H: 원본 L5.2 W1.9 짜리가 약 30° 기울어 서 있어
            #   경로축 폭이 **4.4m**(부풀린 게 아니라 실제 가로폭)인데, 중심이 목표차선
            #   밖(3.6m)이라 '비었다'로 보고 좌측 추월을 커밋했다. 몸통은 목표차선을
            #   덮고 있어 차체 안전망이 0 으로 잡았고 — `OVT:PASS` 인 채 **203초 교착**.
            #   ⚠️⚠️ 폭을 빼면서 **여유 0.5m 를 그대로 두면 이중으로 센다.**
            #      0.5 는 원래 '폭을 모르니 이만큼은 봐준다'는 대체값이었다. 둘을 겹치면
            #      **추월 대상 그 자신이 자기 추월차선을 막는다** — 실측 2026-08-26
            #      EV_ONCOMING/REARPASS/BOTHBLOCK/COMBO_PASSPED: 차로폭 3.16m 자리에서
            #      |(-0.2)-(-3.16)| - 0.9 = 2.06 < 1.75+0.5 = 2.25 (**19cm 차이**)로
            #      '목표차선 막힘'이 나와 WAIT 에서 못 나오고 4판 모두 미완주.
            #      -> 둘 중 **큰 쪽만** 쓴다. 옛 판정보다 절대 느슨해지지 않는다.
            half = self._esc_half if self._esc_half is not None else self.lane_half
            room = self._esc_pass_room if self._esc_pass_room is not None else self.PASS_ROOM
            if abs(fy + d_ego - self.W) - max(owid / 2.0, 0.5) >= half:
                continue
            if 0 <= fx < self.front_watch:                    # ★서 있는 차도 막는다.
                # ★★단 **추월 구간 너머**에 서 있는 차는 이번 기동과 무관하다(2026-09-06).
                #   회귀 hz_blocker_leaves: 막는 차 15m 앞, 목표차선엔 50m 더 앞에 주차된 차.
                #   70m 감시창에 들어온 순간 '막힘'이 돼 영영 못 나갔다(추월은 20m 면 끝난다).
                #   기준선이 통과했던 건 그 차가 창에 들어오기 2초 전에 나갔던 것 — 운이었다.
                #   가까운 정지차(구간 안)는 그대로 막는다(걸러내면 그 차를 들이받는다).
                if (spd < 1.0 and block_fx is not None
                        and fx > block_fx + room):
                    continue
                return False                                   #   (정지차만 걸러내면 그 차를 들이받음)
            if -self.rear_watch < fx < 0 and spd > max(1.0, ego_speed - 1.0):
                return False                                   # 뒤에서 따라붙는 차
        return True

    def _block_fx(self, objs, oid):
        for row in objs:
            if row[0] == oid:
                return row[1]
        return None

    def plan(self, ego, objs, now=None, allow_start=True, d_ego=0.0, hold=False):
        """objs = [(id, ds, d, speed, length, width, height)] — 경로기준 투영(곡선 오판 방지).

        allow_start=False 면 새 추월을 시작하지 않는다(교차로 등 차선 배치를 모르는 곳).
        이미 나가 있는 기동은 중단하지 않는다 — 중간에 멈추는 게 더 위험하다.
        """
        now = time.time() if now is None else now
        for row in objs:                       # 교통차량(움직인 적 있음) 추적
            oid, spd = row[0], row[3]
            if spd > 1.0:
                self.was_moving.add(oid)
                self.still_since.pop(oid, None)
            elif oid not in self.still_since:
                self.still_since[oid] = now
        block = self._own_lane_block(objs, d_ego)
        clear = self._target_lane_clear(objs, ego.speed, d_ego,
                                        block_fx=block[0] if block else None)
        # 바깥(drive.py)이 '고른 쪽이 실제로 막혔나'를 알아야 반대쪽을 시도할 수 있다.
        self.target_clear = clear

        if self.state != "WAIT":
            self._wait_since = None
        if self.state != "RETURN":
            self._return_since = None

        # ★hold: 추월 **대기(WAIT)에도 들어가지 않는다** — 적·황색 신호 앞에서 정지선
        #   안쪽에 선 앞차, 교차로 안·직전(제22조). 실측 2026-09-06 코스 G 신호 173·198·211:
        #   신호 기다리는 앞차를 장애물로 봐 WAIT -> 추월할 쪽 지시등(L)이 14초 켜졌다.
        #   WAIT 는 오프셋 0 이라 차는 안 움직이지만 **깜빡이가 거짓말**을 한다(항목13).
        #   allow_start 는 WAIT->PASS 만 막아서 이걸 못 잡았다.
        if hold and self.state == "WAIT":
            self.state = "FOLLOW"
            self._wait_since = None
        if self.state == "FOLLOW":
            self.offset, self.turn = 0.0, TS_OFF
            if block and block[0] < self.trigger and not hold:
                self.block_id = block[2]
                self.state = "WAIT"

        elif self.state == "WAIT":
            self.offset = 0.0
            if self._wait_since is None:
                self._wait_since = now
            gap_min = self.min_pass_gap
            # ⚠️ `block` 은 **None 일 수 있다.** 막고 있던 차가 그냥 가버리면 사라진다.
            #    실측 2026-08-24 주변교통 판(`_TR`, obj 최대 15): WAIT 로 10초 넘게 서 있다가
            #    앞차가 출발해 block 이 None 이 된 순간 `block[2]` 에서
            #    **TypeError: 'NoneType' object is not subscriptable** 로 주행이 죽었다.
            #    (1330.3,162.4) 에서 프로세스가 통째로 내려갔다 — 대회였으면 그대로 실격이다.
            #    아래 `if block is None: RETURN` 이 있는데 그건 **이 계산보다 뒤**에 있었다.
            #    지금까지 안 걸린 건 우리 판의 막는 차가 전부 **영영 안 움직이는 정지차**여서다.
            if (block is not None and ego.speed < 0.5
                    and now - self._wait_since > self.stuck_relax):
                # ⚠️ 완화 하한은 **기하로 정한다.** 한때 pass_clear(6.0m)로 낮췄는데
                #    그건 종방향 간격이 아니라 다른 양이라, 실제 범퍼 간격이 0.00m 여서
                #    반드시 닿는 값이었다 — 실측 2026-08-16 EV_COMBO_CHAIN: fx=6.11m 에서
                #    추월을 시작해 **clr=-0.01 로 접촉**, 이후 390초간 물린 채 정지
                #    (최악 -1.19m). 갇히는 것보다 들이받는 게 나쁘다.
                olen = next((o[4] for o in objs if o[0] == block[2]), 4.4)
                gap_min = self.front_overhang + olen / 2.0 + self.pass_margin
                if gap_min > self.min_pass_gap:
                    gap_min = self.min_pass_gap  # 완화인데 더 엄해지면 의미 없다
            # ★못 나가는 동안에는 깜빡이를 끈다. 나갈 수 있게 됐을 때만 미리 켠다.
            #   양쪽이 다 막혀 하염없이 서 있는데 좌측 깜빡이만 켜고 있으면 그 자체가
            #   잘못된 신호다(도교법: 진로변경 3초/30m 전에 켠다). 2026-08-15 실측 지적.
            can_go = clear and block is not None and block[0] > gap_min
            self.turn = (TS_LEFT if self.W > 0 else TS_RIGHT) if can_go else TS_OFF
            # ★★지시등을 켠 뒤 PASS_SIG_LEAD 가 지나야 나간다(2026-09-06). 실측 코스 G t=237:
            #   WAIT 에서 조건이 갖춰진 프레임에 바로 PASS -> 차선을 1.8초 뒤에 넘어 항목13 -3.
            #   경로 차선변경·종점 우측정차는 이미 3초를 지키는데 추월만 안 지켰다.
            if can_go and allow_start and now >= self.abort_until:
                if self._go_since is None:
                    self._go_since = now
            else:
                self._go_since = None
            armed = self._go_since is not None and now - self._go_since >= self.PASS_SIG_LEAD
            if block is None:                                  # 장애물 사라짐
                self.state = "RETURN"
            elif not allow_start or now < self.abort_until:
                pass                                            # 앞지르기 금지 구간이거나 중단 직후
            elif clear and block[0] > gap_min and armed:
                bid = block[2]
                traffic = bid in self.was_moving               # 달리다 멈춘 차(급정거 등)
                if not traffic and block[0] > self.early_pass:
                    self._enter_pass()                         # 원래 정지물 -> 멀리서 미리 우회
                elif ego.speed < 1.5:
                    # 급정거 차량은 '충분히 오래 멈춰있음'이 확인돼야 우회(재출발 대비)
                    t0 = self.still_since.get(bid)
                    if (not traffic) or (t0 is not None and now - t0 > self.stopped_hold):
                        self._enter_pass()

        elif self.state == "PASS":
            self.offset = self.W                               # 옆차선으로
            # ★★**옆 차로에 다 들어갔으면 지시등을 끈다**(2026-09-11 v6, 사용자 "차선 들어와서
            #   신호대기로 정지하면 깜빡이 바로 끄고"). 진로변경은 끝났다 — 그 차로를 달리는 중에
            #   켜 두면 뒤차에는 '또 옮긴다'로 보인다. 실측: 좌회전 차로로 비켜 들어가 적신호에
            #   섰는데 **좌측 지시등을 13초 켠 채** 서 있었다(좌회전 차로에서 좌측 = 좌회전한다는 뜻).
            #   원래 차로로 돌아갈 때는 RETURN 이 반대쪽을 켠다.
            if abs(d_ego - self.W) < self.RETURN_DONE_D:
                self.turn = TS_OFF
            else:
                self.turn = TS_LEFT if self.W > 0 else TS_RIGHT
            bfx = self._block_fx(objs, self.block_id)
            if bfx is None:
                # 한 프레임 누락(위 block_lost_hold 주석) — 잠깐은 그대로 간다.
                if self._block_lost_since is None:
                    self._block_lost_since = now
                elif (now - self._block_lost_since >= self.block_lost_hold
                      and not self._vru_in_return_path(objs, d_ego)):
                    self.state = "RETURN"
            else:
                self._block_lost_since = None
            _pc = self._esc_pass_clear if self._esc_pass_clear is not None else self.pass_clear
            if bfx is not None and bfx < -_pc:                 # 장애물 다 지남
                if not self._vru_in_return_path(objs, d_ego):  # ★그 뒤에 선 사람까지 지나야
                    self.state = "RETURN"
            # ★나가 있는 채로 멈춰버리면 복귀한다. 옆(특히 반대)차선에 차를 세워두는 건
            #   그 자체로 위반이고 대향차를 막는다(2026-08-15 EV_COMBO_PASSPED 실측:
            #   목표차선 보행자를 보고 off=+3.5 상태로 굳어 있었다).
            elif ego.speed < 0.5:
                if self._pass_stop_since is None:
                    self._pass_stop_since = now
                elif now - self._pass_stop_since > self.abort_sec:
                    # ★복귀가 **안전할 때만** 취소한다. 차로 중심에 아직 그 차가 서
                    #   있으면 복귀는 곧 그 차로 들어가는 것이다.
                    #
                    #   실측 2026-08-17 공식 v3 (7판 중 3판, 만나면 100% 실패):
                    #     t=25  OVT:PASS      off +3.50   <- 추월을 제대로 시작했다
                    #     t=28  NARROW_BLOCK  off  0.00   <- 여기서 세우자 취소됐다
                    #     t=34  재출발 -> t=36 접촉 -> t=37.5 정지, 1026초 미완주
                    #   우리를 세운 건 우리 안전망(NARROW_BLOCK)이고, 세운 자리가 하필
                    #   그 차 옆이었다. '멈췄으니 포기'가 그대로 충돌 명령이 된다.
                    #
                    #   ⚠️ 취소 규칙 자체는 없애지 않는다 — 반대차선에 차를 세워두는 건
                    #      그 자체로 위반이다(위 주석, EV_COMBO_PASSPED). 차로가 비면
                    #      그때 취소한다. 여기서 `_pass_stop_since` 를 다시 찍지 않는
                    #      것은, 비는 즉시 복귀시키기 위해서다.
                    if (self._own_lane_block(objs, d_ego) is None
                            and not self._vru_in_return_path(objs, d_ego)):
                        self.state = "RETURN"
                        self.abort_until = now + self.abort_cooldown
            else:
                self._pass_stop_since = None

        elif self.state == "RETURN":
            self.turn = TS_RIGHT if self.W > 0 else TS_LEFT    # 원차선 복귀 지시등
            self.offset *= 0.6                                  # 부드럽게 복귀
            if self._return_since is None:
                self._return_since = now
            # ★★복귀는 **명령이 0 이 됐을 때**가 아니라 **차가 실제로 돌아왔을 때** 끝난다(2026-09-09).
            #   명령은 0.6 배씩 줄어 0.2초면 0 인데 차는 3.4m 를 돌아오는 데 8m 를 달린다. 그 사이
            #   FOLLOW 로 넘어가면 `avoiding` 이 꺼져, 아직 옆 차로에 있는 차체 앞의 정지물에
            #   OBSTACLE_STOP 이 걸린다 — 실측 accident_junction: 대향 정지차 13.7m 앞 대향차로
            #   한복판(54.3, +3.47)에 서서 미완주. 몸이 돌아올 때까지(RETURN_DONE_D) 기다리되,
            #   너무 오래 못 돌아오면(RETURN_MAX_S) 그대로 FOLLOW 로 넘긴다.
            body_back = abs(d_ego) < self.RETURN_DONE_D
            timed_out = now - self._return_since > self.RETURN_MAX_S
            if abs(self.offset) < 0.3:
                self.offset = 0.0
            if abs(self.offset) < 0.3 and (body_back or timed_out):
                self.offset, self.turn = 0.0, TS_OFF
                self.block_id = None
                self.state = "FOLLOW"
                self._return_since = None
                self.clear_escape()

        return self.offset, self.turn, self.state, block

    def reset(self):
        self.state = "FOLLOW"
        self.offset = 0.0
        self.turn = TS_OFF
        self.block_id = None
        self._pass_stop_since = None
        self._block_lost_since = None
        self._return_since = None
        self.abort_until = 0.0
        # ★리스폰이면 세상이 바뀐 것이다 — 관측 이력도 같이 버린다. 안 버리면
        #   still_since 의 옛 시각 때문에 '충분히 오래 멈춘 차' 판정이 즉시 참이 된다.
        self._wait_since = None
        self.target_clear = True
        self.was_moving.clear()
        self.still_since.clear()
        self.clear_escape()
