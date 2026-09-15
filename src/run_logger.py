"""주행 로그(CSV) 기록 — 사후 채점(check_run.py)과 신호 정지선 매핑(map_lights.py)의 입력.

⚠️ 매 프레임 flush 한다. 이벤트 주입기(vtd/inject_event.py)가 이 파일을 실시간으로 읽어
   '앞차가 N m 이내'같은 조건을 판단하기 때문(9910은 단일 클라이언트라 못 씀).
"""
import math

# ⚠️ 컬럼 추가는 **맨 끝에만**. vtd/inject_event.py 가 p[9](near_fx)·p[12](near_len)를
#    위치로 읽는다(실시간 tail이라 DictReader를 못 씀).
HEADER = ("t,x,y,heading,v,tl_id,tl_state,reason,nobj,"
          "near_fx,near_fy,near_v,near_len,rf_ds,rf_d,clr,sig,off,cap_by,objs,"
          "d_ego,base,nudge\n")

# ★`objs` = 그 프레임의 **모든** 객체를
#   `fx:fy:len:wid:hgt:spd:clr:dpath:rw:dh` 로 이어 붙인 것('|' 구분).
#   len/wid 는 **원본 치수**, `rw` 는 판단이 실제로 쓰는 **경로축 폭**(route_extent),
#   `dh` 는 자차기준 상대방위[도]. 뒤에 덧붙였으므로 앞칸만 읽는 기존 파서는 그대로 돈다.
#   왜 필요한가(실측 2026-08-18 EV_COMBO_PASSPED): 700초 정지했는데 로그에 최근접 하나만
#   남아서 **무엇이 막고 있는지 알 수가 없었다**(최근접은 14.9m 앞·여유 8.86m 로 멀쩡했다).
#   막는 놈이 두 번째 객체였는데 그게 기록에 없었다. 사후 분석에 이게 없으면 재현 실행을
#   또 돌려야 한다 — 한 판에 수백 초다.
# ★`dpath` = **경로기준 절대 횡위치**(= rf d + d_ego). 판단(nudge·추월·차체안전망)은 전부
#   이 값으로 하는데 ego 기준(fx,fy)만 남기면 **오프라인 재현이 안 된다**.
#   실측 2026-08-18: EV_COMBO_PASSPED 를 유닛테스트로 재현하려 했는데 회피가 시작된
#   시점의 경로기준 좌표가 없어서, 현장 값이 아니라 '같은 구조'를 합성해야 했다.
#   ego 기준은 내가 흔들리면 같이 흔들려서 그때의 판단 근거가 못 된다.
# ★8 이었다. **그 잘림 때문에 진단을 여러 번 헛짚었다**(2026-08-25).
#   주변 교통 판은 객체가 20~30개인데 8개만 남으니, "옆 차로가 비었나" "앞에 뭐가
#   있었나" 를 물어보면 **없는 걸 없다고 답하는지 잘려서 안 보이는지 구분이 안 된다.**
#   실제로 코스 H 교차로 정지를 분석하다 "진입 시점에 앞이 비어 있었다"까지 갔는데,
#   그게 사실인지 8개 밖으로 밀린 건지 알 수 없었다.
#   패킷이 30개까지 주므로 **전부 남긴다.** CSV 가 3~4배 커지지만(코스 E 27MB -> ~90MB)
#   못 믿는 로그보다 낫다. 이 파일은 채점·진단 전용이고 주행 성능과 무관하다.
OBJ_MAX = 30                    # 패킷 상한(=전부). 잘라 두면 진단이 거짓말을 한다

# 차체 박스(뒷축 기준). 앞범퍼 +3.808, 전장 4.848 -> 뒤 오버행 1.040
EGO_FRONT, EGO_LEN, EGO_HALF_W = 3.808, 4.848, 0.943
EGO_CX = EGO_FRONT - EGO_LEN / 2.0          # 차체 중심의 뒷축 기준 전방거리
EGO_HALF_L = EGO_LEN / 2.0


def body_clearance(s, o):
    """ego 차체와 객체 박스 사이 실여유[m]. 음수면 겹침(접촉).

    분리축 정리(SAT): 두 박스의 축 4개에 각각 투영해, 떨어진 정도가 가장 큰 축의 값.
    하나라도 양수면 두 박스는 떨어져 있다.

    ⚠️ 처음엔 객체를 '외접 축정렬 박스'로 근사했는데, **차선변경 중이라 비스듬한 차**는
       전장 4.39m 가 옆폭으로 계산돼 없는 접촉을 만들어냈다(2026-08-15 EV_CUTIN 오탐 −1.31m).
       정렬된 물체끼리는 두 방식이 같아서 그전 수치들은 영향 없다.
    """
    dx, dy = o.x - s.x, o.y - s.y
    fx = dx * math.cos(-s.heading) - dy * math.sin(-s.heading)
    fy = dx * math.sin(-s.heading) + dy * math.cos(-s.heading)
    return oriented_gap(fx - EGO_CX, fy, o.heading - s.heading,
                        o.length / 2.0, o.width / 2.0)


def oriented_gap(cx, cy, dh, ol, ow):
    """**자차 차체 기준** 상자 대 상자 실여유[m]. 음수면 겹침.

    cx, cy : 차체 **중심**에서 본 객체 중심(자차 기준 전방 x+, 좌 y+)
    dh     : 객체 방위 - 자차 방위[rad]
    ol, ow : 객체 반길이 · 반폭(원본 치수)

    ⚠️ **이 구현은 하나뿐이어야 한다.** behavior 의 안전망도 같은 걸 쓴다 —
       한쪽만 고치면 "로거는 안 닿았다는데 제어는 닿는다고 선다"가 된다.
       실제로 오늘(2026-08-26) 같은 계산의 사본 두 개가 갈라져 하루를 태웠다.
    """
    c, sn = math.cos(dh), math.sin(dh)
    best = -1e9
    # ego 축 2개 + 객체 축 2개
    for ax, ay in ((1.0, 0.0), (0.0, 1.0), (c, sn), (-sn, c)):
        d = abs(cx * ax + cy * ay)
        r_ego = EGO_HALF_L * abs(ax) + EGO_HALF_W * abs(ay)
        r_obj = ol * abs(c * ax + sn * ay) + ow * abs(-sn * ax + c * ay)
        best = max(best, d - r_ego - r_obj)
    return best


class RunLogger:
    def __init__(self, path=None):
        self.f = open(path, "w", encoding="utf-8") if path else None
        if self.f:
            self.f.write(HEADER)

    def log(self, t, s, cmd):
        """t[s], s=vtd_io.State, cmd=drive.Command"""
        if not self.f:
            return
        # ego 좌표계 최근접 객체(사람이 보기 쉬운 값)
        nx = ny = nv = nl = 0.0
        best = None
        for o in s.objects:
            dx, dy = o.x - s.x, o.y - s.y
            fx = dx * math.cos(-s.heading) - dy * math.sin(-s.heading)
            fy = dx * math.sin(-s.heading) + dy * math.cos(-s.heading)
            if best is None or abs(fx) < abs(best[0]):
                best = (fx, fy, o.speed, o.length)
        if best:
            nx, ny, nv, nl = best
        # 판단이 실제로 쓰는 '경로기준' 값(차선 오판 디버깅용)
        rds = rd = 0.0
        cand = [o for o in cmd.rf_objs if o[0] > 0 and max(o[3], o[4]) > 1.2]
        if cand:
            c = min(cand, key=lambda o: o[0])
            rds, rd = c[0], c[1]
        # 차체 기준 최소 실여유(음수 = 접촉). 우리 채점기가 '연료통을 밟고도 충돌 아님'으로
        # 통과시킨 구멍을 막는다(2026-08-15).
        clr = min((body_clearance(s, o) for o in s.objects), default=99.9)
        # 모든 객체(가까운 순). ',' 는 CSV 구분자라 못 쓰므로 ':' 와 '|' 로 엮는다.
        # 경로기준 횡위치는 cmd.rf_objs 에 있다(ds, d, spd, len, wid, hgt, id).
        #  d 는 **ego 기준** 상대값이라 d_ego 를 더해 경로 절대값으로 되돌린다.
        # ★★**경로축 폭(rw)과 자차기준 상대방위(dh)도 남긴다.**
        #   판단은 원본 폭이 아니라 `route_extent` 가 낸 **경로축 폭**으로 한다
        #   (비스듬히 선 차는 전장이 옆폭으로 잡힌다). 그런데 로그에는 원본 폭만
        #   있어서, 왜 막혔는지 물으면 매번 **각도를 역산해 추측**해야 했다 —
        #   2026-08-26 하루에만 세 번(코스 H 4.44m 오판, EV_ONCOMING 목표차선 판정,
        #   EV_COMBO_PASSPED). 추측은 두 번 틀렸다. 판단이 쓰는 값을 그대로 남긴다.
        #   ⚠️ 뒤에 덧붙이기만 한다 — 기존 파서는 앞 5~8칸만 읽으므로 그대로 돈다.
        d_ego_now = getattr(cmd, "d_ego", 0.0)
        dpath = {o[6]: o[1] + d_ego_now for o in (cmd.rf_objs or []) if len(o) > 6}
        rwid = {o[6]: o[4] for o in (cmd.rf_objs or []) if len(o) > 6}
        rows = []
        for o in s.objects:
            dx, dy = o.x - s.x, o.y - s.y
            fx = dx * math.cos(-s.heading) - dy * math.sin(-s.heading)
            fy = dx * math.sin(-s.heading) + dy * math.cos(-s.heading)
            rows.append((abs(fx), fx, fy, o))
        rows.sort()
        objs = "|".join(
            f"{fx:.1f}:{fy:.1f}:{o.length:.1f}:{o.width:.1f}:{o.height:.1f}:"
            f"{o.speed:.1f}:{body_clearance(s, o):.2f}:"
            f"{dpath.get(o.id, float('nan')):.2f}:"
            f"{rwid.get(o.id, float('nan')):.2f}:"
            f"{math.degrees((o.heading - s.heading + math.pi) % (2 * math.pi) - math.pi):.0f}"
            for _a, fx, fy, o in rows[:OBJ_MAX])
        self.f.write(f"{t:.2f},{s.x:.3f},{s.y:.3f},{s.heading:.4f},{s.speed:.2f},"
                     f"{s.tl_id},{s.tl_state},{cmd.reason},{len(s.objects)},"
                     f"{nx:.2f},{ny:.2f},{nv:.2f},{nl:.2f},{rds:.2f},{rd:.2f},{clr:.2f},"
                     f"{cmd.turn},{cmd.lane_offset:.2f},{cmd.cap_by},{objs},"
                     f"{getattr(cmd,'d_ego',0.0):.2f},{getattr(cmd,'base',0.0):.2f},{getattr(cmd,'nudge',0.0):.2f}\n")
        self.f.flush()

    def close(self):
        if self.f:
            self.f.close()
            self.f = None
