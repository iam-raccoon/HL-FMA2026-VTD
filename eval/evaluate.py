"""
주행 자동 채점기 — 대회 100점 감점제 모사.

mock_vtd.py 가 GT(정답 위치/신호)를 알고 매 프레임 update() 호출, 종료 시 report().
감점 항목은 도로교통법 평가 기준을 근사한 것(실제 배점은 대회당일 공개 → 조정).
"""
import math
import os as _os
import sys as _sys

_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "src"))
from run_logger import oriented_gap, EGO_CX, EGO_HALF_L, EGO_HALF_W   # noqa: E402  상자 대 상자 실여유


PENALTY = {
    "collision": 50,     # 충돌(치명)
    "red_run": 20,       # 적신호 정지선 통과
    "school_over": 20,   # 어린이보호구역 속도위반(가중)
    "speed_over": 10,    # 일반 속도위반
    "ped_near": 15,      # 보행자 근접통과(양보실패)
    "goal_miss": 15,     # 목표 미도달
    "harsh": 5,          # 급감속(난폭운전)
}

GOAL_R = 15.0            # 목표 도달 판정 반경[m]. check_run.py 와 같은 값으로 맞춘다.


class Scorer:
    def __init__(self, scenario):
        self.sc = scenario
        self.score = 100.0
        self.violations = []
        self._done = set()
        self.collided = False
        self.reached = False
        self.max_speed = 0.0
        self.min_dist = {}
        self.prev_v = None
        self.prev_x = self.prev_y = None

    def _pen(self, key, kind, msg):
        if key in self._done:
            return
        self._done.add(key)
        p = PENALTY[kind]
        self.score -= p
        self.violations.append(f"-{p:>3.0f}  {msg}")

    def update(self, t, ex, ey, ev, actors, tl_state, dt, eh=0.0):
        self.max_speed = max(self.max_speed, ev)
        px = self.prev_x            # 이번 프레임에 갱신되기 **전** 값(정지선 통과 판정용)

        # 속도위반 (보호구역 가중)
        limit = self.sc.limit_at(ex)
        in_school = any(z.x_start <= ex <= z.x_end and "보호구역" in z.name
                        for z in self.sc.zones)
        if ev > limit * 1.1:
            if in_school:
                self._pen(f"sch{int(t)}", "school_over",
                          f"보호구역 속도위반 {ev*3.6:.0f}>{limit*3.6:.0f}km/h @x={ex:.0f}")
            else:
                self._pen(f"spd{int(t)}", "speed_over",
                          f"속도위반 {ev*3.6:.0f}>{limit*3.6:.0f}km/h @x={ex:.0f}")

        # 충돌 / 최소거리 / 보행자 근접
        for a in actors:
            dx, dy = a["x"] - ex, a["y"] - ey
            d = math.hypot(dx, dy)
            self.min_dist[a["id"]] = min(self.min_dist.get(a["id"], 1e9), d)
            # ★상자 대 상자(SAT) — run_logger·score_fma 와 **같은 자**로 잰다(2026-09-09).
            #   예전 '도로정렬 근사'(뒷축 기준 종·횡 각각)는 자차가 비스듬히 옆으로 빠지는 중
            #   실제로는 +0.43m 떨어진 자전거를 '충돌' 로 찍었다(accident_junction 재현판:
            #   헤딩 27° 로 옆을 지나는데 dx 3.3 < 3.4 · dy 0.7 < 1.25). 채점기(항목14)는
            #   SAT 로 보므로 모의만 더 엄해 **되는 회피를 실패로** 만들었다.
            _c, _s = math.cos(eh), math.sin(eh)
            _cx, _cy = ex + EGO_CX * _c, ey + EGO_CX * _s          # 뒷축 -> 차체 중심
            _rx, _ry = a["x"] - _cx, a["y"] - _cy
            _gap = oriented_gap(_rx * _c + _ry * _s, -_rx * _s + _ry * _c,
                                a.get("hd", 0.0) - eh, a["size"][0] * 0.5, a["size"][1] * 0.5)
            if _gap < 0.0:
                self.collided = True
                self._pen(f"col{a['id']}", "collision",
                          f"충돌! actor {a['id']}({a['type']}) d={d:.1f}m")
            elif a["type"] == "pedestrian" and d < 3.0 and ev > 1.0:
                self._pen(f"ped{a['id']}", "ped_near",
                          f"보행자 근접통과 d={d:.1f}m (양보실패)")

        # 리스폰(순간이동) 프레임은 급감속 판정 제외
        jumped = (self.prev_x is not None
                  and math.hypot(ex - self.prev_x, ey - self.prev_y) > 5.0)
        self.prev_x, self.prev_y = ex, ey
        # 급감속(난폭)
        if not jumped and self.prev_v is not None and dt > 0:
            acc = (ev - self.prev_v) / dt
            if acc < -6.0:
                self._pen(f"harsh{int(t*2)}", "harsh", f"급감속 {acc:.1f}m/s²")
        self.prev_v = ev

        # 적신호 정지선 통과 — **선을 넘는 순간**만 본다.
        #  ⚠️ 예전엔 '적색인데 정지선 뒤 8m 안에서 움직이면' 이었다. 그러면 녹색·황색에
        #     적법하게 진입해 교차로를 **빠져나가는 중**에 적색으로 바뀌어도 위반으로
        #     잡힌다. 실측 2026-08-20(비보호 좌회전 판): 좌회전을 시작한 뒤 적색이 됐고,
        #     교차로 안에 서 있는 게 더 위험해 빠져나갔는데 -20 을 먹었다.
        #     **제어기는 옳게 행동했고 채점 기준이 틀렸다** — check_run.py 의 신호 판정을
        #     '진행방향 성분으로 넘는 순간'으로 고친 것과 같은 종류의 오탐이다.
        if px is not None:
            for lt in self.sc.lights:
                if px <= lt.stop_line_x < ex and tl_state == 1:
                    self._pen(f"red{lt.id}", "red_run",
                              f"적신호 정지선 통과 (light {lt.id} @x={lt.stop_line_x:.0f})")

        # 목표 도달
        # ⚠️ 반경은 **주최측 기준(10~20m)** 을 쓴다 — 강의 [103:56] "이 포인트 반경으로
        #    10m, 20m 안으로 들어오면 자동으로 시험 시스템이 종료". eval/check_run.py 도
        #    같은 15m 다. 예전엔 여기만 3.0m 였는데, 차 길이(4.8m)보다 작아서 사실상
        #    '범퍼를 점 위에 올려라'였다. 그래서 cutin_lead 가 -15 를 먹고 있었다:
        #    앞차가 **목표 좌표 (220,0) 위에 정차**해 우리가 그 뒤 10.1m 에 정상적으로
        #    섰는데 미도달로 채점됐다. 제어기는 옳게 행동했고 채점 기준이 틀렸다.
        gx, gy = self.sc.ego_goal
        if math.hypot(gx - ex, gy - ey) < GOAL_R:
            self.reached = True

    def report(self, elapsed):
        if not self.reached and not self.collided:
            self._pen("goal", "goal_miss", f"목표 미도달 ({elapsed:.0f}s 경과)")
        s = max(0.0, self.score)
        ok = (s >= 60) and (not self.collided) and self.reached
        L = []
        L.append("=" * 50)
        L.append(f"  시나리오 : {self.sc.name}")
        L.append(f"  점수     : {s:>5.0f} / 100     {'✅ PASS' if ok else '❌ FAIL'}")
        L.append(f"  목표도달 : {'예' if self.reached else '아니오'}"
                 f"   충돌: {'예' if self.collided else '아니오'}"
                 f"   최고속도: {self.max_speed*3.6:.0f}km/h   소요: {elapsed:.0f}s")
        L.append("-" * 50)
        if self.violations:
            L.append("  감점 내역:")
            L += ["    " + v for v in self.violations]
        else:
            L.append("  감점 없음 — 완벽 주행")
        if self.min_dist:
            L.append("  최소 접근거리:")
            for aid, d in self.min_dist.items():
                L.append(f"    actor {aid}: {d:5.1f} m")
        L.append("=" * 50)
        return "\n".join(L)
