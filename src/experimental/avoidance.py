"""
Lattice 기반 장애물 회피 — 횡방향 오프셋 후보 중 무충돌 최소비용 선택.

참고 레포(2026-08-12 분석):
  777juwon   : Bezier 후보경로 + 충돌비용 + 히스테리시스 + cut-in 가드
  rwambangho : 큐빅 오프셋 6경로 + 충돌비용
방식: 차선중심 기준 횡오프셋 후보 [..-1.75, 0, +1.75..] 각각에 대해
      전방 horizon 내 장애물과의 횡겹침으로 점유 판정 -> 무충돌 후보 중
      (중앙선호 + 이전선택 유지=히스테리시스) 최소비용 선택.
출력: (lateral_offset[m], blocked, turn_signal). 좌+ 규약(오프셋>0=좌측).
"""
import math

TS_OFF, TS_LEFT, TS_RIGHT = 0, 1, 2


def offset_path(path, d):
    """경로를 좌법선 방향으로 d[m] 평행이동 (좌+). Pure Pursuit가 따라갈 목표경로."""
    if abs(d) < 1e-3 or len(path) < 2:
        return path
    out = []
    n = len(path)
    for i in range(n):
        if i < n - 1:
            tx, ty = path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]
        else:
            tx, ty = path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]
        m = math.hypot(tx, ty)
        if m < 1e-6:
            nx, ny = 0.0, 1.0
        else:
            nx, ny = -ty / m, tx / m           # 좌법선
        out.append((path[i][0] + d * nx, path[i][1] + d * ny))
    return out


class LatticeAvoider:
    def __init__(self, offsets=(-3.5, -2.6, -1.75, 0.0, 1.75, 2.6, 3.5),
                 horizon=40.0, veh_half_w=0.95, margin=0.4,
                 w_offset=1.0, w_change=2.5, hold=0.1):
        self.offsets = list(offsets)
        self.horizon = horizon
        self.vhw = veh_half_w
        self.margin = margin
        self.w_off = w_offset
        self.w_chg = w_change
        self.hold = hold          # 방향지시등 트리거 오프셋 변화 임계
        self.d_prev = 0.0

    def plan(self, ego, objects):
        """objects: [Obj] (world). 반환 (offset, blocked, turn_signal)."""
        occ = {d: False for d in self.offsets}
        cur_block_fx = float("inf")               # 현재 차선을 막는 가장 가까운 장애물 종거리
        for o in objects:
            # 횡회피 대상 = 정지 장애물만. 움직이는 차=ACC(behavior), 보행자=정지(behavior).
            is_ped = (o.length < 1.2 and o.width < 1.2)
            if is_ped or o.speed > 0.5:
                continue
            dx, dy = o.x - ego.x, o.y - ego.y
            s = dx * math.cos(-ego.heading) - dy * math.sin(-ego.heading)   # 전방+
            l = dx * math.sin(-ego.heading) + dy * math.cos(-ego.heading)   # 좌+
            if s <= 0 or s > self.horizon:
                continue
            ohw = o.width * 0.5                    # 횡폭 반값(도로정렬 차량 기준)
            clear = self.vhw + ohw + self.margin
            for d in self.offsets:
                if abs(l - d) < clear:
                    occ[d] = True
            if abs(l - self.d_prev) < clear:       # 지금 주행 차선을 막나
                cur_block_fx = min(cur_block_fx, s)

        # 현재 차선 안 막힘 -> 차선 유지(중앙 복귀)
        if not occ[self.d_prev]:
            self.d_prev = 0.0 if abs(self.d_prev) < 0.3 else self.d_prev
            return self.d_prev, False, TS_OFF

        # 안전 차선변경 종거리 확보됐나. 최소 14m(급정거 차 근접 우회는 위험 -> 그냥 정지).
        # 멀리서 보이는 정지장애물만 부드럽게 우회, 가까운 건 정지(behavior).
        safe_lc = max(14.0, ego.speed * 3.0 + 6.0)
        if cur_block_fx < safe_lc:
            return self.d_prev, True, TS_OFF       # 너무 가까움 -> 정지(급브레이크/대기)

        best, best_cost = None, float("inf")
        for d in self.offsets:
            if occ[d]:
                continue
            cost = self.w_off * abs(d) + self.w_chg * abs(d - self.d_prev)
            if cost < best_cost:
                best_cost, best = cost, d

        if best is None:
            return self.d_prev, True, TS_OFF     # 전 차선 막힘 -> 정지

        turn = TS_OFF
        if best > self.d_prev + self.hold:
            turn = TS_LEFT
        elif best < self.d_prev - self.hold:
            turn = TS_RIGHT
        self.d_prev = best
        return best, False, turn
