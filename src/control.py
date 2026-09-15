"""
횡/종 제어 — Pure Pursuit + 2-pass 속도 프로파일 + 종방향 PI(anti-windup).

참고 레포(2026-08-12 분석):
  wonsukHeo1025 : 2-pass 곡률 속도 프로파일 (전/후방 종가속 제한)  <- 최고 자산
  777juwon      : Stanley/PI-D anti-windup
  rwambangho    : v = sqrt(r*g*mu) 곡률 속도, 속도비례 lookahead
출력은 대회 인터페이스에 맞춰 조향각[rad], targetAccel[m/s^2].
"""
import math


def norm_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class PurePursuit:
    """전방주시(lookahead) 기반 조향. lookahead = clamp(lmin + k*v, lmin, lmax)."""

    def __init__(self, wheelbase=2.95, k=0.5, lookahead_min=4.0, lookahead_max=20.0,
                 max_steer=math.radians(35)):
        self.L = wheelbase
        self.k = k
        self.lmin = lookahead_min
        self.lmax = lookahead_max
        self.max_steer = max_steer

    def lookahead(self, speed: float) -> float:
        return max(self.lmin, min(self.lmax, self.lmin + self.k * speed))

    def steer(self, x, y, heading, speed, path):
        """path: 자차 전방 [(x,y), ...]. 조향각[rad] 반환."""
        if len(path) < 2:
            return 0.0
        Ld = self.lookahead(speed)
        # 가장 가까운 점에서 시작해 누적거리 Ld 인 목표점 탐색
        best_i, best_d = 0, float("inf")
        for i, (px, py) in enumerate(path):
            d = math.hypot(px - x, py - y)
            if d < best_d:
                best_d, best_i = d, i
        target = path[-1]
        acc = 0.0
        prev = path[best_i]
        for i in range(best_i, len(path)):
            px, py = path[i]
            acc += math.hypot(px - prev[0], py - prev[1])
            prev = (px, py)
            if acc >= Ld:
                target = (px, py)
                break
        # 차량 좌표계 변환 (전방 x+, 좌 y+)
        dx, dy = target[0] - x, target[1] - y
        lx = dx * math.cos(-heading) - dy * math.sin(-heading)
        ly = dx * math.sin(-heading) + dy * math.cos(-heading)
        dist = math.hypot(lx, ly)
        if dist < 1e-3:
            return 0.0
        curvature = 2.0 * ly / (dist * dist)
        steer = math.atan(self.L * curvature)
        return max(-self.max_steer, min(self.max_steer, steer))


def path_curvature(path):
    """세 점 외접원 기반 곡률(1/R) 리스트."""
    n = len(path)
    k = [0.0] * n
    for i in range(1, n - 1):
        x0, y0 = path[i - 1]; x1, y1 = path[i]; x2, y2 = path[i + 1]
        a = math.hypot(x1 - x0, y1 - y0)
        b = math.hypot(x2 - x1, y2 - y1)
        c = math.hypot(x2 - x0, y2 - y0)
        if a * b * c < 1e-6:
            continue
        area = abs((x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)) * 0.5
        k[i] = 4.0 * area / (a * b * c)
    return k


def speed_profile_2pass(path, v_max, a_lat=3.0, a_acc=1.5, a_dec=3.0):
    """
    곡률 -> 횡가속 제한 속도, 이어서 후방(감속)·전방(가속) 패스로 종가속 제한.
    반환: 각 웨이포인트 목표속도[m/s].
    """
    n = len(path)
    if n == 0:
        return []
    k = path_curvature(path)
    v = [v_max if kk <= 1e-4 else min(v_max, math.sqrt(a_lat / kk)) for kk in k]
    # 후방 패스: 앞의 저속 지점에 맞춰 미리 감속
    for i in range(n - 2, -1, -1):
        ds = math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_dec * ds))
    # 전방 패스: 종가속 한계로 완만히 가속
    for i in range(1, n):
        ds = math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * a_acc * ds))
    return v


def apply_speed_limits(route, v_prof, limit_fn, a_dec=2.0, per_index=None):
    """
    위치 기반 제한속도(보호구역 등)를 v_prof에 반영 + 역방향 패스로 '진입 전 미리 감속'.
    limit_fn(x) -> 해당 x의 제한속도[m/s].
    per_index: [경로점별 제한속도] — 있으면 이걸 쓴다(x 하나로는 루프 있는 코스를
               구분할 수 없다. 노면표시에서 읽은 도로별 제한이 여기로 들어온다).
    """
    n = len(route)
    v = list(v_prof)
    for i in range(n):
        lim = limit_fn(route[i][0])
        if per_index is not None and i < len(per_index) and per_index[i] is not None:
            lim = min(lim, per_index[i])
        v[i] = min(v[i], lim)
    for i in range(n - 2, -1, -1):
        ds = math.hypot(route[i + 1][0] - route[i][0], route[i + 1][1] - route[i][1])
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_dec * ds))
    return v


class LongPI:
    """속도 오차 -> targetAccel[m/s^2] PI 제어 (적분 anti-windup).

    ⚠️ 제동은 가속보다 게인을 크게(비대칭). 대칭 게인(kp0.8)이면 앞차가 갑자기 서서
       목표속도가 확 낮아져도 -1.5m/s² 정도만 나와 **정지거리가 모자라 추돌**한다(2026-08-14 실측).
    """

    def __init__(self, kp=0.8, ki=0.25, a_min=-5.0, a_max=2.0, kp_brake=2.5):
        self.kp, self.ki = kp, ki
        self.kp_brake = kp_brake
        self.amin, self.amax = a_min, a_max
        self.i = 0.0

    HOLD_V = 0.3           # m/s. 이보다 느리면 '서 있는 것'으로 본다
    HOLD_A = -1.5          # m/s². 정차 유지 제동(경사에서 굴러가지 않을 만큼)

    def accel(self, v_target, v_cur, dt):
        # ★"멈춰 있어라"는 명령은 **가속도 0 이 아니라 제동**이다. 속도오차가 0 이면
        #   PI 출력이 0 이 되는데, VTD 물리에서는 경사가 있으면 그대로 굴러간다.
        #   실측 2026-08-16 EV_COMBO_CHAIN: ego v=0.00 · 앞차 v=0.00 인데 간격이
        #   70초에 걸쳐 7.11 -> 5.80m 로 줄었다(≈2cm/s). 그 바람에 추월 최소간격
        #   완화선(6.0m)마저 밑돌아 영원히 못 빠져나왔다.
        if v_target <= 1e-3 and v_cur < self.HOLD_V:
            self.i = 0.0
            return self.HOLD_A
        e = v_target - v_cur
        kp = self.kp_brake if e < 0 else self.kp
        a = kp * e + self.ki * self.i
        a_cl = max(self.amin, min(self.amax, a))
        if a == a_cl:          # 포화 아닐 때만 적분(anti-windup)
            self.i += e * dt
        return a_cl

    def reset(self):
        self.i = 0.0
