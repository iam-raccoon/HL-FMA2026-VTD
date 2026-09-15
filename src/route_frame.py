"""경로(route) 기준 좌표계 — 곡선에서도 정확한 차선 판정용.

⚠️ 왜 필요한가(2026-08-14 실측): 객체를 ego 직교좌표(fx,fy)로만 보면 **도로 곡률이 fy를 오염**시킨다.
   곡선에서 50~70m 앞 같은 차선 차량이 fy=-3m로 잡혀 '옆차선'으로 오판되고,
   반대로 옆차선 차가 fy≈0으로 잡혀 불필요한 급제동을 유발한다.
   -> 객체를 우리 '경로 폴리라인'에 투영해 (s=경로진행거리, d=경로중심 횡오프셋)으로 판정한다.

규약: d > 0 = 경로 진행방향 기준 왼쪽.
"""
import math
from bisect import bisect_right


class RouteFrame:
    def __init__(self, route):
        self.pts = list(route)
        n = len(self.pts)
        self.cum = [0.0] * n
        for i in range(1, n):
            self.cum[i] = self.cum[i-1] + math.hypot(self.pts[i][0]-self.pts[i-1][0],
                                                     self.pts[i][1]-self.pts[i-1][1])
        self.total = self.cum[-1] if n else 0.0

    def project(self, x, y, hint=None, window=80):
        """(s, d, seg_i) 반환. hint 부근만 탐색하면 빠르고 자기교차 경로에서도 안전."""
        n = len(self.pts)
        if n < 2:
            return 0.0, 0.0, 0
        if hint is None:
            lo, hi = 0, n - 1
        else:
            lo, hi = max(0, hint - window), min(n - 1, hint + window)
        best = (1e18, 0.0, 0.0, lo)
        for i in range(lo, hi):
            ax, ay = self.pts[i]; bx, by = self.pts[i+1]
            dx, dy = bx - ax, by - ay
            L2 = dx*dx + dy*dy
            if L2 < 1e-12:
                continue
            t = ((x-ax)*dx + (y-ay)*dy) / L2
            # ★**양 끝 구간은 t 를 가두지 않는다.** 가두면 경로 밖의 물체가 끝점에
            #   달라붙어 '경로 위'로 보인다 — 실측 2026-08-22 hz_traffic:
            #   경로 시작 60m **뒤**에 있던 차가 s=0.00 d=0.00 으로, 즉 **자차와 겹쳐**
            #   투영돼 차체 안전망이 0프레임부터 NARROW_BLOCK 을 걸었고 자차가
            #   **영원히 출발하지 못했다**(최고속도 0km/h). 대회에서 뒤에 차가 한 대라도
            #   있으면 그대로 못 움직인다.
            #   끝 구간만 열어두면 s 가 음수(뒤) 또는 경로길이 초과(앞)로 나와
            #   감시창(-4~25m) 밖으로 정상적으로 걸러진다. 중간 구간은 그대로 가둔다 —
            #   거기서 열면 최근접 구간 판정이 무너진다.
            t = max(0.0 if i > 0 else -1e6, min(1.0 if i < n - 2 else 1e6, t))
            px, py = ax + t*dx, ay + t*dy
            dd = math.hypot(x-px, y-py)
            if dd < best[0]:
                L = math.sqrt(L2)
                cross = ((x-ax)*dy - (y-ay)*dx) / L        # >0 이면 진행방향 오른쪽
                best = (dd, self.cum[i] + t*L, -cross, i)  # d>0 = 왼쪽
        return best[1], best[2], best[3]

    def heading_at(self, i):
        i = max(0, min(len(self.pts)-2, i))
        ax, ay = self.pts[i]; bx, by = self.pts[i+1]
        return math.atan2(by-ay, bx-ax)

    def point_at(self, s):
        """경로 시작부터 진행거리 ``s``[m]에 있는 점을 선형보간해 반환한다.

        경로가 비었거나 ``s``가 경로 범위 밖이면 ``None``을 반환한다. 호출부는
        이를 이용해 전방 기준점을 만들 수 없는 경로 끝 등의 상황에서 안전하게
        기존 판정으로 되돌아간다.

        쓰는 곳: **적신호 우회전 양보 판정의 기준점** — '회전을 마친 뒤 우리가 있을
        자리'가 필요하다(`drive.py::_rtor_point`). 자차 현재 위치로 판정하면 뒤차와
        왼쪽으로 지나가는 차까지 충돌 상대로 잡힌다(실측 2026-08-25 코스 H: 왼쪽 3.1m 의
        1.4m/s 짜리 차 하나에 18초를 섰다).
        """
        if not self.pts or s < 0.0 or s > self.total:
            return None
        if len(self.pts) == 1 or s >= self.total:
            return tuple(self.pts[-1])

        # cum[i] <= s < cum[i+1]인 선분을 찾는다. bisect_right를 쓰면
        # 같은 점이 연속된 0길이 선분도 건너뛸 수 있다.
        i = bisect_right(self.cum, s) - 1
        i = max(0, min(len(self.pts) - 2, i))
        seg = self.cum[i + 1] - self.cum[i]
        if seg <= 1e-12:
            return tuple(self.pts[i])
        t = (s - self.cum[i]) / seg
        ax, ay = self.pts[i]
        bx, by = self.pts[i + 1]
        return ax + t * (bx - ax), ay + t * (by - ay)


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
