#!/usr/bin/env python3
"""시나리오를 로드한 **직후** ego 가 어디를 보고 있는지 한 프레임만 읽는다.

왜 필요한가(실측 2026-08-19): 주최측은 좌표만 준다. 그런데 왕복도로에서 그 좌표를
담는 차로는 **한쪽 방향뿐**일 수 있고, VTD 는 코스 진행 방향에 맞춰 **반대 차로에**
차를 놓는다(2.96m 옆). 그러면 우리 경로가 178° 반대로 시작한다 — 새 코스 A 에서
실제로 그랬고, 차가 앞으로 가버려 **코스의 44% 를 안 갔다**(커버리지 56%).

그래서 대회 당일은 이 순서다:
    ① 시나리오 로드            python3 vtd/load_scenario.py <이름>
    ② ego 헤딩 읽기            python3 vtd/ego_pose.py --host <VTD_IP>
    ③ 그 헤딩으로 경로 계산     python3 vtd/plan_route.py $X --heading <도> x1 y1 ...
    ④ 검사 -> 차로계획 -> 주행

⚠️ **9910 은 단일 클라이언트다.** 주행이 도는 중에 이걸 붙이면 제어가 끊긴다.
   로드 직후, main.py 를 띄우기 **전에만** 쓴다.
"""
import argparse
import math
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0] + "/../src")
from vtd_io import VTDLink                                        # noqa: E402


def _auto_host():
    import os, sys
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
    from netcfg import default_host
    return default_host()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=None,
                    help="안 주면 자기 IP 대역으로 판별(src/netcfg.py)")
    ap.add_argument("--port", type=int, default=9910)
    ap.add_argument("--json", action="store_true", help="한 줄 JSON 으로 출력")
    a = ap.parse_args()
    if a.host is None:
        a.host = _auto_host()

    link = VTDLink(a.host, a.port)
    link.connect()
    try:
        for _ in range(50):                  # 첫 패킷이 비어 올 수 있다
            s = link.recv_state()
            if s is not None:
                break
        else:
            raise SystemExit("상태 패킷을 못 받았다 — 시나리오가 로드/기동됐는지 확인")
    finally:
        try:
            link.close()
        except Exception:
            pass

    deg = math.degrees(s.heading) % 360.0
    if a.json:
        print(f'{{"x": {s.x:.3f}, "y": {s.y:.3f}, "heading_deg": {deg:.2f}}}')
    else:
        print(f"ego  x={s.x:.3f}  y={s.y:.3f}  heading={deg:.2f}°  (객체 {len(s.objects)}개)")
        print(f"\n경로 계산: python3 vtd/plan_route.py $X --heading {deg:.2f} "
              f"{s.x:.1f} {s.y:.1f} <다음 경유지 x y> ...")


if __name__ == "__main__":
    main()
