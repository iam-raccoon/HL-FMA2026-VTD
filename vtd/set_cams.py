#!/usr/bin/env python3
"""VTD 카메라 시점 (taskControl 48179).

★ 대회 공식 시점 = "Relative - follower view" (강의 오후 95:19~, VTD DB에서 발견).
   VTD DB(Data/Distros/Distro/Config/VtGui/database.xml)의 정의:
     xyzRel = -6, 0, 2   (차 뒤 6m, 위 2m)
     hprRel = 0, 10, 0   (아래로 10도)
     showOwner = true

★★2026-09-06 실측 — 각도 속성 이름이 틀렸었다.
   우리는 `<ViewRelative h= p= r=>` 를 보내고 있었는데 VTD 가 받는 이름은
   **`dh` · `dp` · `dr`**(라디안)다. 즉 지금까지 **각도는 통째로 무시**되고
   위치만 먹었다 — 공식 시점의 '아래로 10도'가 실제로는 안 걸려 있었다.
   (근거: VTD 설치본의 시나리오 예제
    `Data/Projects/SampleProject/Scenarios/Parking.xml` 의 SCP 카메라 명령이
    `<ViewRelative dh="0.0" dp="0.872665" dr="0.0"/>` 를 쓴다. 0.872665rad = 50°.)
   고치고 나서 시점 전환이 실제로 동작하는 것을 화면으로 확인했다.

★확인용 옆면 시점(side / side_l) — 왜 필요한가:
   공식 follower 는 **차 뒤 6m·위 2m** 라 앞범퍼 바로 앞 1~2m 노면이 차체에 가린다.
   그래서 정지선 1.2m 앞에 서도 화면에서는 **선을 밟은 것처럼** 보인다.
   앞범퍼와 정지선의 실제 간격은 옆에서 봐야 판별된다.
   ⚠️ 연습에서만 쓴다. 대회 주행은 공식 시점이다 — 그래서 `test.sh` 로 분리해 뒀다.

⚠️ mainRS / Broadcast / LiDAR 세 창은 카메라 이름 "cam1"을 공유(각 *Display.xml).
   taskControl SCP는 브로드캐스트라 창별 개별 지정 불가 -> cam1 바꾸면 세 창 동일.
   즉 시점을 바꾸면 **RTSP 전방영상도 같이 바뀐다** — 녹화 중이면 감안할 것.
"""
import argparse
import math
import socket
import struct

MAGIC, VER = 40108, 1

# 이름 -> (dx, dy, dz, dh°, dp°, fovHor°)
#   차량 좌표: x 앞 · z 위. dy 는 **-가 차의 오른쪽**(실측으로 확인).
#   dp + = 아래로. 각도는 라디안으로 변환해 보낸다.
#   아이오닉6: 기준점=뒷축 중심, 앞범퍼 +3.808 · 뒷범퍼 -1.04
#   (VTD 차량 DB `Config/Players/Vehicles/HyundaiIoniq6_23.xml` 의 VehicleDef
#    DistFront/DistRear. 실차 오버행과도 맞는다.)
VIEWS = {
    # 대회 공식 — 이 값은 바꾸지 말 것.
    #  ⚠️ dp 는 **0** 이다. DB 정의는 '아래로 10도'지만, 지금까지 모든 실주행은
    #     각도가 무시된 상태(=0)로 돌았고 그 화면이 검증된 것이다. 실제로 dp=10°(0.1745rad)
    #     를 넣어 보니 화면이 차 밑 잔디를 보는 엉뚱한 그림이 됐다(2026-09-06 화면 확인).
    #     대회 이틀 전에 검증된 시점을 바꿀 이유가 없다. dh/dp/dr 을 명시해 두는 이유는
    #     **연습에서 옆면을 본 뒤 회전이 남아 있지 않게 되돌리기 위해서**다.
    "follower": (-6.0, 0.0, 2.0, 0.0, 0.0, 70.0),
    # 오른쪽 옆에서. 차 전체 + 앞범퍼 앞 노면이 한 화면에 (2026-09-06 화면 확인)
    "side":     (4.0, -18.0, 3.0, 90.0, 8.0, 45.0),
    # 왼쪽 옆에서 (반대쪽에 벽·나무가 있을 때)
    "side_l":   (4.0, 18.0, 3.0, -90.0, 8.0, 45.0),
    # README GIF 녹화용 — follower 자리에서 **8° 위로** 든다(dp -). 중계 창(mainRS_Broadcast)이
    #   follower 로는 차 앞 20 m 까지만 보여서 사물·보행자가 화면 위 끝에 잠깐만 걸렸다.
    #   이 값이면 차 전체와 앞 도로 100 m 가까이가 한 화면에 든다(2026-09-14 화면 확인).
    #   ⚠️ 주변 교통 판에서 뒤차가 6 m 안에 붙으면 카메라가 그 차 안에 들어간다.
    "gif":      (-6.0, 0.0, 2.0, 0.0, -8.0, 70.0),
}


def scp(xml, host="127.0.0.1", port=48179):
    d = xml.encode()
    h = struct.pack("<HH64s64sI", MAGIC, VER, b"cmd", b"any", len(d))
    s = socket.create_connection((host, port), timeout=5)
    s.sendall(h + d)
    s.close()


def set_view(view="follower", player="Ego", host="127.0.0.1"):
    dx, dy, dz, dh, dp, fov = VIEWS[view]
    # follower 는 예전과 **완전히 같은** 프러스텀을 쓴다(검증된 화면을 그대로 둔다). gif 도 같은 프러스텀.
    frustum = (f'<Frustum near="0.5" far="1500" fovHor="{fov}"/>' if view in ("follower", "gif")
               else f'<Frustum near="0.5" far="1500" fovHor="{fov}" fovVert="{fov * 0.6:.1f}"/>')
    scp(f'<Camera name="cam1" showOwner="true">'
        f'{frustum}'
        f'<PosRelative player="{player}" dx="{dx}" dy="{dy}" dz="{dz}"/>'
        f'<ViewRelative dh="{math.radians(dh):.6f}" dp="{math.radians(dp):.6f}" dr="0"/>'
        f'<Set/></Camera>', host=host)


def follower(player="Ego", host="127.0.0.1"):
    """대회 공식 Relative-follower-view: 뒤6·위2, 아래10도."""
    set_view("follower", player, host)


def main(argv=None):
    ap = argparse.ArgumentParser(description="VTD 카메라 시점")
    ap.add_argument("player", nargs="?", default="Ego", help="따라갈 플레이어 (기본 Ego)")
    ap.add_argument("--view", default="follower", choices=sorted(VIEWS),
                    help="시점. 기본 follower(대회 공식)")
    ap.add_argument("--host", default="127.0.0.1", help="VTD PC 주소 (기본 127.0.0.1)")
    a = ap.parse_args(argv)
    set_view(a.view, a.player, a.host)
    dx, dy, dz, dh, dp, fov = VIEWS[a.view]
    tag = "대회 공식" if a.view == "follower" else "연습 확인용"
    print(f"cam1(3창 공통) = {a.view} ({tag}) | dx={dx} dy={dy} dz={dz} "
          f"dh={dh}° dp={dp}° fov={fov}° | player={a.player} host={a.host}")


if __name__ == "__main__":
    main()
