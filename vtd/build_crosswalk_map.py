#!/usr/bin/env python3
"""xodr -> **횡단보도 DB** (어린이보호구역 무신호 횡단보도 일시정지용).

왜 필요한가(2026-08-20, 사용자 지적 "골목길 횡단보도 정지선에서 왜 안 멈춤?"):
우리 코드는 **신호등(`tl_id`)만** 본다. 그래서 신호 있는 사거리에서는 서지만 신호 없는
골목 횡단보도는 존재 자체를 모른다. 그런데 —

  **도로교통법 제27조⑦** (2022-01-11 신설)
  > 어린이 보호구역 내에 설치된 횡단보도 중 **신호기가 설치되지 아니한** 횡단보도 앞
  > (정지선이 설치된 경우에는 그 정지선)에서는 **보행자의 횡단 여부와 관계없이 일시정지**

코스 A 실측: 경로 8m 안 횡단보도 50곳, 전부 30km/h 구간, 그중 **무신호 24곳**.
지금은 한 곳도 안 선다.

맵에서 읽는 것(전부 xodr `objects/object`):
  · `RM_532_*`  (195개, 6×8m·6×9m)          = **횡단보도 본체**
  · `Rm_StopLine_300cm_JPN_01` (710개, 차로별) = **실제 도색 정지선**
  · `Rm_Warning_Crosswalk_JPN_03` (400개)     = 횡단보도 **예고** 표시(본체 아님, 안 씀)

⚠️ '보호구역인가'는 `road_speed_limits` 의 30km/h 로 본다. 이 맵은 보호구역을
   `RM_536`+`RM_518` 로 표시하고 표시가 없는 도로는 **직전 값 유지**라, 도로 단위로
   물으면 새는 곳이 생긴다. 그래서 **주행 경로의 차로계획 `lim`** 으로 판정하는 게 맞다
   — 그건 이 스크립트가 아니라 쓰는 쪽(`build_lane_plan`)이 이미 갖고 있다.
   여기서는 좌표와 '신호 있음/없음'만 낸다.

사용:
    python3 vtd/build_crosswalk_map.py <xodr> [tl_map.json] > routes/crosswalks.json
"""
import json
import math
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from build_lane_plan import (Map, road_point, road_speed_limits_net,   # noqa: E402
                             zone_extent, in_zone, ZONE_CW_REACH)
from plan_route import sample_lane                                  # noqa: E402

CW_PREFIX = "RM_532_"        # 횡단보도 본체
STOPLINE = "Rm_StopLine_300cm_JPN_01"
SIG_NEAR = 25.0              # 신호 정지선이 이 안에 있으면 '신호 있는 횡단보도'
# ⚠️ 도색 정지선은 **월드 좌표**로 찾는다. 처음엔 '같은 road 의 s 가 30m 안'으로 찾았는데
#    195곳 중 **24곳(12%)** 밖에 못 잡았다 — 횡단보도와 그 정지선이 서로 다른 road 에
#    걸쳐 있는 경우가 대부분이기 때문이다(교차로 진입로 vs 교차로 안).
#    월드 좌표로 재보니 **179곳이 10m 안**에 정지선을 갖고 있었다.
#    사용자 지적 2026-08-20: "정지선은 무시하고 그냥 횡단보도 앞에서 멈추는데".
LINE_NEAR = 12.0             # 횡단보도에서 이 반경 안의 도색 정지선을 그 횡단보도 것으로 본다


def lane_center(mp, rid, s):
    """도로 rid 의 s 지점 차로 중심(양방향 중 먼저 잡히는 쪽)."""
    rd = mp.roads[rid]
    L = float(rd.get("length"))
    s = min(max(s, 0.5), max(0.5, L - 0.5))
    for lid in (-1, 1, -2, 2):
        out = []
        try:
            sample_lane(mp, rid, lid, max(0.0, s - 0.5), min(L, s + 0.5), out)
        except Exception:
            continue
        if out:
            return out[len(out) // 2]
    return None


def stoplines(mp, with_road=False):
    """도색 정지선의 **월드 좌표 + 바라보는 방향** 전부 -> [(x, y, hdg), ...].
    `with_road=True` 면 [(rid, s, x, y, hdg), ...] — 보호구역 판정(도로·s 기준)용.

    ⚠️ 2026-08-20 이전에는 `lane_center(rid, s)` 로 좌표를 냈다. 그건 **객체가 어느 차로
       것인지를 무시**하고 늘 -1 차로만 샘플링해서, 710개 정지선이 **서로 다른 점 281개로
       뭉쳤다**(429개가 남의 자리로 갔다). 3차로 도로의 1·2·3차로 정지선이 전부 같은
       좌표가 되니 횡거리 필터(`CW_LAT`)가 무의미해진다.
       객체는 `t`(기준선 기준 횡오프셋)를 갖고 있으므로 **기준선 위 점 + t** 로 바로 낸다.

    ★`hdg` 는 그 정지선이 **세우는 차량의 진행방향**이다(실측: road 30 t=-1.5 -> hdg 0,
      road 62 t=+4.5 -> hdg pi. 즉 우측차로용은 +s, 좌측차로용은 -s). 이걸 쓰면
      '진입쪽 선'과 '건너편 선'을 거리 비교 같은 어림짐작 없이 **정확히** 가를 수 있다.
    """
    out = []
    for rid, rd in mp.roads.items():
        L = float(rd.get("length"))
        for o in rd.findall("objects/object"):
            if (o.get("name") or "").split(".")[0] != STOPLINE:
                continue
            s = min(max(float(o.get("s", 0)), 0.0), max(0.0, L))
            t = float(o.get("t", 0.0))
            try:
                x, y, h = road_point(rd, s)
            except Exception:
                continue
            # 기준선 법선 방향으로 t 만큼(OpenDRIVE: t 양수 = 진행 왼쪽)
            wx, wy, wh = x - t * math.sin(h), y + t * math.cos(h), h + float(o.get("hdg", 0.0))
            out.append((rid, s, wx, wy, wh) if with_road else (wx, wy, wh))
    return out


def main():
    xodr = sys.argv[1]
    tl = json.load(open(sys.argv[2], encoding="utf-8")) if len(sys.argv) > 2 else {}
    sig = [(v[0], v[1]) for v in tl.values()]
    mp = Map(xodr)
    # ★그 횡단보도가 **보호구역인가**를 여기서 같이 낸다(2026-08-25).
    #   예전엔 "도로 단위로 물으면 샌다"며 쓰는 쪽에 맡겼는데, 그 '쓰는 쪽'이
    #   **자차 현재 위치**의 제한속도로 판정하고 있었다. 그러면 50 구간에서 30 구간으로
    #   **들어가는 입구의 횡단보도**는 경계를 넘기 전까지 존재 자체가 안 보인다.
    #   실측 2026-08-25 코스 E road 2488: 49km/h 로 접근하는 내내 무시되다가
    #   경계를 넘은 순간(자차 제한 50->30) 켜졌는데 그땐 이미 **1.4m 지나쳐** 있었다.
    #   제한속도 전파를 도로망 기준으로 고친 뒤로는 **도로 단위 값이 곧 정답**이다.
    lim_of = road_speed_limits_net(mp)
    # ★`zone` 은 `lim` 과 **따로** 낸다. 제27조⑦ 은 붉은 노면 위가 아니라
    #   **보호구역 안** 무신호 횡단보도에 걸리는 법적 의무다. 2026-09-04 에
    #   주최측 답변대로 `lim` 을 '붉은 노면만 30' 으로 좁혔더니, 그걸 그대로
    #   쓰면 구역 안쪽 무신호 횡단보도 일시정지가 통째로 사라진다.
    # ★2026-09-05: 도로 단위 300m 반경(zone_reach_roads)에서 **표시 s 구간 ± 30m** 로
    #   좁혔다. 실측 코스 A 사용자 지적 ⑤: (1150,-607) 횡단보도가 가장 가까운 표시에서
    #   260m 인데 반경 300 에 걸려 보호구역으로 서 있었다. 붉은 노면이 아니다.
    _ext = zone_extent(mp)

    SL = stoplines(mp)
    out = []
    for rid, rd in mp.roads.items():
        objs = rd.findall("objects/object")
        for o in objs:
            if not (o.get("name") or "").startswith(CW_PREFIX):
                continue
            s = float(o.get("s", 0))
            # ⚠️ 본체 좌표도 정지선과 **같은 버그**를 갖고 있었다(2026-08-20 2차).
            #    lane_center(rid, s) 는 늘 -1 차로에 찍는데, 실제 객체는 t 를 갖는다
            #    (road 2574 의 두 횡단보도는 t=+1.39, 즉 기준선 **왼쪽**). 그래서 저장 좌표가
            #    실제보다 3m 가량 옆으로 밀렸고, 주행 중 '내 앞 6m 안'에 안 들어와
            #    **미정지**로 남았다. 기준선 점 + t 로 바로 낸다.
            try:
                rx, ry, rh = road_point(rd, s)
            except Exception:
                continue
            t = float(o.get("t", 0.0))
            p = (rx - t * math.sin(rh), ry + t * math.cos(rh))
            # ★정지선이 있으면 **그 앞**에 선다(조문: "정지선이 설치된 경우에는 그 정지선").
            #   ⚠️ 여기서 **하나로 고르면 안 된다.** 횡단보도 양쪽에 정지선이 있어
            #      가장 가까운 것을 집으면 **건너편 것**을 잡는다(실측 2026-08-20 코스 A:
            #      24곳 중 10곳이 그랬다). DB 는 우리 진행방향을 모르므로 **후보를 전부**
            #      싣고, 주행 중에 '내 앞이면서 횡단보도보다 가까운 것'을 고르게 한다.
            near = sorted((math.hypot(p[0] - q[0], p[1] - q[1]), i)
                          for i, q in enumerate(SL))
            stops = [[round(SL[i][0], 3), round(SL[i][1], 3), round(SL[i][2], 4)]
                     for d0, i in near if d0 <= LINE_NEAR]
            d_sig = min((math.hypot(p[0] - a, p[1] - b) for a, b in sig), default=1e9)
            out.append({
                "road": rid, "s": round(s, 2),
                "x": round(p[0], 3), "y": round(p[1], 3),
                "stops": stops,                          # 근처 도색 정지선 후보(월드좌표)
                "signal": bool(d_sig <= SIG_NEAR),      # 근처에 신호 정지선이 있나
                "lim": round(lim_of[rid], 3) if rid in lim_of else None,  # 그 도로의 제한[m/s]
                "zone": 1 if in_zone(mp, _ext, rid, s, ZONE_CW_REACH) else 0,   # 제27조⑦
            })

    out.sort(key=lambda c: (c["road"], c["s"]))
    json.dump({"crosswalks": out}, sys.stdout, ensure_ascii=False, indent=1)
    sys.stdout.write("\n")
    n_sig = sum(1 for c in out if c["signal"])
    n_line = sum(1 for c in out if c["stops"])
    n_zone = sum(1 for c in out if c.get("zone"))
    sys.stderr.write(f"횡단보도 {len(out)}곳 — 신호 있음 {n_sig} · 무신호 {len(out)-n_sig} "
                     f"· 도색 정지선 있음 {n_line} · 보호구역(30) {n_zone}\n")


if __name__ == "__main__":
    main()
