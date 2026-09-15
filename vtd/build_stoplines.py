#!/usr/bin/env python3
"""xodr -> **고아 정지선 DB** (`routes/stoplines.json`).

사용자 지적 2026-08-20: "정지선 무시". 실제로 코스 A 경로 위에, **신호등도 횡단보도도
없는 도색 정지선**이 6곳 있고 우리는 전부 그냥 지나쳤다. 우리 코드가 보는 정지선은
  · 신호 정지선(tl_map)      · 횡단보도에 붙은 정지선(crosswalks.json)
둘뿐이라, 어디에도 안 붙은 정지선은 존재 자체를 모른다.

  도로교통법 시행규칙 별표6, 노면표시 530 '정지선'
  > 차가 **정지하여야 할 지점**을 표시하는 것

확인한 것(2026-08-20): 이 6곳은 가장 가까운 신호등이 30.7~201.7m 로, 신호에 딸린
정지선이 아니다. 가장 가까운 횡단보도도 15m 밖이다.

★★2026-09-05: **그 전제가 틀렸다. 이제 양보 표시가 붙은 것만 남긴다(106 -> 2개).**

  위 "도색만 보고 일시정지로 단정" 은 8/20 당시 표지 유무를 확인 못 해서 안전측으로
  둔 것이다. 9/5 에 확인해 보니:
    · **정지선(노면표시 530) 자체에는 정지의무가 없다.** 대법원 전원합의체가
      일시정지 표지(227·521)와 명확히 구분했다 — 530 은 "정지를 해야 할 경우
      정지해야 할 **지점**을 표시하는 것으로서 그 표시 자체에 의하여 정지의무가
      있음을 표시하는 것은 아니다".
    · **이 맵에 일시정지 노면표시·표지는 0개다.** 양보(Rm_Give_Way) 23개뿐이다
      (RM_510~512 는 일시정지가 아니라 **화살표** 표시였다 — 한 번 헷갈렸다).
    · 대회 채점 15개 항목에 '정지선 미준수' 가 없다. 7·9 는 신호등 전용이고
      12 는 오히려 '횡단보도 **위** 정지' 감점이다.
    · 비용: 코스당 20~160초. 주최측 사전테스트 경로 2번에서도 40초.
  보행자 보호는 이것과 **무관하게** 계속 돈다 — 제27조①은 `YIELD_PED`(보호구역
  무관, 물체 기준)가, 제27조⑦은 `crosswalks.json` 의 `zone` 이 맡는다.
  사용자 결정 2026-09-05(안 A).

  ⚠️ 양보(Give_Way)는 엄밀히 '일시정지' 가 아니라 '양보' 다. 그래도 2곳뿐이라
     비용이 0 에 가까워 서는 쪽으로 남긴다.

★★2026-09-05(같은 날 저녁) **보호구역 안의 고아 정지선은 되살린다.** 사용자 결정:
  "어린이 보호구역에선 정지선(횡단보도 없는 정지선) 지켜야 하는 거 아님?" -> "저건 해야
  하는 거잖아". 법 조문만으로는 의무가 없지만(제27조⑦은 횡단보도 얘기다) 보호구역은
  심판이 주시하는 곳이고, 비용은 코스 A 6곳 ≈ 60초, 주최측 경로에서는 거의 0 이다.
  '보호구역 안' 의 정의는 횡단보도 zone 과 **같다**(`in_zone` + ZONE_CW_REACH) —
  두 판정이 다른 지도를 보면 서로 다른 답을 낸다.

사용: python3 vtd/build_stoplines.py <xodr> <tl_map.json> <crosswalks.json> > routes/stoplines.json
      python3 vtd/build_stoplines.py <xodr> --all > routes/stoplines_all.json

★`--all` (2026-09-04): **거르지 않은 도색 정지선 전부**(710개). 신호에 딸린 것까지 담는다.
  왜: VTD 가 보고하는 tl_id 중 **xodr 에 controller 가 없는 것**이 있다 —
  실측으로 tl 80(2895 프레임)·82(290 프레임). 그러면 `tl_stops` 조회가 비어
  `_tl_stop_dist` 가 None 을 돌려주고 적신호가 `RED_STOP_BLIND`(정지선 미상 -> 그 자리
  정지)로 간다. 대회에서 이건 길 한복판 정지다. 이 DB 로 **눈앞의 도색선**을 찾아 메운다.
  ⚠️ 이 파일은 `stoplines.json`(고아 정지선) 을 **대체하지 않는다**. 역할이 다르다.
     고아 = "무조건 선다", 전체 = "모르는 신호일 때 정지선 위치만 빌린다".
"""
import json
import math
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from build_crosswalk_map import stoplines                          # noqa: E402
from build_lane_plan import (road_point, zone_extent, in_zone,     # noqa: E402
                             ZONE_CW_REACH)
from build_lane_plan import Map                                    # noqa: E402

NEAR_SIG = 15.0      # 신호 정지선이 이 안이면 신호용 -> 제외
NEAR_CW = 15.0       # 횡단보도가 이 안이면 횡단보도용 -> 제외
GIVE_WAY = "Rm_Give_Way_JPN_03"     # 이 맵의 유일한 양보 노면표시
NEAR_GW = 20.0       # ★양보 표시가 이 안에 있는 것만 남긴다 — 아래 2026-09-05 참고


def main():
    if "--all" in sys.argv:
        mp = Map(sys.argv[1])
        out = [[round(x, 3), round(y, 3), round(h, 4)] for x, y, h in stoplines(mp)]
        json.dump({"stoplines": out}, sys.stdout, ensure_ascii=False)
        sys.stdout.write("\n")
        sys.stderr.write(f"도색 정지선 전체 {len(out)}개\n")
        return
    xodr, tlp, cwp = sys.argv[1], sys.argv[2], sys.argv[3]
    tl = json.load(open(tlp, encoding="utf-8"))
    cw = json.load(open(cwp, encoding="utf-8"))["crosswalks"]
    sig = [(v[0], v[1]) for v in tl.values()]
    mp = Map(xodr)
    gw = []                                   # 양보 노면표시 좌표
    for rd in mp.roads.values():
        L = float(rd.get("length"))
        for ob in rd.findall("objects/object"):
            if (ob.get("name") or "").split(".")[0] != GIVE_WAY:
                continue
            gs = min(max(float(ob.get("s", 0)), 0.0), max(0.0, L))
            gt = float(ob.get("t", 0.0))
            try:
                gx, gy, gh = road_point(rd, gs)
            except Exception:
                continue
            gw.append((gx - gt * math.sin(gh), gy + gt * math.cos(gh)))
    ext = zone_extent(mp)
    out, n_all, n_gw, n_zone = [], 0, 0, 0
    for rid, s_, x, y, h in stoplines(mp, with_road=True):
        if min((math.hypot(x - a, y - b) for a, b in sig), default=1e9) <= NEAR_SIG:
            continue
        if min((math.hypot(x - c["x"], y - c["y"]) for c in cw), default=1e9) <= NEAR_CW:
            continue
        n_all += 1
        # ★남기는 조건 둘(위 docstring 근거):
        #   (a) 양보 표시가 붙어 있다   (b) 보호구역 안이다(횡단보도 zone 과 같은 정의)
        #   정지선 도색만으로는 정지의무가 없고, 이 맵에 일시정지 표시는 하나도 없다.
        near_gw = min((math.hypot(x - a, y - b) for a, b in gw), default=1e9) <= NEAR_GW
        # ★정지선이 **세우는 방향**(도색 hdg vs 도로 방위)으로 그 방향의 구역을 본다(2026-09-06).
        try:
            _rh = road_point(mp.roads[rid], s_)[2]
            _dir = 1 if math.cos(h - _rh) >= 0.0 else -1
        except Exception:
            _dir = None
        zone = in_zone(mp, ext, rid, s_, ZONE_CW_REACH, dir=_dir)
        if not (near_gw or zone):
            continue
        n_gw += near_gw; n_zone += zone
        out.append([round(x, 3), round(y, 3), round(h, 4)])
    sys.stderr.write(f"신호·횡단보도에 안 붙은 정지선 {n_all}개 중 {len(out)}개 남김 "
                     f"(양보 표시 {n_gw} · 보호구역 안 {n_zone})\n")
    json.dump({"stoplines": out}, sys.stdout, ensure_ascii=False)
    sys.stdout.write("\n")
    sys.stderr.write(f"고아 정지선 {len(out)}개 (전체 정지선 710개 중)\n")


if __name__ == "__main__":
    main()
