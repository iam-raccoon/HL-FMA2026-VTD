#!/usr/bin/env python3
"""처음 보는 코스를 만들어 **VTD 에서 실제로 돌려보기** 위한 시나리오 생성기.

왜 필요한가(2026-08-19): 회귀 18판이 전부 무결점이어도, 그건 **코스 3개**(v1~v6 공용 /
v7 / WP9)에서만 확인한 것이다. 대회 당일엔 처음 보는 좌표를 받는다. 그리고 오늘 하루에만
경로 생성기 버그가 3개 나왔다 — 셋 다 `check_route` ①~⑦ 을 통과한 채로.
**"오프라인으로 검증했다"는 두 번 틀렸다. 새 코스는 VTD 로 돌려서 확인한다.**
(세 번째 버그 — 없어지는 차로에 목표를 빼앗기는 것 — 을 잡은 게 바로 이 시험이다.)

대회 형식 그대로다: 사람이 X,Y 를 몇 개 불러준다([103:20] "X, Y, 그거를 저희가 드릴 겁니다").
같은 X,Y 로 두 가지를 만든다 —
  · VTD 시나리오의 `<Path>`  = 주최측이 깔아주는 경로(이게 있어야 리스폰 판정이 성립한다)
  · 우리 주행 경로            = `plan_route.py` 가 그 X,Y 만 보고 계산한 것
둘이 어긋나면 VTD 가 경로이탈로 리스폰을 건다 — 그게 이 시험으로 잡고 싶은 것 중 하나다.

⚠️ 시나리오를 **맨손으로 쓰지 않는다.** 실제로 도는 시나리오를 템플릿으로 열어 Path 와
   PathRef 만 갈아끼운다. VTD 가 기대하는 섹션(TrafficElements/PulkTraffic/LightSigns/
   Selections…)을 하나라도 빠뜨리면 로드가 실패하는데, 그 실패는 VTD 를 한 번 띄우고
   기다린 뒤에야 알 수 있다 — 한 번에 5분씩 버린다.

사용:
    python3 vtd/make_course.py <xodr> <템플릿.xml> x1 y1 x2 y2 [x3 y3 ...] > 새시나리오.xml
"""
import math
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from build_lane_plan import Map                                   # noqa: E402
from plan_route import plan                                       # noqa: E402

EDGE_KEEP = 5.0   # 경유지를 도로 끝에서 이만큼 떼어 놓는다[m]


def main():
    xodr, tmpl = sys.argv[1], sys.argv[2]
    v = [float(a) for a in sys.argv[3:]]
    if len(v) < 4 or len(v) % 2:
        raise SystemExit("좌표를 x y 쌍으로 2개 이상 주세요")
    pts = [(v[i], v[i + 1]) for i in range(0, len(v), 2)]
    mp = Map(xodr)

    # ★먼저 우리가 실제로 쓸 경로를 계산하고, CSV 지점을 그 경로 위에 **순서대로**
    # 투영한다. 교차로에서는 같은 X,Y를 여러 도로/연결로가 겹쳐 지나가므로
    # locate_lanes_all(...)[0]을 쓰면 반대편 도로나 엉뚱한 연결로가 선택될 수 있다.
    # 사전주행 1·2번은 출발점부터 각각 3131/-2, 1218/-3으로 잘못 잡혔지만 실제
    # 제어 경로는 2817/+3, 429/+2였다. Path와 제어 경로가 다르면 VTD 리스폰 대상이다.
    route, _path, meta = plan(mp, *pts)
    wps, first, begin = [], None, 0
    for x, y in pts:
        i = min(range(begin, len(route)),
                key=lambda k: math.dist((x, y), route[k]))
        begin = i
        rid, lid, s = meta[i][:3]
        # ⚠️ 경유지가 **도로 끝에 딱 붙으면** VTD 가 Path 를 못 만든다(방향이 모호하다).
        #    실측 2026-08-19: 주최측 예시 좌표 8개 중 둘이 s≈0 이었고
        #    `Config-Error Path: ("Path01") contains errors - quitting Path creation`
        #    이 났다. Path 가 없으면 PathRef 가 참조할 게 없어 **ego 가 월드 원점(0,0)에
        #    떨어진다** — 그 상태로도 주행은 되지만 시작 구간을 통째로 빼먹고 리스폰이 붙는다.
        # ★★**교차로 연결로는 Path 경유지가 될 수 없다.** VTD 가 통째로 거부한다:
        #     Config-Error Path: ("Path01", id: 1) contains errors - quitting path creation
        #     Config-Error for player "Ego": path with ID 1 not existing
        #   그러면 위 주석대로 **ego 가 월드 원점(0,0)에 떨어진다**(실측 2026-09-04).
        #   ⚠️ 주최측 좌표는 **일부러** 교차로 안에 있다 — 안내 2026-09-03:
        #      "경유지는 전체 패스 설정을 위한 **교차로 끝, 교차로 시작 지점**을 임의로
        #       피팅하여 제공". 사전테스트 경로 1 은 8개 중 **5개**가 연결로였다.
        #   그래서 연결로에 걸리면 **그 연결로가 빠져나가는 일반도로**로 옮겨 잡는다.
        #   contactPoint 가 그 도로의 어느 끝에 붙는지 알려준다.
        if mp.roads[rid].get("junction", "-1") != "-1":
            moved = None
            for tag in ("successor", "predecessor"):
                el = mp.roads[rid].find(f"link/{tag}")
                if el is None or el.get("elementType") != "road":
                    continue
                nid = el.get("elementId")
                if nid not in mp.roads or mp.roads[nid].get("junction", "-1") != "-1":
                    continue
                nL = float(mp.roads[nid].get("length"))
                ns = EDGE_KEEP if el.get("contactPoint") == "start" else nL - EDGE_KEEP
                moved = (nid, max(EDGE_KEEP, min(ns, nL - EDGE_KEEP)))
                break
            if moved is None:
                raise SystemExit(f"({x:.1f},{y:.1f}) 는 교차로 연결로 {rid} 인데 "
                                 f"이어지는 일반도로가 없다 — 좌표 확인")
            print(f"  경유지 ({x:.1f},{y:.1f}) 가 교차로 연결로 {rid} 위다 "
                  f"-> 일반도로 {moved[0]} s={moved[1]:.1f} 로 옮김", file=sys.stderr)
            rid, s = moved
        L = float(mp.roads[rid].get("length"))
        s = min(max(s, EDGE_KEEP), max(EDGE_KEEP, L - EDGE_KEEP))
        if first is None:
            first = (rid, lid, s)
        # ★같은 도로가 연달아 나오면 하나로 합친다 — VTD 는 같은 TrackId 를 되짚는
        #   Path 를 못 만든다(사전테스트 경로 1 은 2815 가 두 번 나왔다).
        # ★★**첫 도로만은 덮어쓰지 않는다**(2026-09-08). 합칠 때 뒤 경유지의 s 를 쓰는데,
        #   그게 첫 도로면 **ego 출발점이 통째로 바뀐다.**
        #   실측 사전테스트2: 경유지1·2 가 둘 다 road 429 로 잡혀(각각 s=216.4, s=5.0)
        #   출발점이 216.4 -> 5.0 으로 덮어써졌다. 그래서 차가 CSV 1번 지점(239.8,146.0)이
        #   아니라 **2번 지점 근처(324.3,-56.7)** 에 스폰됐고, drive.sh 가 "시나리오와 경로가
        #   다른 코스다(220m 차이)" 로 주행을 막았다. 그게 안전장치가 맞게 작동한 것이다.
        #   뒤쪽 도로는 그대로 덮어쓴다 — 거긴 '그 도로를 어디로 빠져나가나'라 뒤 값이 맞다.
        if wps and wps[-1][0] == rid:
            if len(wps) > 1:
                wps[-1] = (rid, s)
            continue
        wps.append((rid, s))

    # ★TargetS 는 **실제 경로 길이**로 잡는다. 직선거리로 어림하면 크게 모자란다 —
    #   일방통행 때문에 돌아가는 코스가 흔하다(실측: 직선 900m 코스의 실제 경로 5278m,
    #   어림값은 1956m 였다). 모자라면 경로가 도중에 끝나고 VTD 가 "not mapped to a
    #   path" 로 리스폰을 걸거나 Traffic 모듈이 죽는다.
    #   짧은 것만 위험하고 긴 건 무해하므로(EndAction="continue") 넉넉히 1.3배 준다.
    ours = sum(math.dist(route[i], route[i - 1]) for i in range(1, len(route)))
    target_s = ours * 1.3 + 200.0

    tree = ET.parse(tmpl)
    root = tree.getroot()
    path_el = next(root.iter("Path"), None)
    if path_el is None:
        raise SystemExit(f"{tmpl} 에 <Path> 가 없다 — Path 있는 시나리오를 템플릿으로 쓸 것")
    pid = path_el.get("PathId", "1")
    for c in list(path_el):
        path_el.remove(c)
    for rid, s in wps:
        ET.SubElement(path_el, "Waypoint",
                      {"PathOption": "shortest", "s": f"{s:.16e}", "TrackId": str(rid)})

    ego = next((p for p in root.iter("Player")
                if (p.find("Description") is not None
                    and p.find("Description").get("Name") == "Ego")), None)
    if ego is None:
        raise SystemExit(f"{tmpl} 에 Ego 플레이어가 없다")
    ref = ego.find("Init/PathRef")
    if ref is None:
        raise SystemExit(f"{tmpl} 의 Ego 에 PathRef 가 없다")
    ref.set("StartS", "3.0")
    ref.set("TargetS", f"{target_s:.3f}")
    ref.set("StartLane", str(abs(first[1])))
    ref.set("PathId", pid)
    ref.set("EndAction", "continue")

    # ★Ego 말고 다른 플레이어는 전부 뺀다. 템플릿의 상대차는 **그 코스** 좌표에 놓여
    #   있어서 새 코스에서는 엉뚱한 자리에 서 있다(길 한복판일 수도 있다).
    for parent in root.iter():
        for p in list(parent.findall("Player")):
            d = p.find("Description")
            if d is not None and d.get("Name") != "Ego":
                parent.remove(p)

    # ⚠️⚠️ **XML 선언을 반드시 붙인다.** 없으면 VTD 파서가
    #    `Testcase::xmlLoadScenario() - Error document empty` 를 내고 시나리오가 안
    #    올라간다(실측 2026-08-19). 게다가 이 스크립트는 결과를 **stdout 으로** 내보내므로
    #    plan() 이 찍는 진행 로그가 그대로 XML 앞에 섞여 파일을 깨뜨렸다 —
    #    아래 print 는 전부 stderr 로 보낸다(plan_route 쪽도 그렇게 고쳤다).
    sys.stdout.write("<?xml version='1.0' encoding='utf-8'?>\n")
    sys.stdout.write(ET.tostring(root, encoding="unicode"))
    sys.stdout.write("\n")
    sys.stderr.write(f"경유지 {len(pts)}개  출발 road {first[0]} lane {first[1]:+d} "
                     f"s={first[2]:.1f}  우리경로 {ours:.0f}m  TargetS={target_s:.0f}\n")


if __name__ == "__main__":
    main()
