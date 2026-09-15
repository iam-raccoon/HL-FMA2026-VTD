#!/usr/bin/env python3
"""아무 코스에나 **취약대상(VRU) 판**을 심는다 -> `scenarios/<코스>_VRU.xml`.

사용자 요청 2026-09-09: "대회장에 자전거·휠체어가 나오면 긴급제동이나 회피를 할 수
있는지 확인을 못 했다. 일단 VTD 에서 돌려봐야 확실할 것 같다."

오프라인(eval/scenarios/vru_*.json)으로 먼저 돌려 **결함 하나**를 찾았고,
이건 그걸 VTD 실제 코스에서 다시 보기 위한 것이다. 오프라인 판의 도로는 직선 한 줄이라
'차로를 옮길 자리가 정말 있나'를 못 본다 — 그게 여기서만 확인된다.

심는 것(경로 길이 비율로 자리를 잡고, 각 자리는 **교차로 밖 + 옆 여유 3.2m 이상**):

    0.25  Wheelchair  내 차로 중앙에 **정지한 휠체어**(H 0.92)
    0.45  Bicycle     내 차로 중앙에 **정지한 자전거**(H 1.10)
    0.65  SlowLead    **4.5m/s(16km/h) 로 굴러가는 선행차**

앞의 둘은 오프라인에서 통과한 동작(옆 차로로 옮겨 통과)을 실제 지도에서 확인하는 것이고,
셋째는 오프라인에서 **재현된 결함**을 확인하는 것이다:
  실측 2026-09-09 `eval/scenarios/vru_bicycle_slow.json` — 왼쪽 차로가 통째로 비어 있는데
  자전거 뒤 15.4m 에서 16km/h 로 끝까지 따라가 **미완주(85점)**.
  같은 판에서 막는 것만 승용차로 바꿔도 결과가 같다(16.6m, 미완주). 즉 자전거만의
  문제가 아니라 `overtake.py:201` 의 `spd > 1.0` 이 **움직이는 것 전부**를 추월 대상에서
  빼기 때문이다. SlowLead 가 그걸 실제 코스에서 확인한다.

    python3 vtd/add_vru.py <xodr> <코스이름>
      예: python3 vtd/add_vru.py $X HL_FMA_NEW_A  -> scenarios/HL_FMA_NEW_A_VRU.xml

⚠️ **오브젝트는 움직이지 않는다.** 보행자류는 Player 로 스폰되지 않아
   (`add_hazards.py` 주석: Type="Hannah" 실패, SCP Player|Create 도 실패)
   MovingObjectsControl 오브젝트로 넣는데, 그건 제자리에 서 있기만 한다.
   그래서 여기서 VTD 로 보는 것은 **정지한 VRU 회피**뿐이다.
   '뛰어드는 휠체어에 급제동'은 오프라인 판(`vru_wheelchair_dart`)이 담당한다.
⚠️ **모델 이름이 뜨는지는 VTD 에서 확인해야 안다.** 카탈로그
   (`reference/vtd_2025.2_models.txt`)에 있는 이름을 그대로 쓰지만, 그게
   MovingObjectsControl 의 `Definition` 으로 받아들여지는지는 실측 전이다.
   화면에 아무것도 안 보이면 `--def-wheelchair DummyPerson` 처럼 바꿔서 다시 만든다.
   (그때는 치수가 성인 보행자가 되므로 **H 0.92 검증은 안 된 것**이다 — 반드시 기록할 것.)
⚠️ 자리는 `add_events.pick_blocker` 와 같은 기준으로 고른다 — 교차로 안에 놓으면
   자차가 갇혀서 판이 무의미해진다(`add_hazards.py` 주석의 실측).
"""
import argparse
import json
import math
import shutil
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from add_oncoming import add                                        # noqa: E402
from add_hazards import elevation, OBJ_Z_LIFT                       # noqa: E402
from build_lane_plan import Map                                     # noqa: E402
from plan_route import locate_lanes_all                             # noqa: E402

HERE = __file__.rsplit("/", 1)[0]
NEED_ROOM = 3.2      # 놓을 곳에 필요한 옆 여유[m] — 이보다 좁으면 비켜 갈 길이 없다
SLOW_LEAD_V = 4.5    # 저속 선행차 속도[m/s] = 16km/h (오프라인 재현판과 같은 값)


def cumdist(rt):
    c = [0.0]
    for i in range(1, len(rt)):
        c.append(c[-1] + math.dist(rt[i], rt[i - 1]))
    return c


def pick_spot(rt, lp, cum, frac, used, min_sep=60.0):
    """교차로 밖 + 옆 여유가 있는 자리를 `frac` 지점부터 앞으로 찾는다.

    ★`used` 에 이미 쓴 자리들을 넘겨 **서로 min_sep 이상 떨어지게** 한다.
      셋이 붙어 있으면 하나를 피하는 동작이 다음 것에 걸려 무엇 때문에 멈췄는지
      구분이 안 된다 — 판이 섞이면 진단이 안 된다.
    """
    start = min(range(len(rt)), key=lambda i: abs(cum[i] - cum[-1] * frac))
    for i in range(start, len(rt) - 20):
        p = lp[i] if i < len(lp) else None
        if not p or p.get("j") or p.get("jx"):
            continue
        if max(p.get("l", 0.0) or 0.0, p.get("r", 0.0) or 0.0) < NEED_ROOM:
            continue
        if any(abs(cum[i] - cum[j]) < min_sep for j in used):
            continue
        h = math.atan2(rt[i + 3][1] - rt[i][1], rt[i + 3][0] - rt[i][0])
        return i, rt[i][0], rt[i][1], math.degrees(h) % 360.0
    return None


def add_object(in_xml, out_xml, mp, x, y, hdg, definition, name, z_lift=OBJ_Z_LIFT):
    """MovingObjectsControl 에 오브젝트 하나를 넣는다(`add_hazards.add_ped` 와 같은 방식).

    Z 는 노면 고도 + z_lift 다. 기본 OBJ_Z_LIFT 는 기름통·사람 기준 — 그냥 노면으로 주면 반쯤 묻혀
    안 보인다(`add_hazards.py` 의 OBJ_Z_LIFT 주석에 실측 근거가 있다). 모델마다 원점이 달라서
    WheelStack01 은 0 이어야 노면에 붙는다(0.74 면 그림자와 떨어져 떠 보인다, 2026-09-14 VTD 화면).
    """
    c = locate_lanes_all(mp, x, y)
    z = (elevation(mp.roads[c[0][0]], c[0][2]) + z_lift) if c else 0.0
    src = open(in_xml, encoding="utf-8").read()
    obj = (f'<Object Type="other" Name="{name}" Definition="{definition}">'
           f'<StartPosAbs X="{x:.16e}" Y="{y:.16e}" Z="{z:.16e}" '
           f'Direction="{(hdg + math.pi / 2) % (2 * math.pi):.16e}" Pitch="0.0" Roll="0.0"/>'
           f'</Object><ObjectActions Object="{name}"/>')
    if "<MovingObjectsControl />" in src:
        src = src.replace("<MovingObjectsControl />",
                          f"<MovingObjectsControl>{obj}</MovingObjectsControl>", 1)
    elif "</MovingObjectsControl>" in src:
        src = src.replace("</MovingObjectsControl>", obj + "</MovingObjectsControl>", 1)
    else:
        print(f"⚠️ MovingObjectsControl 앵커가 없다 — {name} 생략", file=sys.stderr)
    open(out_xml, "w", encoding="utf-8").write(src)
    return z


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("xodr")
    ap.add_argument("course")
    ap.add_argument("--def-wheelchair", default="wheelchair_adult",
                    help="휠체어 모델명. VTD 에서 안 보이면 DummyPerson 등으로 바꾼다")
    ap.add_argument("--def-bicycle", default="bicycle_adult",
                    help="자전거 모델명. VTD 에서 안 보이면 바꾼다")
    ap.add_argument("--no-slow-lead", action="store_true",
                    help="저속 선행차를 빼고 정지 VRU 둘만 심는다")
    a = ap.parse_args()

    rt = json.load(open(f"{HERE}/../routes/{a.course}.json", encoding="utf-8"))["ego_route"]
    lp = json.load(open(f"{HERE}/../routes/{a.course}_lane.json", encoding="utf-8"))["pts"]
    mp = Map(a.xodr)
    cum = cumdist(rt)

    src = f"{HERE}/../scenarios/{a.course}.xml"
    out = f"{HERE}/../scenarios/{a.course}_VRU.xml"
    used, made, stage, n = [], [], src, 0

    def tmp():
        nonlocal n
        n += 1
        return f"/tmp/_vru_{n}.xml"

    for frac, definition, nm in ((0.25, a.def_wheelchair, "Wheelchair"),
                                 (0.45, a.def_bicycle, "Bicycle")):
        spot = pick_spot(rt, lp, cum, frac, used)
        if spot is None:
            print(f"⚠️ {nm}: 교차로 밖 + 여유 {NEED_ROOM}m 자리를 못 찾았다 — 생략",
                  file=sys.stderr)
            continue
        i, x, y, hd_deg = spot
        used.append(i)
        dst = tmp()
        z = add_object(stage, dst, mp, x, y, math.radians(hd_deg), definition, nm)
        stage = dst
        made.append(f"{nm}[{definition}] s={cum[i]:.0f}m Z={z:.1f}")

    if not a.no_slow_lead:
        spot = pick_spot(rt, lp, cum, 0.65, used)
        if spot is None:
            print("⚠️ SlowLead: 자리를 못 찾았다 — 생략", file=sys.stderr)
        else:
            i, x, y, hd_deg = spot
            used.append(i)
            dst = tmp()
            # ⚠️ 트리거를 크게 잡으면 시나리오 초반에 걸려 **그대로 달려가 사라진다**
            #    (`add_hazards.py` 의 trig 경고, 실측 2026-08-24). 짧게 잡아야 마주친다.
            add(stage, dst, x, y, hd_deg, SLOW_LEAD_V, 70.0, name="SlowLead")
            stage = dst
            made.append(f"SlowLead[{SLOW_LEAD_V}m/s] s={cum[i]:.0f}m")

    if stage == src:
        raise SystemExit(f"{a.course}: 아무것도 못 심었다")
    shutil.copy(stage, out)
    print(f"-> {out}\n   심은 것: " + "\n              ".join(made))


if __name__ == "__main__":
    main()
