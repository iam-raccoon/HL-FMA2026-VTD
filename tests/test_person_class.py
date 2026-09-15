"""객체 분류(`vtd_io.is_vru`)를 **VTD 모델 카탈로그 실측값**으로 못박는다.

왜 이 파일이 있나: 판정식이 `behavior.py`·`overtake.py`·`drive.py`에 복사돼
있었다. 주석에 "한쪽만 바뀌면 모순" 이라고 적어두고도 실제로 어긋났고, 그 상태로
2026-09-02 에 세 개가 한꺼번에 드러났다:
  ① 휠체어(H=0.92)·자전거(H=1.10)를 사람으로 안 봤다 — 옆여유를 차량용 0.35m 만 뒀다
  ② 길가 표지판(0.04x0.60x2.10)을 사람으로 봤다 — 갓길 3~4m 에서 18km/h 로 깎였다
  ③ `overtake._is_ped` 가 높이를 안 봐서 포트홀·노면패치 56종이 사람이 됐다(죽은 코드였다)

치수 출처: VTD 2025.2 `Data/Distros/Current/Config/Players/{Pedestrians,Objects,Vehicles}`
전수 222개 (2026-09-02 실측).
"""
import ast
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from vtd_io import (ObjectClass, classify_object, is_person, is_vru)  # noqa: E402

#  (이름, L, W, H, 사람인가)
CATALOG = [
    # --- 사람이어야 하는 것
    ("male_adult",        0.60,  0.70,  1.80,  True),
    ("female_adult",      0.55,  0.63,  1.625, True),
    ("male_child",        0.50,  0.60,  1.35,  True),   # ★어린이도 잡아야 한다
    ("female_child",      0.50,  0.60,  1.425, True),
    ("DummyPerson",       0.234, 0.397, 1.401, True),   # 우리 시나리오가 쓰는 것
    ("실측 로그의 사람",     2.00,  0.60,  1.70,  True),   # 자전거 탄 사람으로 보인다
    ("wheelchair_adult",  1.01,  0.62,  0.92,  True),   # ★법상 보행자[제2조 17호 나목]
    ("bicycle_adult",     1.90,  0.65,  1.10,  True),   # ★
    ("Bike01(세워둔 자전거)", 2.22, 0.20,  1.15,  True),
    # --- 사람이면 안 되는 것 (낮은 소형물)
    ("라바콘 Pylon03",     0.30,  0.30,  0.32,  False),  # 2026-08-14 영구정지 사고
    ("라바콘(누움)",        0.30,  0.32,  0.30,  False),
    ("기름통 Fuelcan01",   0.15,  0.46,  0.61,  False),
    ("양동이 Bucket01",    0.32,  0.31,  0.36,  False),
    ("붕붕카 BobbyCar01",  0.96,  0.41,  0.64,  False),
    # --- 사람이면 안 되는 것 (길가 기둥 — 얇다)
    ("속도표지판",          0.04,  0.60,  2.10,  False),
    ("Sg205Vorfahrt",     0.04,  0.60,  2.50,  False),
    ("SgPole2.5m",        0.05,  0.05,  2.50,  False),
    ("RdPoleRedWhite",    0.08,  0.08,  1.05,  False),
    ("삽 Shovel01",        0.06,  0.28,  1.57,  False),
    ("SgParkRightSide01", 0.05,  0.43,  1.19,  False),
    # --- 사람이면 안 되는 것 (넓은 사물)
    ("손수레 WheelBarrow01", 1.01, 2.06,  0.85,  False),
    ("공사가드 RoadWork",    0.70,  2.30,  1.09,  False),
    ("WheelStack01",      0.82,  0.83,  0.94,  False),
    ("DummyCube",         1.00,  1.00,  1.00,  False),
    ("WorkBench01",       0.86,  1.79,  1.18,  False),
    # --- 차량
    ("Ioniq6",            4.85,  1.89,  1.507, False),
    ("버스",               6.70,  2.20,  2.80,  False),
    ("smart_fortwo",      2.51,  1.48,  1.520, False),
    ("Trailer",           4.54,  2.23,  1.240, False),
    # --- 노면 (포트홀·패치) — 죽은 `_is_ped` 가 사람으로 보던 것들
    ("RdPothole01",       0.614, 0.492, 0.005, False),
    ("Rd_Damage_Patch_02", 0.642, 0.570, 0.012, False),
    ("과속방지턱 RdSpeedBump02", 3.0, 6.0,  0.08,  False),
]


def test_대표_카탈로그가_기대대로_분류된다():
    bad = []
    for name, L, W, H, want in CATALOG:
        got = is_vru(L, W, H)
        if got != want:
            bad.append(f"{name} {L}x{W}x{H}: {got} (기대 {want})")
    assert not bad, "\n  ".join([""] + bad)


def test_카탈로그_222개_전부_정책대로_분류된다():
    """원본 222개를 실제로 읽는다. 대표 표본만 검사하고 '전수'라고 부르지 않는다."""
    path = os.path.join(HERE, "..", "reference", "vtd_2025.2_models.txt")
    rows = []
    for line in open(path, encoding="utf-8"):
        m = re.match(r"\s*([0-9.]+)\s+([0-9.]+)\s+([0-9.]+)\s+(.+?)\s+"
                     r"(사물|차량|보행자류)(?:\s|$)", line)
        if m:
            h, length, width = map(float, m.group(1, 2, 3))
            rows.append((m.group(4), m.group(5), length, width, h))
    assert len(rows) == 222, f"카탈로그 파싱 {len(rows)}/222"

    # VTD 폴더상 사물/차량이어도 충돌 시 사람이 다치는 대상은 VRU 여유를 적용한다.
    protected_objects = {"Bike01", "DummyPerson"}
    bad = []
    for name, catalog_class, length, width, height in rows:
        protected_vehicle = "[motorbike]" in name or name.startswith("Segway_2kw")
        want = (catalog_class == "보행자류"
                or name in protected_objects
                or protected_vehicle)
        got = is_vru(length, width, height)
        if got != want:
            bad.append(f"{name}: {got} (기대 {want})")
    assert not bad, "\n  ".join([""] + bad)


def test_정책분류는_취약대상_차량_사물_노면을_나눈다():
    assert classify_object(0.60, 0.70, 1.80) is ObjectClass.VRU
    assert classify_object(4.85, 1.89, 1.507) is ObjectClass.VEHICLE
    assert classify_object(0.63, 0.58, 1.27) is ObjectClass.OBSTACLE  # RubbishBin
    assert classify_object(0.614, 0.492, 0.005) is ObjectClass.ROAD_SURFACE


def test_옛_is_person은_is_vru와_호환된다():
    samples = [(0.60, 0.70, 1.80), (1.01, 0.62, 0.92), (4.85, 1.89, 1.507)]
    assert all(is_person(*dims) == is_vru(*dims) for dims in samples)


def test_어린이는_어른과_똑같이_잡힌다():
    """VTD 어린이 모델은 1.35m·1.425m 다. 문턱 1.2 를 올리면 놓친다."""
    assert is_vru(0.50, 0.60, 1.35)
    assert is_vru(0.50, 0.60, 1.425)


def test_라바콘_문턱에_여유가_있다():
    """2026-08-14: 라바콘(H=0.61 기름통 포함)을 사람으로 보고 영구정지했다.
    취약자용 문턱 0.85 는 그 위로 0.24m 떨어져 있다."""
    from vtd_io import VRU_MIN_H
    assert VRU_MIN_H > 0.61 + 0.2, VRU_MIN_H
    assert not is_vru(0.30, 0.30, 0.32)
    assert not is_vru(0.15, 0.46, 0.61)


def test_사람_최소_길이_문턱이_실제_사람보다_작다():
    """길가 기둥을 빼려고 L 하한을 뒀다. 제일 마른 사람 모델(DummyPerson 0.234)보다
    작아야 사람을 안 놓친다."""
    from vtd_io import PERSON_MIN_L
    assert PERSON_MIN_L < 0.234, PERSON_MIN_L
    assert PERSON_MIN_L > 0.11, "표지판 기둥(0.04~0.11)은 걸러야 한다"


def test_판정이_한_군데뿐이다():
    """behavior 세 군데 + overtake 한 군데 + **drive 두 군데**에 복사돼 있던 걸 합쳤다.
    다시 갈라지면 "저쪽은 사람이라 세우는데 이쪽은 차라 추월한다" 가 재발한다.

    ⚠️ 실제로 두 번 갈라졌다. 2026-09-02 에 behavior·overtake 만 합치고 "한 군데뿐"
       이라고 적었는데 `drive.py` 의 `_lat_free`·`_nudge_offset` 이 아직
       `ohgt >= 1.2` 로 따로 판정하고 있었다 — **이 테스트가 그 두 파일만 봐서
       놓쳤다.** 그래서 아래는 이름이 아니라 **패턴**으로 본다.
    """
    for f in ("behavior.py", "overtake.py", "drive.py"):
        src = open(os.path.join(HERE, "..", "src", f), encoding="utf-8").read()
        tree = ast.parse(src, filename=f)
        calls = [n for n in ast.walk(tree)
                 if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Name)
                 and n.func.id == "is_vru"]
        assert calls, f"{f}: is_vru() 호출이 없다"

        # 예전 분기에서 사용한 독립 높이 문턱이 이름만 바꿔 되살아나는 것도 막는다.
        stale = {"ped_min_h", "NUDGE_MAX_H"}
        names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
        attrs = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
        assert not stale.intersection(names | attrs), f


def test_치수로_사람을_직접_판정하는_코드가_없다():
    """★이름을 바꿔 가며 다시 갈라지는 걸 막는다. 높이/폭 상수와 비교해서 사람인지
    가리는 식은 `vtd_io.is_vru` 밖에 있으면 안 된다."""
    import re
    # 높이로 보이는 변수를 1.2 언저리 상수와 비교하는 식
    pat = re.compile(r"\b(ohgt|o\.height|hgt|height)\s*[<>]=?\s*"
                     r"(0\.8[0-9]|0\.9[0-9]|1\.[0-4][0-9]?|self\.\w*(?:_h|_H)\b)")
    bad = []
    for f in ("behavior.py", "overtake.py", "drive.py"):
        path = os.path.join(HERE, "..", "src", f)
        for i, line in enumerate(open(path, encoding="utf-8"), 1):
            if line.lstrip().startswith("#"):
                continue                      # 주석은 설명이라 봐준다
            if pat.search(line):
                bad.append(f"{f}:{i}  {line.strip()[:80]}")
    assert not bad, ("치수로 사람을 직접 가리는 코드가 남아 있다 — "
                     "`vtd_io.is_vru` 를 쓸 것:\n  " + "\n  ".join(bad))


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_"):
            continue
        try:
            fn()
            print(f"  ✅ {name}")
        except AssertionError as e:
            fails += 1
            print(f"  ❌ {name}: {e}")
    print("실패 없음" if not fails else f"{fails}건 실패")
    sys.exit(1 if fails else 0)
