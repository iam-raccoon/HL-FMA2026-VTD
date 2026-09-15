"""취약대상(VRU) 판정에 **속도**를 넣는다 — 62km/h 이륜차는 사람이 아니다.

실측 2026-09-05 코스 A t=465~478 (1284,-165): 2.0x0.6x1.7(VTD 보행자와 같은 치수)
물체가 62km/h 로 끼어들어 2km/h 까지 줄었다가 50 으로 다시 나갔다. 치수만 보면
'사람' 이라 YIELD_PED(앞범퍼 5m 정지)가 걸리고, 느려질 때마다 사람/차량이 뒤집혀
FOLLOW(a=-5) -> YIELD_PED -> NARROW_BLOCK+N -> CROSSWALK_STOP -> YIELD_PED,
38 -> 25 -> 37 -> 12 -> 20 -> 7 -> 1 -> 16km/h 로 12초 요동했다. 급제동 2회.
behavior.py 에 "보행자는 60km/h 로 못 간다" 고 적혀 있었지만 게이트가 없었다.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from vtd_io import is_vru, VRU_MAX_SPEED                              # noqa: E402
from drive import DrivingStack                                        # noqa: E402
from overtake import Overtaker                                        # noqa: E402

HERE = os.path.join(os.path.dirname(__file__), "..")
PED = (2.0, 0.6, 1.7)          # VTD 보행자 = 오토바이 치수


def test_치수만_보면_사람이다():
    assert is_vru(*PED) is True
    assert is_vru(*PED, None) is True


def test_차량_속도면_사람이_아니다():
    assert is_vru(*PED, 62 / 3.6) is False
    assert is_vru(*PED, VRU_MAX_SPEED + 0.01) is False


def test_걷거나_자전거_속도면_그대로_사람이다():
    assert is_vru(*PED, 1.5) is True             # 걷기
    assert is_vru(*PED, 25 / 3.6) is True        # 자전거
    assert is_vru(*PED, VRU_MAX_SPEED) is True   # 경계값 포함


def test_문턱은_자전거가_안_나오는_40kmh():
    assert abs(VRU_MAX_SPEED * 3.6 - 40.0) < 1e-6


def _row(spd, oid, ds=20.0):
    # rf_objs 행: (ds, d, spd, len, wid, hgt, oid, raw_w, raw_l)
    return (ds, 0.5, spd, 2.0, 0.6, 1.7, oid, 0.6, 2.0)


def test_drive는_한번_빨랐던_id를_느려져도_차량으로_둔다():
    """★요동의 원인. 이륜차가 앞에서 2km/h 로 기어갈 때 다시 사람으로 뒤집히면 안 된다."""
    d = DrivingStack()
    assert d._rf_is_vru(_row(62 / 3.6, 77)) is False
    assert d._rf_is_vru(_row(2 / 3.6, 77)) is False          # 같은 id, 느려짐 -> 차량 유지
    assert d._rf_is_vru(_row(2 / 3.6, 78)) is True           # 다른 id -> 사람


def test_behavior와_drive가_같은_set을_쓰고_reset이_비운다():
    d = DrivingStack()
    assert d.beh._fast_ids is d._fast_ids
    d._rf_is_vru(_row(62 / 3.6, 77))
    assert 77 in d.beh._fast_ids
    d.reset(0.0)
    assert not d._fast_ids and not d.beh._fast_ids


def test_overtaker도_속도를_본다():
    ov = Overtaker()
    # (id, ds, d, spd, len, wid, hgt, raw_w, raw_l)
    assert ov._is_vru((5, 20.0, 0.5, 62 / 3.6, 2.0, 0.6, 1.7, 0.6, 2.0)) is False
    assert ov._is_vru((5, 20.0, 0.5, 1.0, 2.0, 0.6, 1.7, 0.6, 2.0)) is True


def test_behavior의_두_판정_자리가_속도를_넘긴다():
    """YIELD_PED 본체와 CPA 가드 제외 — 둘 중 하나라도 치수만 보면 게이트가 새어 나간다."""
    src = open(os.path.join(HERE, "src", "behavior.py"), encoding="utf-8").read()
    assert "is_vru(raw_l, raw_w, ohgt, ospeed)" in src, "YIELD_PED 판정이 속도를 안 넘긴다"
    assert "is_vru(o.length, o.width, o.height, _sp)" in src, "CPA 가드 제외가 속도를 안 넘긴다"
    assert "is_vru(raw_l, raw_w, ohgt)\n" not in src, "속도 없는 is_vru 호출이 남아 있다"
