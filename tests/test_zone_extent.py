"""보호구역(붉은 노면)을 **도로 전체가 아니라 표시 s 구간**으로, 그리고 **진행방향별**로 본다.

실측 2026-09-05 코스 A 사용자 지적:
  ④ road 2312(290m)는 RM_536/518 표시가 s=14~75 에만 있는데 도로 전체를 30 으로
     달렸다. 사용자가 본 자리 s=116~256 은 붉지 않았다.
  ⑤ (1150,-607) 횡단보도가 가장 가까운 표시에서 260m 인데 옛 300m 반경에 걸려
     보호구역으로 서 있었다. 붉지 않은 곳이다.
실측 2026-09-06 코스 H·E 사용자 지적:
  H-3 road 2818(476m): 유일한 표시 s=378 은 **+s 방향 차로의 것**인데 -s 차로 s=402~462 를
      30 으로 달렸다("빨간 도로 아닌데 30"). '도로 전체' 확장은 표시가 향한 방향에만.
  E-2 road 2264: 표시가 하나도 없는데 보호구역 교차로(junction 60) 쪽 s≈34 까지 붉었다
      ("빨간 도로인데 50"). 표시 없는 팔은 교차로 쪽 ZONE_ARM_REACH 를 구역으로.
"""
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "vtd"))
from build_lane_plan import (zone_extent, in_zone, ZONE_MARGIN,       # noqa: E402
                             ZONE_CW_REACH, ZONE_CLUSTER, ZONE_ARM_REACH,
                             lane_dir)

PI = "3.14159"
XML = f"""
<roads>
  <road id="A" length="290" junction="-1"><objects>
    <object name="RM_536.flt" s="14" t="2.1" hdg="{PI}"/><object name="RM_518.flt" s="20" t="2.0" hdg="{PI}"/>
    <object name="RM_518.flt" s="59" t="-1.6" hdg="0"/><object name="RM_536.flt" s="75" t="1.9" hdg="{PI}"/>
  </objects></road>
  <road id="B" length="476" junction="-1"><objects>
    <object name="RM_518.flt" s="378" t="-1.5" hdg="0"/><object name="RM_518.flt" s="378" t="-4.0" hdg="0"/>
  </objects></road>
  <road id="C" length="30" junction="7"><link>
    <predecessor elementType="road" elementId="A" contactPoint="start"/>
    <successor elementType="road" elementId="E" contactPoint="end"/></link></road>
  <road id="D" length="30" junction="8"><link>
    <predecessor elementType="road" elementId="A" contactPoint="end"/></link></road>
  <road id="E" length="100" junction="-1"/>
  <road id="F" length="100" junction="-1"><link>
    <predecessor elementType="junction" elementId="7"/></link></road>
  <road id="G" length="60" junction="-1"><link>
    <predecessor elementType="junction" elementId="7"/>
    <successor elementType="junction" elementId="7"/></link></road>
  <road id="H" length="100" junction="-1"><link>
    <successor elementType="junction" elementId="8"/></link></road>
  <road id="J" length="30" junction="9"><link>
    <predecessor elementType="road" elementId="B" contactPoint="end"/></link></road>
  <road id="K" length="100" junction="-1"><link>
    <predecessor elementType="junction" elementId="9"/></link></road>
</roads>"""


class _Map:
    def __init__(self):
        self.roads = {r.get("id"): r for r in ET.fromstring(XML).iter("road")}


def test_군집이_둘이면_양방향_모두_표시_범위로_좁힌다():
    ext = zone_extent(_Map())
    assert ext[("A", 1)] == ext[("A", -1)] == (0.0, 75 + ZONE_MARGIN), ext      # 14-15 -> 0 으로 깎임


def test_군집이_하나면_표시가_향한_방향만_도로_전체():
    """road 2818 케이스 — +s 표시 하나. +s 는 모르면 넓게, -s 는 표시 ±여유만."""
    ext = zone_extent(_Map())
    assert ext[("B", 1)] == (0.0, 476.0)
    assert ext[("B", -1)] == (378 - ZONE_MARGIN, 378 + ZONE_MARGIN)


def test_2818_반대방향은_표시에서_멀면_50():
    mp = _Map(); ext = zone_extent(mp)
    assert in_zone(mp, ext, "B", 430.0, dir=-1) is False      # H-3: s=402~462 는 붉지 않았다
    assert in_zone(mp, ext, "B", 430.0, dir=+1) is True       # 표시 방향은 그대로 넓게
    assert in_zone(mp, ext, "B", 430.0) is True               # 방향 모르면 넓게(횡단보도 판정)
    assert in_zone(mp, ext, "B", 380.0, dir=-1) is True       # 표시 바로 옆은 반대 방향도 붉다고 본다


def test_차로_번호로_진행방향을_읽는다():
    assert lane_dir(-1) == 1 and lane_dir(-3) == 1            # 우측통행: 음수 차로 = +s
    assert lane_dir(1) == -1 and lane_dir(2) == -1
    assert lane_dir(None) == 1


def test_표시_없는_도로는_구역이_아니다():
    mp = _Map(); ext = zone_extent(mp)
    assert ("E", 1) not in ext and ("E", -1) not in ext
    assert in_zone(mp, ext, "E", 50.0) is False


def test_표시_도로_안팎():
    mp = _Map(); ext = zone_extent(mp)
    assert in_zone(mp, ext, "A", 50.0) is True
    assert in_zone(mp, ext, "A", 200.0) is False               # ④: s=116~256 은 50
    assert in_zone(mp, ext, "A", 95.0) is False                # 90 넘음
    assert in_zone(mp, ext, "A", 95.0, ZONE_CW_REACH) is True  # 횡단보도용 여유로는 안


def test_연결로는_붉은_끝에_붙었을_때만():
    mp = _Map(); ext = zone_extent(mp)
    assert in_zone(mp, ext, "C", 5.0) is True     # A 의 s=0 끝(구간 0~90 안)에 붙음
    assert in_zone(mp, ext, "D", 5.0) is False    # A 의 s=290 끝 — 구간에서 200m 떨어짐 (⑤ 류)


def test_보호구역_교차로에_붙은_표시없는_팔은_교차로쪽_90m():
    """E-2: road 2264(90m) — 표시 없음. 처음 40m 로 잡았더니 반대쪽 끝 직후부터도 붉었다
    (2026-09-06 재주행 A·E). 2312 의 표시 구간도 교차로에서 90m — 이 맵의 구역 길이다."""
    mp = _Map(); ext = zone_extent(mp)
    assert ZONE_ARM_REACH == 90.0
    assert ext[("F", 1)] == ext[("F", -1)] == (0.0, ZONE_ARM_REACH)
    assert in_zone(mp, ext, "F", 60.0) is True
    assert in_zone(mp, ext, "F", 95.0) is False
    assert ("H", 1) not in ext                     # junction 8 은 보호구역 교차로가 아니다(D 가 안 닿음)


def test_한_군집_도로의_전체_확장으로는_교차로를_보호구역으로_만들지_않는다():
    """junction 89: road 2818 의 유일한 표시(s=378)는 끝(476)에서 98m — '도로 전체' 확장(안전측)으로
    닿았을 뿐인데 팔 4개가 30 이 됐다. 사용자(코스 E): "여기 왜 30 으로 감??" — 붉지 않았다.
    B(476m) 의 끝은 junction 9 라 치고, 거기 붙은 표시 없는 도로 K 는 구역이 아니어야 한다."""
    mp = _Map(); ext = zone_extent(mp)
    assert ext[("B", 1)] == (0.0, 476.0)           # 표시 방향은 여전히 넓게
    assert ("K", 1) not in ext                     # 그러나 그 끝 교차로의 팔은 구역이 아니다


def test_양끝이_보호구역_교차로면_사이도_구역():
    ext = zone_extent(_Map())
    assert ext[("G", 1)] == (0.0, 60.0)


def test_군집_문턱이_2312_패턴을_둘로_가른다():
    """{14,20} 과 {59,75}: 간격 39m > ZONE_CLUSTER 라 두 군집이어야 한다."""
    assert 20 < ZONE_CLUSTER < 39
