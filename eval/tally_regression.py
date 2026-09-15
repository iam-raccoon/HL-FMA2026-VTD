#!/usr/bin/env python3
"""회귀 로그 대신 CSV 를 직접 채점해 한 줄씩 요약한다.

사용: python3 eval/tally_regression.py $(date -d '21:22' +%s)
     인자 = 이번 회귀 시작 시각(epoch). 그보다 오래된 CSV 는 무시한다.
"""
import glob
import os
import re
import subprocess
import sys

REPO = "/home/user/hlfma2026"
R = REPO + "/routes"
V1 = f"{R}/planned_from_xml_v1.json"
ROUTE = {"HL_FMA_VTD_LivingLab_v7": f"{R}/planned_v7.json",
         "HL_FMA_WP9": f"{R}/planned_wp9.json"}
ORDER = ([f"HL_FMA_VTD_LivingLab_v{i}" for i in range(1, 8)] + ["HL_FMA_WP9"] +
         ["EV_LEADBRAKE", "EV_CUTIN", "EV_PED", "EV_OBSTACLE", "EV_ONCOMING",
          "EV_CURVE", "EV_REARPASS", "EV_BOTHBLOCK", "EV_COMBO_PASSPED",
          "EV_COMBO_CHAIN"])


def g(pat, s, d="-"):
    m = re.search(pat, s)
    return m.group(1) if m else d


print(f"{'판':<20}{'시간':>6}{'거리':>7}{'커버':>6}{'종점':>6} {'신호':>4}{'리스폰':>6}"
      f"{'접촉':>5}{'속도위반':>8}{'최소여유':>9}  판정")
n_ok = 0
n = 0
for name in ORDER:
    f = f"/tmp/run_{name}.csv"
    if not os.path.exists(f) or os.path.getsize(f) < 1000:
        continue
    # ★이번 회귀 것만. /tmp 에 몇 달치 옛 판이 쌓여 있어 그냥 읽으면 **옛 결과를 보고한다**
    #   (실제로 오늘 오후 3시 판이 섞여 나왔다 — 종점 오차 9m 가 옛 코드 표식이었다).
    if os.path.getmtime(f) < float(sys.argv[1]):
        continue
    rt = ROUTE.get(name, V1)
    o = subprocess.run([sys.executable, f"{REPO}/eval/check_run.py", f, "8.33",
                        rt, rt.replace(".json", "_lane.json")],
                       capture_output=True, text=True).stdout
    n += 1
    ok = "완주 ✅" in o and "신호위반 0건" in o and "리스폰 0건" in o and "접촉 0프레임" in o
    n_ok += ok
    vals = [
        g(r"\uc2dc\uac04 (\d+)s", o) + "s",
        g(r"\uc8fc\ud589 (\d+)m", o) + "m",
        g(r"\ucee4\ubc84\ub9ac\uc9c0\] (\d+%)", o),
        g(r"\ubaa9\ud45c\uc5d0\uc11c (\d+)m", o) + "m",
        g(r"\uc2e0\ud638\uc704\ubc18 (\d+)\uac74", o),
        g(r"\ub9ac\uc2a4\ud3f0\] (\d+)\uac74", o),
        g(r"\uc811\ucd09 (\d+)\ud504\ub808\uc784", o),
        g(r"\ucd08\uacfc\(10%\u2191\) \ud504\ub808\uc784 (\d+)/", o),
        g(r"\ucd5c\uc18c ([+-][0-9.]+)m", o) + "m",
    ]
    short = name.replace("HL_FMA_VTD_LivingLab_", "").replace("HL_FMA_", "")
    verdict = "\ubb34\uacb0\uc810" if ok else "\u26a0\ufe0f " + o.strip().splitlines()[-1][:60]
    print("{:<20}{:>6}{:>7}{:>6}{:>6}{:>4}{:>6}{:>5}{:>8}{:>9}  {}".format(
        short, *vals, verdict))
