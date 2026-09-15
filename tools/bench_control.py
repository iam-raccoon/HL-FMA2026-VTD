#!/usr/bin/env python3
"""제어 스택이 **이 기계에서 25Hz(40ms)를 지킬 수 있나**를 실측한다.

    python3 tools/bench_control.py runs_tr/run_E_TR.csv HL_FMA_NEW_E

왜 필요한가(2026-08-25, 사용자 질문 "제어PC를 라파3B+가 하면 힘들려나"):
답을 CPU 스펙으로 어림하면 틀린다. 우리 부하는 **경로 길이에 좌우**되는데
코스마다 3배 넘게 차이난다(코스 E 3768점은 다른 코스의 2~3배 무겁다).
그러니 **그 기계에서 그 코스를 실제로 돌려 보는 것**만이 답이다.

이 스크립트는 VTD 없이, 기록된 주행 CSV 를 그대로 되먹여 `stack.step()` 만
호출하고 프레임당 시간을 잰다. 순수 stdlib 라 파이·젯슨 어디서든 돈다.

⚠️ 재는 것은 **연산**뿐이다. 실전에서 40ms 를 깨는 다른 원인(네트워크 지터,
   USB 랜, 열 스로틀링)은 여기서 안 잡힌다 — 아래 '판정'의 여유율을 그만큼
   보수적으로 읽을 것.
⚠️ CSV 의 objs 에는 상대 차량의 **방위와 id 가 없다**(로거가 안 남긴다). 방위는
   자차 방위로, id 는 순번으로 채운다. 판단 결과는 원본과 달라질 수 있지만
   **지나가는 코드 경로와 객체 수는 같아서** 연산량 측정에는 영향이 없다.
"""
import csv
import gc
import json
import math
import os
import platform
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))

from vtd_io import Obj, State                                    # noqa: E402
from scenario import Scenario                                    # noqa: E402
from drive import DrivingStack                                   # noqa: E402

BUDGET_MS = 40.0            # 25Hz 제어 주기


def load_route_files(course):
    """`HL_FMA_NEW_E` 같은 코스 이름 -> drive.sh 와 같은 짝의 파일들."""
    r = os.path.join(ROOT, "routes")
    base = course
    for suf in ("_EV", "_HZ", "_TR"):
        if base.endswith(suf):
            base = base[: -len(suf)]
    return {
        "scenario": os.path.join(r, base + ".json"),
        "lane": os.path.join(r, base + "_lane.json"),
        "tl": os.path.join(r, "tl_map_livinglab.json"),
        "cw": os.path.join(r, "crosswalks.json"),
        "sl": os.path.join(r, "stoplines.json"),
    }


def rows_from_csv(path):
    """기록 CSV -> State 목록. objs 는 자차기준이므로 월드로 되돌린다."""
    out = []
    with open(path, encoding="utf-8") as f:
        for i, r in enumerate(csv.DictReader(f)):
            s = State()
            s.t = float(r["t"])
            s.x, s.y = float(r["x"]), float(r["y"])
            s.heading = float(r["heading"])
            s.speed = float(r["v"])
            s.tl_id = int(r["tl_id"])
            s.tl_state = int(r["tl_state"])
            c, sn = math.cos(s.heading), math.sin(s.heading)
            objs = []
            for k, tok in enumerate((r.get("objs") or "").split("|")):
                p = tok.split(":")
                if len(p) < 8:
                    continue
                try:
                    fx, fy, ln, wd, ht, sp = (float(p[j]) for j in range(6))
                except ValueError:
                    continue
                objs.append(Obj(k + 1, s.x + fx * c - fy * sn, s.y + fx * sn + fy * c,
                                0.0, s.heading, sp, ln, wd, ht))
            s.objects = objs
            out.append(s)
    return out


def synth_states(stack, n_obj=5, v=8.33, hz=25.0):
    """CSV 없이 **경로만으로** 상태 흐름을 만든다.

    왜: 기록 CSV 는 수십 MB 라 남의 기계로 옮기기 번거롭다. 벤치의 목적은
    '이 기계가 40ms 를 지키나' 뿐이고, 그 부하는 **경로 길이와 객체 수**가 정한다.
    둘 다 여기서 그대로 재현되므로 실제 CSV 판과 같은 코드 경로를 지난다.
    ⚠️ 판단 결과는 실제 주행과 다르다 — **성능 측정 전용**이다.
    ⚠️ 실측 대조(라파5 유휴, 코스 E): 합성 p99 1.61ms vs 실제 CSV 2.00ms —
       꼬리에서 **약 20% 낙관적**이다. '되나 안 되나'를 가르는 데는 충분하지만,
       여유율이 10배 언저리로 나오면 실제 CSV 로 다시 볼 것.
    """
    pts = stack.route
    step = v / hz
    out, t, i, acc = [], 0.0, 0, 0.0
    while i < len(pts) - 1:
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        if seg < 1e-9:
            i += 1
            continue
        u = acc / seg
        s = State()
        s.t, s.speed = t, v
        s.x, s.y = ax + (bx - ax) * u, ay + (by - ay) * u
        s.heading = math.atan2(by - ay, bx - ax)
        c, sn = math.cos(s.heading), math.sin(s.heading)
        # 실제 판의 평균 객체 수(4.6개)에 맞춰 앞뒤·좌우로 흩어 놓는다. 일부는 움직인다.
        objs = []
        for k in range(n_obj):
            fx = 8.0 + 11.0 * k + 6.0 * math.sin(t * 0.7 + k)
            fy = -3.2 + 1.6 * ((k * 7) % 5)
            objs.append(Obj(k + 1, s.x + fx * c - fy * sn, s.y + fx * sn + fy * c,
                            0.0, s.heading, 6.0 if k % 2 else 0.0,
                            4.4 if k % 3 else 2.0, 1.8 if k % 3 else 0.6,
                            1.5 if k % 3 else 1.7))
        s.objects = objs
        out.append(s)
        t += 1.0 / hz
        acc += step
        while i < len(pts) - 1 and acc >= math.hypot(pts[i + 1][0] - pts[i][0],
                                                     pts[i + 1][1] - pts[i][1]):
            acc -= math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            i += 1
    return out


def pct(a, q):
    if not a:
        return 0.0
    return a[min(len(a) - 1, max(0, int(len(a) * q)))]


def main():
    # ★CSV 없이도 돌게 한다 — 남의 기계(파이·젯슨)에 27MB 를 옮기지 않아도 되도록.
    if len(sys.argv) == 3 and sys.argv[1] == "--synth":
        csv_path, course = None, sys.argv[2]
    elif len(sys.argv) >= 3:
        csv_path, course = sys.argv[1], sys.argv[2]
    else:
        print(__doc__)
        print("사용: python3 tools/bench_control.py <run.csv> <코스이름>")
        print("      python3 tools/bench_control.py --synth <코스이름>   (CSV 없이)")
        print("  예: python3 tools/bench_control.py --synth HL_FMA_NEW_E")
        raise SystemExit(2)
    f = load_route_files(course)
    for k in ("scenario", "lane"):
        if not os.path.exists(f[k]):
            print(f"❌ 없는 파일: {f[k]}")
            raise SystemExit(1)

    t0 = time.perf_counter()
    sc = Scenario.load(f["scenario"])
    tl = {int(k): v for k, v in json.load(open(f["tl"], encoding="utf-8")).items()} \
        if os.path.exists(f["tl"]) else {}
    lane = json.load(open(f["lane"], encoding="utf-8"))["pts"]
    cw = json.load(open(f["cw"], encoding="utf-8"))["crosswalks"] \
        if os.path.exists(f["cw"]) else None
    sl = json.load(open(f["sl"], encoding="utf-8"))["stoplines"] \
        if os.path.exists(f["sl"]) else None
    stack = DrivingStack(scenario=sc, base_limit=8.33, tl_stops_extra=tl,
                         lane_plan=lane, crosswalks=cw, stoplines=sl)
    t_setup = time.perf_counter() - t0

    states = synth_states(stack) if csv_path is None else rows_from_csv(csv_path)
    if not states:
        print("❌ 빈 CSV")
        raise SystemExit(1)

    # ★★재기 전에 **GC 를 한 번 돌린다.** 이 벤치는 15,756프레임을 통째로 메모리에
    #   올려두는데(객체 8만 개), 그 더미에 대한 첫 gen-2 수집이 루프 도중에 터져
    #   **한 프레임에 30ms 로 찍혔다**(실측 2026-08-25 라파5: 최대 29.8/31.7ms).
    #   그건 스택이 아니라 이 벤치의 부작용이다 — 실제 제어 루프는 프레임을 쌓지 않는다.
    #   미리 걷어내면 같은 기계 같은 입력에서 **최대 2.26ms** 다.
    gc.collect()
    # ★첫 프레임은 캐시·분기 예열 때문에 유난히 느리다. 통계에서 뺀다(대신 따로 찍는다).
    times, nobj, err = [], 0, 0
    warm = None
    for i, s in enumerate(states):
        dt = 0.04 if i == 0 else max(1e-3, s.t - states[i - 1].t)
        nobj += len(s.objects)
        a = time.perf_counter()
        try:
            stack.step(s, dt, s.t)
        except Exception:
            err += 1
        b = time.perf_counter()
        if i == 0:
            warm = (b - a) * 1000.0
        else:
            times.append((b - a) * 1000.0)
    times.sort()

    mach = f"{platform.machine()} · {platform.python_version()}"
    try:
        with open("/proc/cpuinfo") as fh:
            for ln in fh:
                if ln.startswith(("model name", "Model")):
                    mach = ln.split(":", 1)[1].strip() + " · " + mach
                    break
    except OSError:
        pass

    p50, p95, p99, mx = pct(times, .5), pct(times, .95), pct(times, .99), times[-1]
    print(f"\n기계     : {mach}")
    print(f"코스     : {course}  (경로 {len(stack.route)}점 · {stack.total:.0f}m)"
          + ("  [합성 입력]" if csv_path is None else ""))
    print(f"프레임   : {len(times)}  · 평균 객체 {nobj / len(states):.1f}개"
          + (f"  ⚠️ step 예외 {err}프레임" if err else ""))
    print(f"준비시간 : 로딩·구성 {t_setup * 1000:.0f}ms · 첫 프레임 {warm:.1f}ms (1회성)")
    print(f"프레임당 : 중앙 {p50:.2f}ms · p95 {p95:.2f}ms · p99 {p99:.2f}ms · 최대 {mx:.2f}ms")
    print(f"예산     : {BUDGET_MS:.0f}ms (25Hz)")
    # ⚠️ **판정을 `최대`로 하면 안 된다.** 최대 프레임은 그 기계에서 같이 도는 다른 일
    #    (GC·스케줄러·ssh·복사)에 통째로 좌우된다 — 실측 2026-08-25: 같은 기계 같은
    #    입력으로 두 번 돌렸더니 최대가 15.8ms -> 4.9ms 로 바뀌었고 튀는 프레임 위치도
    #    달랐다. p99 는 두 판에서 2.13 / 2.17ms 로 안정적이었다.
    #    그래서 **p99 로 판정**하고, 최대는 참고로만 찍는다.
    head = BUDGET_MS / p99 if p99 > 0 else 999
    verdict = ("✅ 넉넉" if head >= 8 else "⚠️ 빠듯 — 열·네트워크 여유가 없다"
               if head >= 4 else "❌ 부족")
    print(f"여유율   : p99 기준 **{head:.0f}배**   -> {verdict}")
    print(f"  (최대 {mx:.1f}ms 는 그 기계에서 같이 돈 다른 일에 좌우된다 — 판정에 안 쓴다.")
    print("   연산 외에 네트워크 지터·열 스로틀링이 더 먹으니 8배 이상을 권한다)")


if __name__ == "__main__":
    main()
