#!/usr/bin/env python3
"""녹화 영상에서 **그 순간**의 장면을 꺼낸다 — 주행 CSV 의 t 로 찾는다.

    python3 vtd/grab_at.py <run.mp4> <run.csv> <출력디렉터리> [reason=CROSSWALK_STOP] [개수]
    python3 vtd/grab_at.py <run.mp4> <run.csv> <출력디렉터리> --t 123.4 200.0

왜(2026-08-22): 채점기는 '정지선 앞 2.3m 에 섰다'까지 말해주지만, 그게 **화면에서
맞아 보이는지**는 못 본다. 녹화와 CSV 를 맞추면 그 프레임을 직접 볼 수 있다.

⚠️ 영상과 CSV 는 **시작 시각이 다르다**(녹화를 먼저 켜니까). `--offset` 으로 맞춘다.
   맞추는 법: 자차가 처음 움직이는 순간을 양쪽에서 찾아 그 차이를 넣는다.
"""
import csv
import subprocess
import sys
import os


def main():
    mp4, csvp, outdir = sys.argv[1], sys.argv[2], sys.argv[3]
    rest = sys.argv[4:]
    offset = 0.0
    if "--offset" in rest:
        i = rest.index("--offset")
        offset = float(rest[i + 1]); rest = rest[:i] + rest[i + 2:]
    os.makedirs(outdir, exist_ok=True)
    if rest and rest[0] == "--t":
        times = [(float(t), f"t{t}") for t in rest[1:]]
    else:
        want = rest[0] if rest else "CROSSWALK_STOP"
        n = int(rest[1]) if len(rest) > 1 else 12
        rows = list(csv.DictReader(open(csvp, encoding="utf-8")))
        times, prev = [], None
        for r in rows:
            if r.get("reason") == want and float(r["v"]) < 0.5:
                t = float(r["t"])
                if prev is None or t - prev > 3.0:     # 같은 정지에서 한 장만
                    times.append((t, f"{want}_{len(times)+1:02d}"))
                    prev = t
            if len(times) >= n:
                break
    for t, tag in times:
        dst = os.path.join(outdir, f"{tag}_t{t:.1f}.png")
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", f"{max(0.0, t + offset):.2f}",
                        "-i", mp4, "-frames:v", "1", dst], check=False)
        print(f"  {dst}")
    if not times:
        print("해당하는 순간이 없다")


if __name__ == "__main__":
    main()
