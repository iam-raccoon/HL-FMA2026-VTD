#!/usr/bin/env python3
"""제어PC에서 실행: 주행 CSV를 실시간으로 읽어 조건이 되면 VTD에 이벤트를 SCP로 주입.

⚠️ 9910(제어 링크)은 단일 클라이언트라 절대 건드리면 안 된다. 그래서 상태는
   main.py 가 쓰는 --log-csv 파일을 tail 해서 얻고, 명령만 48179(SCP)로 보낸다.

사용: python3 inject_event.py <csv> <VTD_IP> lanechange <actor> <발동거리m> [direction]
      python3 inject_event.py <csv> <VTD_IP> brake      <actor> <발동거리m> [rate]
"""
import socket, struct, sys, time, os

def scp(xml, host, port=48179):
    d = xml.encode()
    h = struct.pack("<HH64s64sI", 40108, 1, b"cmd", b"any", len(d))
    s = socket.create_connection((host, port), timeout=5); s.sendall(h + d); s.close()

def main():
    csvp, host, kind, actor = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
    trig = float(sys.argv[5]); arg = float(sys.argv[6]) if len(sys.argv) > 6 else 1.0
    t0 = time.time()
    while not os.path.exists(csvp) and time.time() - t0 < 30:
        time.sleep(0.2)
    f = open(csvp, encoding="utf-8"); f.readline()
    while time.time() - t0 < 300:
        line = f.readline()
        if not line:
            time.sleep(0.05); continue
        p = line.rstrip("\n").split(",")
        if len(p) < 13:
            continue
        try:
            fx, nl = float(p[9]), float(p[12])
        except ValueError:
            continue
        if nl > 2.0 and 0 < fx < trig:              # 전방 '차량'이 발동거리 안으로
            if kind == "lanechange":
                scp(f'<Traffic><ActionLaneChange actor="{actor}" direction="{int(arg)}" time="2.5"/></Traffic>', host)
                scp(f'<Traffic><ActionSpeedChange actor="{actor}" target="3.0" rate="3.0"/></Traffic>', host)
            else:
                scp(f'<Traffic><ActionSpeedChange actor="{actor}" target="0.0" rate="{arg}"/></Traffic>', host)
            print(f"[inject] {kind} 발동 (fx={fx:.1f}m, t={p[0]}s)")
            return
    print("[inject] 조건 미충족으로 종료")

main()
