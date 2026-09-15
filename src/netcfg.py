"""VTD 호스트 주소 한 곳 — 대회망과 연습망을 **자기 IP로 자동 판별**한다.

[대회 안내문 2026-08-27 · 3. 사전 준비 — 네트워크 설정]
    제어기 PC (참가팀 PC)   192.168.50.10
    VTD PC   (Host)        192.168.50.11
    서브넷 255.255.255.0 · 같은 대역
    포트 9910 TCP(양방향) · 8554 RTSP(제어->Host) · 9912 UDP(Host->제어)
  -> **포트는 우리 값 그대로다.** 바뀌는 건 IP 대역뿐이다.

★2026-09-04 부터 **연습망도 대회 주소로 통일**했다 — 제어PC 192.168.50.10 /
   OMEN 직결랜 192.168.50.11. 대회날 IP 를 바꿀 일 자체를 없앴다(랜선 꽂는 순간
   15분이 돌고, 제어기 작동 뒤에는 제어PC 를 만지면 실격이다).
   192.168.100.x 는 **되돌렸을 때를 위한 예비**로만 남겨둔다.

⚠️ 그래도 자동 판별을 지우지 않는 이유: 대역을 되돌리거나 다른 PC 를 쓸 때
   기본값이 한쪽으로 박혀 있으면 **반드시 한쪽에서 틀린다**. 제어PC 자기 주소가
   곧 답이므로 그걸 보고 고른다. 못 고르면 대회값으로 떨어진다(실패 비용이 크다).

강제로 정하려면 `VTD_HOST=192.168.100.1` 처럼 환경변수를 준다.
망을 맞추거나 점검하는 건 `bash drive.sh net [fix]` 다.
"""
import os
import socket
import subprocess

# (제어PC 대역 접두사, VTD 호스트) — 위에서부터 자기 IP 와 맞춰 본다
NETS = [
    ("192.168.50.", "192.168.50.11"),    # 대회 (안내문)
    ("192.168.100.", "192.168.100.1"),   # 연습 직결랜
]
COMPETITION_HOST = "192.168.50.11"
PORT = 9910


def local_ipv4s():
    """이 PC 의 IPv4 주소 목록. 실패하면 빈 리스트."""
    out = []
    try:
        r = subprocess.run(["ip", "-4", "-o", "addr"], capture_output=True,
                           text=True, timeout=2)
        for line in r.stdout.splitlines():
            parts = line.split()
            if "inet" in parts:
                out.append(parts[parts.index("inet") + 1].split("/")[0])
    except Exception:
        pass
    if not out:                       # ip 명령이 없는 환경 대비
        try:
            out = [i[4][0] for i in socket.getaddrinfo(socket.gethostname(), None,
                                                       socket.AF_INET)]
        except Exception:
            pass
    return out


def default_host():
    """붙어야 할 VTD 주소. 환경변수 > 자기 IP 대역 > 대회값."""
    env = os.environ.get("VTD_HOST")
    if env:
        return env
    mine = local_ipv4s()
    for prefix, host in NETS:
        if any(ip.startswith(prefix) for ip in mine):
            return host
    return COMPETITION_HOST


def describe():
    """어느 망으로 판단했는지 사람이 읽을 한 줄."""
    h = default_host()
    if os.environ.get("VTD_HOST"):
        return f"{h} (VTD_HOST 강제)"
    tag = {"192.168.50.11": "대회망", "192.168.100.1": "연습 직결랜"}.get(h, "기본값")
    mine = [ip for ip in local_ipv4s() if ip.startswith(("192.168.50.", "192.168.100."))]
    return f"{h} ({tag}" + (f", 내 IP {mine[0]}" if mine else ", 내 IP 미확인") + ")"


if __name__ == "__main__":
    import sys
    print(default_host() if "--host-only" in sys.argv else describe())
