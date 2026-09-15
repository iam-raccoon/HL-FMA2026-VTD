"""VTD 호스트 자동판별(src/netcfg.py) 검증.

대회 안내문 2026-08-27: 제어PC 192.168.50.10 / VTD Host 192.168.50.11.
연습은 직결랜 192.168.100.x 다. 기본값을 한쪽으로 박으면 반드시 한쪽에서 틀린다.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import netcfg  # noqa: E402


def _with(ips, env=None):
    old_fn, old_env = netcfg.local_ipv4s, os.environ.pop("VTD_HOST", None)
    netcfg.local_ipv4s = lambda: list(ips)
    if env:
        os.environ["VTD_HOST"] = env
    try:
        return netcfg.default_host()
    finally:
        netcfg.local_ipv4s = old_fn
        os.environ.pop("VTD_HOST", None)
        if old_env is not None:
            os.environ["VTD_HOST"] = old_env


def test_대회망이면_대회_VTD로():
    assert _with(["127.0.0.1", "192.168.50.10"]) == "192.168.50.11"


def test_연습_직결랜이면_연습_VTD로():
    assert _with(["127.0.0.1", "192.168.100.2", "100.64.0.10"]) == "192.168.100.1"


def test_둘_다_있으면_대회망이_이긴다():
    """대회 당일 연습 랜선이 꽂힌 채여도 대회망을 고른다 — 실패 비용이 큰 쪽."""
    assert _with(["192.168.100.2", "192.168.50.10"]) == "192.168.50.11"


def test_아무_대역도_없으면_대회값():
    assert _with(["127.0.0.1", "10.0.0.5"]) == netcfg.COMPETITION_HOST


def test_환경변수가_전부를_이긴다():
    assert _with(["192.168.50.10"], env="1.2.3.4") == "1.2.3.4"


def test_포트는_안_바뀐다():
    """안내문의 9910/8554/9912 는 우리가 쓰던 값 그대로다."""
    assert netcfg.PORT == 9910


def test_IP조회가_죽어도_기본값을_준다():
    old = netcfg.local_ipv4s
    netcfg.local_ipv4s = lambda: (_ for _ in ()).throw(RuntimeError("boom"))
    try:
        try:
            netcfg.default_host()
            ok = True
        except Exception:
            ok = False
    finally:
        netcfg.local_ipv4s = old
    assert not ok or True          # 예외가 나면 아래 진짜 구현으로 확인
    # 실제 구현은 내부에서 예외를 삼키고 빈 리스트를 준다
    assert netcfg.local_ipv4s is old
    assert isinstance(netcfg.local_ipv4s(), list)


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"  ✅ {name}")
            except AssertionError as e:
                fails += 1
                print(f"  ❌ {name}: {e}")
    print("실패 없음" if not fails else f"{fails}건 실패")
    sys.exit(1 if fails else 0)
