#!/usr/bin/env bash
# HL-FMA 2026 제어PC(노트북, Ubuntu 24.04) 개발환경 셋업
# 사용: bash setup_laptop.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"

echo "[1/4] 기본 패키지 (빌드/영상/네트워크 도구)"
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip build-essential ffmpeg tcpdump git curl

echo "[2/4] GCC-8 / G++-8  (VTD RDB C++ API 빌드용)"
# ⚠️ Ubuntu 24.04(noble)엔 gcc-8이 기본 저장소/toolchain PPA에 없을 수 있음.
if sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test 2>/dev/null && \
   sudo apt-get update && sudo apt-get install -y gcc-8 g++-8 2>/dev/null; then
  echo "  -> gcc-8 설치 성공"
else
  echo "  ⚠️ apt로 gcc-8 설치 실패(noble에 패키지 없음)."
  echo "     대안: (a) org 제공 빌드 컨테이너/도커 사용  (b) 22.04(jammy) .deb 수동설치"
  echo "     (c) 소스 빌드. 8/13 오후 '제어연동' 교육에서 org 안내 확인 후 결정."
fi

echo "[3/4] Python 가상환경 + 의존성 (시스템 numpy와 격리)"
python3 -m venv "$HERE/.venv"
"$HERE/.venv/bin/pip" install -U pip
"$HERE/.venv/bin/pip" install -r "$HERE/requirements.txt"

echo "[4/4] (선택) NVIDIA 드라이버 — GTX 1050Ti (카메라 YOLO 등 CUDA 쓸 때만)"
echo "     현재 드라이버 미설치. 필요시:  sudo ubuntu-drivers autoinstall  후 재부팅"
echo "     (제어 스택은 CPU 파이썬이라 드라이버 없어도 동작)"

echo
echo "완료. 오프라인 검증:"
echo "  cd $HERE"
echo "  python3 mock_vtd.py &          # 터미널1 (또는 백그라운드)"
echo "  .venv/bin/python3 main.py --host 127.0.0.1"
echo
echo "실 VTD 연결(직결선):"
echo "  .venv/bin/python3 main.py --route route.csv   # VTD 주소는 자동판별"
