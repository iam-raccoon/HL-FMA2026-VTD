#!/usr/bin/env bash
# VTD 전방 카메라(RTSP)를 파일로 녹화한다. 주행과 **동시에** 돌리면 나중에 장면을 볼 수 있다.
#
#   bash vtd/record_run.sh <출력.mp4> [초] [host]
#     예: bash vtd/record_run.sh /tmp/runA.mp4 900 192.168.50.11
#
# 왜 필요한가(2026-08-22): 지금까지 화면은 사용자만 볼 수 있었다. "커브에서 깜빡이가 켜진다",
# "횡단보도 앞에서 멈춘다" 같은 지적은 전부 눈으로 본 것이고, 숫자로는 사후에야 확인했다.
# VTD 는 8554 로 전방 카메라를 RTSP 로 내보낸다(MediaMTX/gortsplib, 경로 **/front**,
# 1920x1080 H.264 20fps). 녹화해 두면 CSV 의 t 와 맞춰 그 순간 장면을 꺼내 볼 수 있다.
#
# ⚠️ 대역폭. 1080p20 은 수 Mbps 다. 랩 네트워크를 타고 당기면 그만큼 부하가 걸린다.
#    기본을 **540p 10fps** 로 낮춰 잡았다. 원본이 필요하면 RAW=1 로 준다.
# ⚠️ 9910(제어 링크)은 건드리지 않는다. 이건 읽기 전용 RTSP 다.
set -u
OUT="${1:-/tmp/vtd_run.mp4}"
SEC="${2:-900}"
HOST="${3:-${VTD_HOST:-192.168.50.11}}"
URL="rtsp://$HOST:8554/front"
if [ "${RAW:-0}" = "1" ]; then VF=(); else VF=(-vf "fps=10,scale=960:-2"); fi
echo "  녹화 $URL -> $OUT  (${SEC}s, $([ "${RAW:-0}" = 1 ] && echo 원본 || echo 960x540@10))"
# ⚠️ **조각 mp4 로 쓴다.** 일반 mp4 는 moov atom 을 마지막에 쓰기 때문에, 녹화 도중에는
#    파일을 열 수조차 없다(실측 2026-08-24: 주행 중 장면을 보려다 'moov atom not found').
#    ffmpeg 이 죽으면 그때까지 찍은 것도 통째로 못 쓴다. 조각 mp4 는 도중에도 읽힌다.
exec ffmpeg -rtsp_transport tcp -y -i "$URL" -t "$SEC" "${VF[@]}" \
     -c:v libx264 -preset veryfast -crf 28 -an \
     -movflags +frag_keyframe+empty_moov+default_base_moof "$OUT"
