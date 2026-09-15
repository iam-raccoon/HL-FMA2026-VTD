#!/usr/bin/env bash
# VTD 렌더 창 하나를 **창 ID 로** 녹화한다 — README GIF 오른쪽 영상용. **VTD PC 에서** 돌린다.
#
#   bash vtd/record_window.sh <출력.mp4> [초] [창 이름]
#     예: bash vtd/record_window.sh ~/runs_gif/LC4.mp4 115              # 기본 mainRS_Broadcast
#
# 왜 이 창인가(2026-09-14): `mainRS_Broadcast` 는 대회 중계와 같은 **차 뒤 위에서 차 전체가 보이는** 시점이다.
#    RTSP 전방 카메라(record_run.sh)는 운전석 시점이라 보닛·계기판이 앞 5~20 m 를 가리고 차도 안 보인다.
# 창 ID 로 찍으면 다른 창(터미널 등)에 가려져 있어도 **그 창 내용만** 찍힌다 — 창을 옮기거나 올릴 필요가 없고,
#    화면 전체를 찍을 때처럼 옆 터미널이 담길 일도 없다. 1854x1011 을 1280 폭으로 줄여 저장한다.
# 카메라는 이 스크립트가 건드리지 않는다 — GIF 는 `set_cams.py --view gif`(8° 위로) 로 찍었다(docs/img/README.md).
#
# 영상↔CSV 맞추기: <출력>.log 첫 'start:' 가 첫 프레임의 벽시계 시각이다(-use_wallclock_as_timestamps).
#    main.py 를 띄운 벽시계 시각과의 차이 + 접속까지 걸린 시간(실측 0.3~1.5 s)이 offset 이다.
#    make_gif.py --find-offset 은 출발 순간으로 이 값을 다시 찾는다.
set -u
OUT="${1:?출력.mp4}"
SEC="${2:-120}"
NAME="${3:-mainRS_Broadcast}"
export DISPLAY="${DISPLAY:-:1}"
# xdotool search 는 VTD 를 다시 띄우기 전의 죽은 창 ID 까지 돌려준다 — 지금 떠 있는 창 목록에서 고른다
WID=""
for w in $(xprop -root _NET_CLIENT_LIST | grep -o '0x[0-9a-f]*'); do
  [ "$(xdotool getwindowname $((w)) 2>/dev/null)" = "$NAME" ] && WID=$w
done
[ -n "$WID" ] || { echo "창 '$NAME' 이 없다 — VTD 가 떠 있나?"; exit 1; }
echo "  녹화 $NAME ($WID) -> $OUT  (${SEC}s, 10fps, 1280 폭)"
# 조각 mp4 — 도중에도 열리고, ffmpeg 이 죽어도 찍은 데까지 남는다(record_run.sh 와 같은 이유)
exec ffmpeg -y -f x11grab -framerate 10 -use_wallclock_as_timestamps 1 -window_id "$WID" -i "$DISPLAY" \
     -t "$SEC" -vf scale=1280:-2 -c:v libx264 -preset veryfast -crf 24 -pix_fmt yuv420p -an \
     -movflags +frag_keyframe+empty_moov+default_base_moof "$OUT" 2> "$OUT.log"
