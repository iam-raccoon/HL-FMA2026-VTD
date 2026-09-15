#!/bin/bash
# ★제어PC에서 한 줄로 주행. **시나리오 이름** 또는 **주최측 경로 CSV** 를 준다.
#
#     bash ~/hlfma2026/drive.sh v2                      # 연습 — 이름만
#     bash ~/hlfma2026/drive.sh HL_FMA_NEW_A_TR
#     bash ~/hlfma2026/drive.sh HL_FMA_PRETEST_1         # 만든 사전주행 시나리오
#     bash ~/hlfma2026/drive.sh /media/usb/route.csv    # ★대회 — USB 로 받은 경로
#     bash ~/hlfma2026/drive.sh /media/usb/route.csv --dry   # 준비만 하고 안 달림
#     bash ~/hlfma2026/drive.sh net                     # 망 점검만
#     bash ~/hlfma2026/drive.sh net fix                 # 유선 IP 를 대회망으로 맞춘다
#
# ★★CSV 를 주면 **계획부터 주행까지** 한 줄로 한다(예전 `race.sh`).
#   따로 두었더니 주행 직전 출발점 확인·카메라 세팅 같은 걸 계속 빠뜨렸다 —
#   실측 2026-09-04: ego 가 (0,0) 인 채로 주행이 시작돼 제어기가 첫 프레임에
#   `✅ 목표 도달` 을 찍고 끝났고, 카메라가 차를 안 따라가 깜빡이를 볼 수 없었다.
#   주행 경로는 **하나뿐**이어야 한다.
#
# 왜 필요한가(실측 2026-08-19): 시나리오와 경로 파일의 짝이 이름으로 드러나지 않는다.
#   v1~v6 -> planned_from_xml_v1.json     (이름에 v2~v6 이 없다)
#   v7    -> planned_v7.json              (from_xml 이 없다)
#   WP9   -> planned_wp9.json
# 그래서 `planned_from_xml_v7.json` 을 찾다 없다고 나오고, OFFEX 를 올려놓고 v1 경로로
# 달려서 **600m 떨어진 코스를 따라가려다** 리스폰이 났다. 짝을 사람이 외울 일이 아니다.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
R="$HERE/routes"
NAME="${1:-}"
# ★VTD 주소는 **자기 IP 대역으로 자동 판별**한다(src/netcfg.py).
#   대회·연습 모두 192.168.50.11 로 통일했다(2026-09-04). 옛 직결랜 192.168.100.1 도 받는다.
#   HOST 나 VTD_HOST 를 주면 그게 이긴다.
HOST="${HOST:-$(python3 "$HERE/src/netcfg.py" --host-only)}"
# ⚠️ 로그 CSV 이름은 **모드가 정해진 뒤에** 짓는다. 예전엔 여기서 `$NAME` 으로
#    먼저 잡았는데, 대회 모드에서는 `$NAME` 이 경로 CSV **파일경로**라
#    `/tmp/run_routes/pretest/route_pretest_1.csv.csv` 같은 없는 디렉터리가 나왔다
#    (실측 2026-09-04: FileNotFoundError 로 주행이 시작조차 못 했다).
CSV_USER="${CSV:-}"

DRY=0; [ "${2:-}" = "--dry" ] && DRY=1
die() { echo; echo "❌ $*"; echo "   -> 여기서 멈춘다. 이 상태로 주행하면 안 된다."; exit 1; }

# ── ★망 ──────────────────────────────────────────────────────────────────────
# 연습망도 **대회망과 같은 주소**로 통일했다(2026-09-04).
#   제어PC 192.168.50.10  ↔  VTD 192.168.50.11
# 왜: 대회는 랜선을 꽂는 순간 15분이 돌고, 5번(제어기 작동) 뒤에는 제어PC 를 만지면
#     실격이다. 그 안에서 IP 를 바꾸는 건 위험하다. 평소에 대회 주소로 살면 바꿀 일이 없다.
# 되돌리려면 `bash drive.sh net fix 192.168.100.2` 처럼 주소를 직접 준다.
NET_ME="192.168.50.10"          # 제어PC 유선 IP
NET_PFX="192.168.50."           # 이 대역이면 대회망
NET_CON="vtd-direct"            # 제어PC 의 유선 직결 NetworkManager 커넥션 이름

net_myips() { ip -4 -o addr show scope global 2>/dev/null | awk '{print $2, $4}'; }

net_show() {
  echo "  ── 망 점검 ──"
  net_myips | sed 's/^/       /'
  local ok=0
  net_myips | awk '{print $2}' | grep -q "^$NET_PFX" && ok=1
  if [ "$ok" = "1" ]; then echo "       ✅ 대회망 대역($NET_PFX) 에 있다"
  else                     echo "       ❌ 대회망 대역($NET_PFX) 이 없다"; fi
  echo -n "       VTD $HOST : "
  if ping -c1 -W1 "$HOST" >/dev/null 2>&1; then echo "✅ ping 응답"
  else echo "⚠️ ping 무응답 (ICMP 를 막아뒀을 수도 있다)"; fi
  python3 "$HERE/src/netcfg.py" | sed 's/^/       netcfg 판정: /'
  return $((1 - ok))
}

net_fix() {
  local want="${1:-$NET_ME}"
  echo "  유선 커넥션 '$NET_CON' 를 $want/24 로 맞춘다 (sudo 필요)"
  sudo nmcli con mod "$NET_CON" ipv4.method manual ipv4.addresses "$want/24" \
    || die "nmcli con mod 실패 — 커넥션 이름이 '$NET_CON' 가 맞는지 확인 (nmcli con show)"
  sudo nmcli con up "$NET_CON" || die "nmcli con up 실패 — 랜선이 꽂혀 있는지 확인"
  echo
  HOST="$(python3 "$HERE/src/netcfg.py" --host-only)"
  net_show
}

if [ "$NAME" = "net" ]; then
  case "${2:-}" in
    fix) net_fix "${3:-$NET_ME}" ;;
    "")  net_show || { echo; echo "   맞추려면:  bash drive.sh net fix"; exit 1; } ;;
    *)   echo "쓰는 법: bash drive.sh net [fix [주소]]"; exit 1 ;;
  esac
  exit 0
fi

# ★주행 전 망 점검. 자기 IP 가 틀리면 **여기서 멈춘다** — 20초짜리 ego_pose 타임아웃을
#   두 번 기다린 뒤에야 알아채는 건 대회 15분에 쓸 수 없다.
#   ping 무응답은 막지 않는다(대회장에서 ICMP 를 막아뒀을 수 있다).
if ! net_myips | awk '{print $2}' | grep -q "^$NET_PFX"; then
  net_show
  die "제어PC 유선 IP 가 대회망($NET_ME) 이 아니다 -> 고치려면:  bash drive.sh net fix"
fi
ping -c1 -W1 "$HOST" >/dev/null 2>&1 \
  || echo "  ⚠️ VTD $HOST 가 ping 에 무응답 — 랜선·시나리오를 확인할 것(계속 진행한다)"

# ── ★대회 모드: 인자가 경로 CSV 면 계획부터 한다 ────────────────────────────
if [ -f "$NAME" ] && case "$NAME" in *.csv) true;; *) false;; esac; then
  set -o pipefail       # ⚠️ 없으면 `python3 | sed` 의 실패를 못 잡는다(2026-09-04 실측)
  SRC="$NAME"
  # 맵은 레포 안(map/)을 먼저 본다 — 제어PC 에는 ~/hlfma2026_map 이 없었다(실측)
  X="${X:-}"
  if [ -z "$X" ]; then
    for c in "$HERE/map/HL_FMA_VTD_LivingLab.xodr" \
             "$HOME/hlfma2026_map/HL_FMA_VTD_LivingLab.xodr"; do
      [ -s "$c" ] && { X="$c"; break; }
    done
  fi
  [ -s "${X:-}" ] || die "지도(xodr)가 없다 — 레포에 map/ 이 있어야 한다 (X=... 로 지정 가능)"
  ROUTE="$R/race.json"; LANE="$R/race_lane.json"
  CSV="${CSV_USER:-/tmp/run_$(basename "${SRC%.csv}").csv}"

  echo "════════ 대회 준비  $(date +%H:%M:%S) ════════"
  echo "  경로 CSV : $SRC   ($(($(wc -l < "$SRC") - 1))개 지점)"

  # ① ego 헤딩 — **이 경로의 것인지 확인하고** 쓴다.
  #    시나리오가 안 올라갔으면 VTD 는 ego 를 (0,0)/맵 밖에 두고 헤딩 0° 를 준다.
  #    그걸 믿으면 엉뚱한 차로로 출발한다(2026-08-19 코스 A 커버리지 56%).
  HD=""; SAID=0
  POSE=$(timeout 20 python3 "$HERE/vtd/ego_pose.py" --host "$HOST" --json 2>/dev/null | tail -1)
  if [ -n "$POSE" ]; then
    HD=$(python3 -c "
import json,sys
try: print(f\"{json.loads(sys.argv[1])['heading_deg']:.2f}\")
except Exception: pass" "$POSE" 2>/dev/null)
  fi
  if [ -n "$HD" ]; then
    FAR=$(python3 - "$SRC" "$POSE" <<'PY' 2>/dev/null
import csv, json, math, sys
r = next(csv.DictReader(open(sys.argv[1])))
p = json.loads(sys.argv[2])
print(f"{math.hypot(p['x'] - float(r['x']), p['y'] - float(r['y'])):.1f}")
PY
)
    if [ -n "$FAR" ] && [ "$(python3 -c "print(1 if float('$FAR') > 30 else 0)")" = "1" ]; then
      echo "  ① ego 헤딩  : ⚠️ **버린다** — 차가 경로 출발점에서 ${FAR}m 떨어져 있다"
      echo "               (시나리오가 안 올라갔거나 다른 시나리오다)"
      HD=""; SAID=1
    fi
  fi
  if [ -n "$HD" ]; then
    echo "  ① ego 헤딩  : ${HD}°  (시뮬에서 읽음, 출발점까지 ${FAR}m)"
    HDARG=(--heading "$HD")
  else
    [ "$SAID" = "1" ] || echo "  ① ego 헤딩  : ⚠️ 못 읽었다 -> 좌표로 추정"
    echo "               좌표로 추정해 계획한다. 시뮬이 뜬 뒤 **한 번 더** 돌릴 것"
    HDARG=()
  fi

  echo "  ② 경로 계획"
  timeout 300 python3 "$HERE/vtd/plan_route.py" "$X" "${HDARG[@]}" \
    --from-csv "$SRC" "$ROUTE" 2>&1 | sed 's/^/       /' || die "경로 계획 실패"

  echo "  ③ 경로 검사"
  CHK=$(timeout 300 python3 "$HERE/eval/check_route.py" "$ROUTE" "$X" 2>&1)
  echo "$CHK" | grep -E "^\[|이상|⚠" | sed 's/^/       /'
  echo "$CHK" | grep -q "이상 0곳" || die "경로 검사에서 이상이 나왔다 (위 목록)"
  OFF=$(echo "$CHK" | grep -oP '\[③ 도로 밖\]\s*\K[0-9]+' | head -1)
  [ "${OFF:-0}" = "0" ] || die "경로가 도로 밖으로 ${OFF}점 나간다 — CSV 좌표 확인"

  echo "  ④ 차로 계획"
  timeout 300 python3 "$HERE/vtd/build_lane_plan.py" "$X" "$ROUTE" "$LANE" 2>&1 \
    | sed 's/^/       /' || die "차로 계획 실패"

  if [ $DRY -eq 1 ]; then
    echo; echo "   ✅ 준비 끝 (--dry 라 주행 안 함)"
    echo "      달리려면:  bash drive.sh $SRC"
    exit 0
  fi
  echo
else
# ── 연습 모드: 시나리오 이름 ─────────────────────────────────────────────────

# 공식 시나리오는 긴 VTD 이름 대신 v1~v7 만 줘도 된다. 경로 데이터는 v1~v6 이
# 공용이지만, NAME 은 실제 VTD 시나리오 이름으로 정규화해 아래 매핑과 로그를 일치시킨다.
case "$NAME" in
  v1|v2|v3|v4|v5|v6|v7) NAME="HL_FMA_VTD_LivingLab_$NAME" ;;
esac

case "$NAME" in
  HL_FMA_VTD_LivingLab_v1|HL_FMA_VTD_LivingLab_v2|HL_FMA_VTD_LivingLab_v3|\
  HL_FMA_VTD_LivingLab_v4|HL_FMA_VTD_LivingLab_v5|HL_FMA_VTD_LivingLab_v6|EV_*)
      ROUTE="$R/planned_from_xml_v1.json" ;;   # 같은 코스에 이벤트만 다르다
  HL_FMA_VTD_LivingLab_v7) ROUTE="$R/planned_v7.json" ;;
  HL_FMA_WP9)              ROUTE="$R/planned_wp9.json" ;;
  HL_FMA_OFFEX)            ROUTE="$R/HL_FMA_OFFEX.json" ;;
  HL_FMA_PRETEST_1)        ROUTE="$R/route_pretest.json" ;;
  HL_FMA_PRETEST_2)        ROUTE="$R/route_pretest_2.json" ;;
  # 2026-09-12 A조 중계 영상 재현판. `_TR`은 주변 교통 50대 난수판,
  # `_EXACT`는 영상 액터 고정판, 접미사가 없는 이름은 빈 도로 확인용이다.
  HL_FMA_2026_VIDEO_A|HL_FMA_2026_VIDEO_A_TR|HL_FMA_2026_VIDEO_A_EXACT)
                           ROUTE="$R/HL_FMA_2026_VIDEO_A.json" ;;
  route_pretest|route_pretest_2) ROUTE="$R/$NAME.json" ;;
  # 연습용 새 코스 — 이름과 경로 파일 이름이 같다(2026-08-20 부터 규칙을 맞췄다)
  # ★`_EV`(이벤트) · `_HZ`(돌발상황) 은 **같은 코스에 상대차만 더한 것**이라 경로가 같다.
  #   접미사를 떼야 한다 — 안 그러면 routes/HL_FMA_NEW_A_HZ.json 을 찾다 없다고 죽는다.
  # ⚠️ 접미사는 **긴 것부터** 뗀다. `_TRP`/`_TRV` 를 `_TR` 보다 먼저 떼지 않으면 `_P`/`_V` 가
  #    남아 경로를 못 찾는다(2026-09-09: `_TRP` 를 만들어 놓고 이걸 안 고쳐 못 돌릴 뻔했다).
  HL_FMA_NEW_*)            B="${NAME%_TRP}"; B="${B%_TRV}"; B="${B%_VRU}"
                           B="${B%_EV}"; B="${B%_HZ}"; B="${B%_TR}"; ROUTE="$R/$B.json" ;;
  *)  echo "쓰는 법:"
      echo "  bash drive.sh <시나리오이름>        연습 — VTD 에 올린 코스"
      echo "  bash drive.sh <경로.csv>            ★대회 — 계획부터 주행까지"
      echo "  bash drive.sh <경로.csv> --dry      준비만 하고 안 달림"
      echo "  bash drive.sh net [fix]             망 점검 / 대회망으로 맞추기"
      echo "  아는 이름: HL_FMA_VTD_LivingLab_v1~v7 · HL_FMA_WP9 · HL_FMA_OFFEX"
      echo "             HL_FMA_2026_VIDEO_A (_TR: 난수 교통, _EXACT: 영상 액터 고정 배치)"
      echo "             EV_* · HL_FMA_NEW_A B D E G H (뒤에 _EV/_HZ/_TR/_TRP/_TRV/_VRU 가능)"
      echo "             HL_FMA_PRETEST_1 · HL_FMA_PRETEST_2"
      echo "             route_pretest · route_pretest_2 (경로명 직접 지정)"
      echo "  routes/ 에 있는 경로 파일:"; ls "$R"/*.json | xargs -n1 basename | sed 's/^/    /'
      exit 1 ;;
esac
LANE="${ROUTE%.json}_lane.json"
  CSV="${CSV_USER:-/tmp/run_$NAME.csv}"
fi
mkdir -p "$(dirname "$CSV")" 2>/dev/null || true
[ -s "$ROUTE" ] || { echo "❌ 경로 파일이 없다: $ROUTE"; exit 1; }
[ -s "$LANE" ]  || { echo "❌ 차로계획이 없다: $LANE"; exit 1; }

# ★주행 전에 **차가 경로 시작점에 있는지** 본다. 이걸 안 보면 엉뚱한 코스를 달린다.
POSE=$(python3 "$HERE/vtd/ego_pose.py" --host "$HOST" --json 2>/dev/null | tail -1)
python3 - "$POSE" "$ROUTE" <<'PY' || exit 1
import json, math, sys
try:
    p = json.loads(sys.argv[1])
except Exception:
    print("❌ ego 위치를 못 읽었다 — VTD 에 시나리오가 올라갔는지 확인할 것"); raise SystemExit(1)
r = json.load(open(sys.argv[2], encoding="utf-8"))["ego_route"][0]
d = math.hypot(p["x"] - r[0], p["y"] - r[1])
print(f"  ego ({p['x']:.1f},{p['y']:.1f})  경로 시작 ({r[0]:.1f},{r[1]:.1f})  차이 {d:.1f}m")
if d > 30.0:
    print(f"❌ **시나리오와 경로가 다른 코스다**({d:.0f}m 차이). VTD 에 올린 시나리오를 확인할 것")
    raise SystemExit(1)
PY

# ★★**대회에서는 주최측이 준 포트만 쓴다** — 기본값을 '카메라 안 건드림'으로 둔다.
#   주최측 답변(2026-09-11 Redmine, "대회에서 사용되는 프로토콜 질문"):
#     Q. TCP·RTSP·UDP 3개를 쓴다고 아는데 RDB 포트도 제어기에서 사용해도 되나요?
#     A. **제공해드린 포트만 사용 가능합니다.**
#   안내문이 준 것은 9910 TCP · 8554 RTSP · 9912 UDP 셋뿐인데, 카메라 시점 SCP 는
#   taskControl **48179** 라 그 밖이다. 여기서 매 주행마다 그 포트를 열고 있었다.
#   화면 시점은 주최측 PC 몫이고 점수와 무관하다 — 안 보내는 쪽이 맞다.
#
# ★연습에서는 `CAM=1` 로 켠다. 시나리오를 새로 올리거나 Apply 를 누르면 기본 시점으로
#   돌아가 차가 화면에서 사라지기 때문에, 우리 PC 로 볼 때는 이게 편하다.
#   (2026-09-06: 각도 속성 이름이 `h/p/r` 이 아니라 **`dh/dp/dr`** 였다 — 무시되고 있었다.)
if [ "${CAM:-0}" = "1" ]; then
  python3 "$HERE/vtd/set_cams.py" --view follower --host "$HOST" >/dev/null 2>&1 || true
else
  echo "  카메라 시점은 안 건드린다(대회 허용 포트 밖 48179). 연습에서 켜려면 CAM=1"
fi

echo "  주행: $(basename "$ROUTE")  -> $CSV"
cd "$HERE/src" && exec python3 -u main.py --host "$HOST" --port 9910 \
  --scenario "$ROUTE" --lane-plan "$LANE" --tl-map "$R/tl_map_livinglab.json" \
  --crosswalks "$R/crosswalks.json" --stoplines "$R/stoplines.json" \
  --stoplines-all "$R/stoplines_all.json" \
  --log-csv "$CSV"
