#!/usr/bin/env bash
# 시나리오 하나를 오프라인(VTD 없이)으로 돌리고 자동 채점.
# 사용: bash eval/run_eval.sh eval/scenarios/ped_crossing.json
#       PY=.venv/bin/python3 bash eval/run_eval.sh eval/scenarios/school_zone.json
D="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$D/.." && pwd)"
SC="${1:-$D/scenarios/ped_crossing.json}"
PORT="${PORT:-9910}"
PY="${PY:-python3}"

# ★차로계획이 있으면 반드시 같이 넘긴다(`<시나리오>_lane.json`).
#   대회 주행(drive.sh)은 언제나 `--lane-plan` 을 넘기는데 여기만 안 넘기고 있었다.
#   차로 배치를 모르면(`plan=None`) 기본 모드는 새 횡기동을 시작하지 않으므로
#   (주최측 답변에 따라 중앙선을 회피 공간으로 쓰지 않는다, `_side_for` 주석),
#   회피가 필요한 판은 앞 장애물 앞에서 그냥 서서 **미완주**가 된다.
#   실측 2026-09-08: static_obstacle·hz_blocker_leaves·hz_wrongway·v3_leadbrake 가
#   전부 85점(목표 미도달)이었고, passped_junction 은 차로계획 파일이 **있는데도**
#   안 읽혀 85점이었다. 넷 다 차로계획을 주면 100점이다.
#   즉 그 점수들은 주행 결함이 아니라 **하네스가 대회와 다르게 돌던 것**이다.
LANE="${SC%.json}_lane.json"
LANE_ARG=()
[ -f "$LANE" ] && LANE_ARG=(--lane-plan "$LANE")

"$PY" -u "$D/mock_vtd.py" --scenario "$SC" --port "$PORT" &
MK=$!
sleep 1.3
"$PY" -u "$ROOT/src/main.py" --host 127.0.0.1 --port "$PORT" --scenario "$SC" \
    "${LANE_ARG[@]}" || true
wait "$MK" 2>/dev/null
