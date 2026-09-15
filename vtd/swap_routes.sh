#!/bin/bash
# 새로 만든 경로를 routes/ 로 들여놓는다. **회귀가 도는 중에는 절대 실행하지 말 것** —
# run_scenario.sh 가 판마다 routes/ 를 읽어 제어PC로 보내므로, 도중에 바꾸면
# 앞판과 뒷판이 서로 다른 경로로 돌아 비교가 무의미해진다.
#
# 사용: bash vtd/swap_routes.sh <새경로가 있는 디렉터리>
#       (그 안에 fin_v1.json fin_v1_lane.json fin_v7.* fin_wp9.* 가 있어야 한다)
set -eu
SRC="${1:?새 경로 디렉터리}"
R=~/hlfma2026/routes
for f in fin_v1 fin_v1_lane fin_v7 fin_v7_lane fin_wp9 fin_wp9_lane; do
  [ -s "$SRC/$f.json" ] || { echo "❌ $SRC/$f.json 이 없다"; exit 1; }
done
cp "$SRC/fin_v1.json"        "$R/planned_from_xml_v1.json"
cp "$SRC/fin_v1_lane.json"   "$R/planned_from_xml_v1_lane.json"
cp "$SRC/fin_v7.json"        "$R/planned_v7.json"
cp "$SRC/fin_v7_lane.json"   "$R/planned_v7_lane.json"
cp "$SRC/fin_wp9.json"       "$R/planned_wp9.json"
cp "$SRC/fin_wp9_lane.json"  "$R/planned_wp9_lane.json"
ls -la "$R"/planned_*.json
echo "교체 완료 — 이제 bash vtd/run_regression.sh"
