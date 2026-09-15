# 그림 자료 — 무엇이 있고 어떻게 만들었나

[← 문서 목차](../README.md) · [README](../../README.md)

[`README.md`](../../README.md) 와 각 장이 가리키는 그림을 모아 둔다. 전부 스크립트나 녹화에서 다시 만들 수 있다.

## 정지 그림

| 파일 | 내용 | 만든 방법 |
|---|---|---|
| `courses_light.png` · `courses_dark.png` | 연습 코스 6개의 계획 경로를 지도 위에 | xodr 도로 기준선 + `routes/HL_FMA_NEW_*.json`, matplotlib. 라이트·다크 두 벌(`docs/06_results.md` 가 `<picture>` 로 테마에 맞춰 고른다) |
| `competition_header.png` | 대회 공식 평가 화면 상단 — doroMaster 판 #145 | 대회 중계 다시보기 5:59:12 프레임에서 잘라냄 |
| `competition_items.png` | 같은 판의 15개 항목 × 5구간 감점표 | 같은 프레임 |
| `competition_full.jpg` | 같은 프레임 전체 | 같은 프레임 |

대회 화면 세 장의 출처는 주최측 공개 중계(YouTube *HL FMA 2026 자율주행경진대회 iVH시뮬레이션 경진대회*, 2026-09-12)이며, 우리 팀 판만 잘라 썼다.

## README 맨 위 GIF 5개 — 좌우 분할

**왼쪽**은 주행 CSV 로 그린 "차가 아는 세상", **오른쪽**은 같은 순간의 VTD 중계 창(`mainRS_Broadcast` — 차 뒤 위에서 차 전체가 보이는 시점) 녹화다. 왼쪽에 들어가는 것은
전부 제어기가 그 프레임에 실제로 가진 정보다 — 지도에서 읽은 차로 경계·중앙선·정지선, 계획 경로와 지금 따라가는
횡오프셋 경로, GT 물체(차 주황 · 사람 청록 · 사물 회색), 자차, 속도, **속도를 잡은 규칙(`cap_by`)**, 지시등, 신호.
시야(앞 20 m) 밖이지만 내 차로 좌우 5 m 안에 있는 앞쪽 물체는 위 가장자리에 ▲ 와 거리로, 1 m 보다 작은 사물은 마름모로 표시한다.

2026-09-14 VTD PC 에서 대회 코드(32fb790)로 한 판씩 달려 찍었다. 녹화 10 fps, GIF 6 fps · 833×300.

| 파일 | 장면 | 판 · CSV 구간 | offset |
|---|---|---|---|
| `red_light_stop.gif` | 우회전할 교차로에서 초록→노랑→빨강, **정지선에 섰다가** 우회전(`RIGHT_TURN_STOP` → `RED_STOP` → `RIGHT_ON_RED`) | `HL_FMA_NEW_H` · t=36.0~49.0 s | 7.53 s |
| `crosswalk_ped_yield.gif` | 횡단보도를 건너는 사람 앞에서 4 km/h 까지 늦추고, 사람이 내 차로를 벗어나면 간다(`YIELD_PED` → `PED_LEAVING`) | `HL_FMA_NEW_A_TRP` · t=8.5~19.0 s | 7.10 s |
| `obstacle_lane_change.gif` | 차로 위 타이어 더미(0.8 m)를 만나 **차로 하나를 온전히 옮겼다가** 돌아온다 | `LC4_OBS_TIRE` · t=6.3~16.3 s | 6.42 s |
| `lane4_to_1_left.gif` | 좌회전을 앞두고 맨 오른쪽에서 **세 차로를 건너** 맨 왼쪽 차로로 간다 | `LC4_OBS_TIRE` · t=46.9~54.5 s | 6.42 s |
| `overtake_stopped_car.gif` | 내 차로에 선 차 뒤에 섰다가 옆 차로로 **추월하고 복귀**(`OVT:PASS` → `RETURN`) | `HL_FMA_NEW_A_SLA` · t=96.3~109.0 s | 7.10 s |

- `LC4_OBS_TIRE` 는 `LC4_OBS_LEFT` 의 연료통 세 자리(s=90·200·310 m)에 **WheelStack01** 을 놓은 판이다
  (`vtd/add_objects_at.py … --definition WheelStack01 --z-lift 0`). 연료통(0.15×0.46 m)은 화면에서 거의 안 보였다.
  타이어 더미 0.82×0.83×0.94 m 는 폭이 휠체어·자전거 상한(0.70 m)을 넘어 제어기에서 연료통과 같은 **사물**로 분류되고,
  회피도 같은 시각(t=7.0~15.9 s)에 같은 폭(3.8 m)으로 났다. 이 모델은 원점이 바닥이라 기본 높이 보정(0.74 m)을 주면 떠 보인다.
- `HL_FMA_NEW_A_SLA`(`vtd/add_slalom.py`)·`LC4_OBS_TIRE` 는 PR #57, `HL_FMA_NEW_A_TRP`(`vtd/add_crosswalk_peds.py`)는 main 에 있다.
- 추월 판은 이번 판에서 정지차 9 m 뒤에 25초 섰다가(`NARROW_BLOCK`) t=98.1 s 에 추월했다 — 같은 판을 이전에 돌렸을 때는
  서지 않고 t=74 s 에 바로 추월했다. GIF 는 추월 구간만 잘랐다.

⚠️ 주변 교통(`_TR`) 판의 보행자는 **VTD 화면에 그려지지 않았다**(2026-08-28 코스 G 녹화 — CSV 에는 차로 앞 9 m·14 m 에
사람이 있는데 영상엔 아무도 없다). 보행자 장면은 `Character` 로 사람을 직접 심은 `_TRP` 판으로 찍었고, 이 사람은 화면에 보였다.

### 왜 전방 카메라(RTSP)가 아니라 중계 창을 찍나

`vtd/record_run.sh` 가 받는 RTSP 전방 카메라(8554 `/front`)는 운전석 시점이라 차가 안 보이고, 보닛·계기판이 **앞 5~20 m 를 가린다**.
VTD PC 의 `mainRS_Broadcast` 창은 대회 중계와 같은 차 뒤 위 시점이라 **차 전체와 앞 도로**가 한 화면에 들어온다.

이 창은 보통 다른 창 밑에 깔려 있다. `x11grab -window_id` 로 **창 ID 를 지정해 찍으면 가려져 있어도 그 창 내용만** 찍힌다 —
창을 옮기거나 올릴 필요가 없고, 화면 전체를 찍을 때처럼 옆 터미널이 담길 일도 없다(`vtd/record_window.sh`).

**카메라는 8° 위로 든다**(`vtd/set_cams.py --view gif` = follower 자리에서 dp −8°). 공식 follower 그대로면 중계 창에 차 앞
20 m 까지만 보여 사물·보행자가 화면 위 끝에 잠깐만 걸린다. 들면 차 전체와 앞 도로 100 m 가까이가 들어온다.
시나리오를 다시 올리면(`load_scenario.py`) follower 로 돌아가니 **올린 뒤에** 바꾸고, 녹화가 끝나면 `--view follower` 로 되돌린다.
⚠️ 주변 교통 판에서 뒤차가 6 m 안에 붙으면 카메라가 그 차 안에 들어간다.

### 찍는 순서

```bash
# ① [VTD PC] 시나리오 올리고, 카메라를 든다
cd ~/hlfma2026/vtd && python3 load_scenario.py LC4_OBS_TIRE && python3 set_cams.py --view gif

# ② [VTD PC] 녹화를 먼저 켠다 — 중계 창을 창 ID 로, 벽시계 시각 기록. 주행 시간 + 20초쯤
bash record_window.sh ~/runs_gif/TIRE.mp4 115            # 로그는 ~/runs_gif/TIRE.mp4.log

# ③ [제어PC] 5초쯤 뒤 주행 — 띄운 벽시계 시각을 같이 남긴다
#    시험 판(`_SLA` · `_TRP` · `LC4_OBS_TIRE` …)은 원본 경로 이름으로 달린다(경로가 같다)
cd src && date +%s.%N > /tmp/TIRE.main_start && python3 -u main.py --host 192.168.50.11 --port 9910 \
  --scenario ../routes/LC4_OBS_LEFT.json --lane-plan ../routes/LC4_OBS_LEFT_lane.json \
  --tl-map ../routes/tl_map_livinglab.json --crosswalks ../routes/crosswalks.json \
  --stoplines ../routes/stoplines.json --stoplines-all ../routes/stoplines_all.json \
  --log-csv /tmp/run_TIRE.csv

# ④ [아무 PC] 장면 시각을 CSV 에서 찾는다 — 사물 회피는 nudge≠0, 추월은 reason 의 OVT:, 보행자는 YIELD_PED, 차로 옮기기는 sig, 정지는 v≈0
python3 -c "import csv; [print(r['t'], r['reason'], r['cap_by'], r['sig'], r['nudge']) for r in csv.DictReader(open('run_TIRE.csv')) if float(r['nudge'])]" | head

# ⑤ GIF — 왼쪽 CSV 그림 | 오른쪽 녹화. --vcrop 은 속도계를 빼고 차가 가운데 오게 16:9 로 자른다(1280x698 녹화 기준)
python3 vtd/make_gif.py --video TIRE.mp4 --csv run_TIRE.csv --course LC4_OBS_LEFT \
  --t0 6.3 --dur 10 --fps 6 --find-offset --vcrop 1040:585:120:113 --out docs/img/obstacle_lane_change.gif

# ⑥ [VTD PC] 카메라 되돌리기
python3 set_cams.py --view follower
```

### 영상과 CSV 맞추기

`offset` = 영상 시각 − CSV 시각. 녹화를 먼저 켜므로 6~8초쯤 된다.

- **벽시계로 대략**: 녹화 로그의 첫 `start:` 가 첫 프레임의 벽시계 시각이다. `main_start − start` 에 접속까지 걸린 시간이
  더해진 값이 offset 이다. 접속 시간이 판마다 달라(실측 0.1~1.5 s) 이것만으로는 모자라지만, **offset 은 이 값보다 작을 수 없다**.
- **`--find-offset`**: 녹화에서 **가장 오래 서 있다가 출발한 순간**을 CSV 의 출발과 맞춘다. 주변 교통이 없는 판(H · TIRE · SLA)은 이것으로 맞았다.
  ⚠️ 주변 교통 판은 출발 전에도 옆 차가 화면을 움직여 **엉뚱한 순간을 잡는다**(`_TRP`: 5.75 s — 벽시계 하한 6.21 s 보다 작아 틀린 값).
- **장면 안의 정지·출발로 확인**: 차 바로 옆·뒤 노면(화면 아래쪽 좌우)의 흐름이 0 이 되는 구간·다시 생기는 순간을 CSV 의 v 와 맞춘다.
  위 표는 전부 이것으로 확인했다 — 코스 H 정지 구간 7.5~7.6 s, 추월 판 재출발 7.11 s, `_TRP` 판 출발 7.10 s(이 판은 이 값을 썼다).
- **`--refine stop|go`**: 장면 안의 정지·출발 순간으로 다시 맞춘다. 스트림이 끊기며 뒤로 밀리는 RTSP 녹화에 필요했다
  (실측: 같은 판에서 출발 기준 109.3 s, 15초 뒤 정지 기준 111.9 s). 창 녹화(벽시계 타임스탬프)의 GIF 에는 쓰지 않았다.
  다시 맞춘 값이 `--find-offset` 에서 **1초 넘게 튀면 다른 정지를 잡은 것**이니 버린다(한 번 +4.4 s 로 튀었다).
- 확인은 만든 GIF 를 여러 장 뽑아 좌우의 정지선·차로선·물체 위치를 대조한다.

```bash
ffmpeg -v error -i docs/img/obstacle_lane_change.gif -vf "select='not(mod(n\,8))',tile=2x4" -frames:v 1 /tmp/sheet.png
```
