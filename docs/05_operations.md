# 5장. 운용 절차 — 실제로 돌리는 법

[← 문서 목차](README.md) · [README](../README.md)

VTD PC 와 제어PC 로 **실제 주행을 돌리는 매뉴얼**이다. 연습 때는 [5.2 한 판 돌리기](#52-한-판-돌리기)를,
대회 당일 새 좌표를 받으면 [5.3 대회 당일](#53-대회-당일--주최측-좌표로-달리기)을 따라 하면 된다.
나머지는 경로 검사·시나리오 목록·검증 러너·녹화·문제 해결이다. 절차마다 **어디서 돌리는지**(`[VTD PC]` `[제어PC]` `[개발PC]`)를 붙였다.
규칙 뒤의 "왜"(디버깅 경위)는 [부록 A](99_dev_log.md) 에 날짜별로 있다.

**목차**

- [5.1 준비 — PC 와 코드](#51-준비--pc-와-코드)
- [5.2 한 판 돌리기](#52-한-판-돌리기)
- [5.3 대회 당일 — 주최측 좌표로 달리기](#53-대회-당일--주최측-좌표로-달리기)
- [5.4 경로 검사 — `eval/check_route.py`](#54-경로-검사--evalcheck_routepy)
- [5.5 시나리오](#55-시나리오)
- [5.6 검증 — 러너 · 오프라인 회귀 · 테스트](#56-검증--러너--오프라인-회귀--테스트)
- [5.7 화면 녹화](#57-화면-녹화)
- [5.8 문제 해결](#58-문제-해결)

## 5.1 준비 — PC 와 코드

### 5.1.1 PC 와 주소

대회 구조는 **제어PC 가 VTD PC 에 접속해 9910 으로 제어**하는 것이다. VTD PC 는 주최측이 제공하고, 제어PC 만 지참한다.

| PC | 주소 | 하는 일 |
|---|---|---|
| VTD PC (연습 때는 OMEN) | **`192.168.50.11`** — 대회·연습 같은 주소(안내문 2026-08-27) | VTD 실행, 시나리오 로드 |
| 제어PC | **`192.168.50.10`** | `src/main.py` 주행 |
| 개발PC | 원격이면 `--host <VTD PC 원격 주소>`(연습 때 tailscale) | 경로 만들기, 채점, 녹화 |

- 2026-09-04 부터 **연습 직결랜도 대회 주소로 통일**했다. 대회날 IP 를 바꿀 일이 없다.
  OMEN 쪽은 USB 랜 커넥션 `hl-direct`·`vtd-direct` **둘 다** 50.11 로 해 두어 어느 동글을 꽂아도 된다. 랩 LAN(192.168.0.x)·와이파이는 건드리지 않았다.
- 주소 점검·변경은 제어PC 에서 **`bash drive.sh net`** / **`bash drive.sh net fix`**.
  옛 직결랜 `192.168.100.x` 로 되돌리려면 `bash drive.sh net fix 192.168.100.2`.
- `main.py` 에 `--host` 를 안 주면 **자기 IP 대역으로 자동 판별**한다(`src/netcfg.py`).
- 개발머신에서 한 번에(로드 → 주행 → 채점): `bash vtd/run_official.sh v1 /tmp/v1.json`

기본 주행 명령(제어PC):

```bash
cd ~/hlfma2026/src
python3 main.py --scenario ../routes/official_v1_v6.json \
  --tl-map ../routes/tl_map_livinglab.json --lane-plan ../routes/lane_plan_v1_v6.json \
  --log-csv /tmp/run.csv
```

### 5.1.2 제어PC 에 코드 올리기 — `git pull`

⚠️ **제어PC 는 자기 레포의 `src/` 를 돌린다.** 개발머신에서 고친 걸 안 올리면 옛 코드로 검증하게 된다
(2026-08-16 실측: `drive.py` 수정 후 그대로 돌려 '수정 전' 결과를 볼 뻔했다).

```bash
# [제어PC]
cd ~/hlfma2026 && git pull
```

- 제어PC 의 `~/hlfma2026` 은 2026-08-20 부터 git 클론이다. **무엇이 올라가 있는지 `git log` 로 확인**한다.
  그 전의 복사본 운용에서 생긴 문제는 [부록 A](99_dev_log.md).
- 비공개 레포라 토큰이 필요하다. `git config --global credential.helper store` 후 한 번 `git clone` 하면서
  사용자명 `iam-raccoon` + 토큰을 넣으면 이후 자동이다.

## 5.2 한 판 돌리기

자동 러너([5.6.1](#561-vtd-검증-러너))가 안 될 때, 그리고 **혼자 켤 때** 이 순서를 따른다(2026-08-19 확인).

| 단계 | 어디서 | 명령 |
|---|---|---|
| ① VTD 켜기 | VTD PC | `./bin/vtdStart.sh` |
| ② 시나리오 올리기 | VTD PC | `python3 load_scenario.py <시나리오>` |
| ③ 차가 제자리인지 확인 | 제어PC | `python3 vtd/ego_pose.py` |
| ④ 주행 | 제어PC | `bash drive.sh <시나리오>` 또는 `main.py` |
| ⑤ 채점 | 개발PC | `eval/check_run.py` |

### 5.2.1 VTD 켜기

```bash
# [VTD PC] 데스크톱에 로그인한 뒤
cd ~/Hexagon/VTD.2025.2 && ./bin/vtdStart.sh     # 이 터미널을 닫지 말 것(부모 프로세스다)
```

- ⚠️ **반드시 데스크톱에 로그인되어 있어야 한다.** `who` 에 `user :1` 이 있어야 하고, `(login screen)` 만 있으면 VtGui 가 안 뜬다.
- 1~1.5분 뒤 VtGui 창이 뜬다. `lmgrd` 는 절대 죽이지 말 것.
- 재부팅 뒤 기동·죽었을 때 복구는 [`ref/vtd_recovery.md`](ref/vtd_recovery.md).

### 5.2.2 시나리오 올리기

```bash
# [VTD PC] 이 한 줄이 시나리오 지정 + Apply + Init + Start 를 다 한다
cd ~/hlfma2026/vtd && python3 load_scenario.py HL_FMA_VTD_LivingLab_v1
#   끝에 `streaming=OK` 가 뜨면 성공. `streaming=FAIL` 이면 아래 '손으로'.
```

⚠️ **툴바 ✓(Apply)만 누르면 안 된다.** Apply 는 **프로젝트 기본 시나리오**를 올린다. 지금 `SampleProject.vpj` 의 기본값은
`Scenarios/TrafficDemo.xml` — **독일 맵**이다(2026-08-19 실측: 이 단계를 빠뜨려 독일 맵이 떴다). **어느 시나리오를 올릴지 먼저 정해 줘야 한다.**

스크립트가 실패했을 때 손으로:

```
1) 툴바의 초록 체크 ✓         -> 모듈 탭 6개가 뜬다 (보라 -> 초록으로 변함)
2) 메뉴  Simulation ▸ Init
3) 메뉴  Simulation ▸ Start
```

- 이 순서는 **프로젝트 기본 시나리오**를 올린다. 다른 걸 올리려면 `SampleProject.vpj` 의 `scenario=` 를 바꾸고
  **VTD 를 껐다 켜야** 한다(VtGui 는 그 파일을 기동할 때 한 번만 읽고, 실행 중 수정은 무시한다).
- ⚠️ **단축키 `Ctrl+Shift+I` 는 안 먹는다.** VtGui 로그에 `QAction::eventFilter: Ambiguous shortcut overload: Ctrl+Shift+I` 가 뜬다 —
  Qt 는 같은 단축키에 액션이 둘 이상이면 **아무것도 실행하지 않는다.** 메뉴에서 누를 것.
- ⚠️ **SCP `<SimCtrl><LoadScenario>` 는 이 빌드에서 무시된다.** 명령이 GUI 의 SCP 탭에 도착하는 게 보이는데 아무 일도 안 일어난다.
  어느 시나리오가 올라오는지는 프로젝트의 `scenario=` 속성이 정한다.

### 5.2.3 차가 제자리에 있는지 확인

```bash
# [제어PC]
cd ~/hlfma2026 && python3 vtd/ego_pose.py        # 주소 자동판별
```

맵 안 좌표면 정상이다. **`(7429, -3086)` 같은 값이면 시나리오가 안 올라간 것** — 5.2.2 를 다시.

### 5.2.4 주행

**쉬운 방법 — `drive.sh`** (시나리오 이름만 준다)

```bash
# [제어PC]  v1~7 처럼 줄여도 되고 전체 이름도 된다
bash ~/hlfma2026/drive.sh HL_FMA_VTD_LivingLab_v7
```

`drive.sh` 가 하는 일:

1. **시나리오 이름 → 경로 파일 짝**을 찾는다([5.2.5](#525-시나리오--경로-파일-짝)의 표를 코드로 박아 둔 것).
2. **ego 가 경로 시작점 30m 안에 있는지 검사**한다. 아니면 주행하지 않고 멈춘다 —
   `❌ 시나리오와 경로가 다른 코스다(600m 차이). VTD 에 올린 시나리오를 확인할 것`
3. **카메라**(공식 follower view)를 넣는다 — 손으로 올리면 매번 빠진다.
4. `main.py` 를 실행한다. CSV 는 `/tmp/run_<시나리오>.csv`.

환경변수: `HOST`/`VTD_HOST`(VTD 주소 — 안 주면 자기 IP 대역으로 자동 판별, `src/netcfg.py`) · `CSV`(기록 위치).
새 코스를 추가하려면 `case` 문에 한 줄 넣는다.

```bash
# 핵심만 추린 것 (전문은 drive.sh)
case "$NAME" in
  HL_FMA_VTD_LivingLab_v1|...|v6|EV_*) ROUTE="$R/planned_from_xml_v1.json" ;;
  HL_FMA_VTD_LivingLab_v7)             ROUTE="$R/planned_v7.json" ;;
  HL_FMA_WP9)                          ROUTE="$R/planned_wp9.json" ;;
  HL_FMA_OFFEX)                        ROUTE="$R/HL_FMA_OFFEX.json" ;;
esac
LANE="${ROUTE%.json}_lane.json"
#  ego 가 경로 시작점 30m 안인지 확인한 뒤에만 주행한다
cd src && python3 -u main.py --host "$HOST" --port 9910 \
  --scenario "$ROUTE" --lane-plan "$LANE" --tl-map "$R/tl_map_livinglab.json" \
  --log-csv "$CSV"
```

**직접 돌리기 — `main.py`**

```bash
# [제어PC]  src 안에서 실행한다(main.py 가 여기 있다)
cd ~/hlfma2026/src
python3 -u main.py --port 9910 \
  --scenario  ../routes/planned_from_xml_v1.json \
  --lane-plan ../routes/planned_from_xml_v1_lane.json \
  --tl-map    ../routes/tl_map_livinglab.json \
  --log-csv   /tmp/run.csv
```

### 5.2.5 시나리오 ↔ 경로 파일 짝

**이게 안 맞으면 차가 엉뚱하게 간다.**

| VTD 시나리오 | `--scenario` / `--lane-plan` |
|---|---|
| `HL_FMA_VTD_LivingLab_v1` ~ `v6` | `routes/planned_from_xml_v1.json` / `_lane.json` |
| `HL_FMA_VTD_LivingLab_v7` | `routes/planned_v7.json` / `_lane.json`  ⚠️ `from_xml` **없다** |
| `HL_FMA_WP9` | `routes/planned_wp9.json` / `_lane.json` |
| `HL_FMA_OFFEX` (주최측 예시 CSV) | `routes/HL_FMA_OFFEX.json` / `_lane.json` |
| 시험 판 `<코스>_VRU` · `_TRP` · `_TRV` · `_EV` 등 | 접미사를 뗀 원래 코스의 경로(시나리오에만 액터가 늘었다) |

- **주행 코드는 하나뿐이다.** 시나리오마다 바뀌는 건 **경로 파일(데이터)** 뿐이고 `src/` 는 그대로다.
  v1~v6 이 같은 경로를 쓰는 것도 **같은 코스에 상대차 이벤트만 다른 것**이기 때문이다. 당일에도 코드는 두고 경로 파일만 새로 만든다.
- ⚠️ 2026-08-19 실측: VTD 에 `HL_FMA_OFFEX` 를 올려놓고 `planned_from_xml_v1.json` 으로 달렸다. 두 코스의 시작점이
  **600m** 떨어져 있어 차가 엉뚱한 방향으로 직진하고 리스폰이 났다. **5.2.3 으로 반드시 확인할 것.**

### 5.2.6 채점

```bash
# [개발PC] CSV 를 가져와 채점한다
scp user@<제어PC 주소>:/tmp/run.csv /tmp/run.csv
python3 eval/check_run.py /tmp/run.csv 8.33 \
  routes/planned_from_xml_v1.json routes/planned_from_xml_v1_lane.json
```

⚠️ **경로와 차로계획을 같이 줘야** 완주·속도 판정이 맞는다. 안 주면 제한속도가 30 고정이라
"속도위반 1839프레임" 같은 거짓 결과가 나온다.

```bash
python3 eval/check_run.py /tmp/run.csv 8.33 routes/day.json routes/day_lane.json
python3 eval/check_lanes.py <xodr> /tmp/run.csv               # 차로 기준 채점
python3 eval/check_route.py routes/day.json <xodr>            # 경로 자체 검사
```

대회 문턱 그대로의 15개 항목 채점(`eval/score_fma.py`)은 [4.4 절](04_evaluation.md#44-실주행-채점--score_fmapy).

### 5.2.7 카메라

**화면에 차가 안 보이면** — 수동으로 시나리오를 올리면 카메라가 안 붙는다(`load_scenario.py` 는 마지막에 SCP 48179 로 넣는다).

```bash
# [VTD PC] 공식 follower view(ego 뒤 6m · 위 2m)로
python3 ~/hlfma2026/vtd/set_cams.py
```

⚠️ SCP 카메라 명령의 각도 속성은 `h/p/r` 이 아니라 **`dh/dp/dr`**(라디안)다 — 예전 표기는 VTD 가 **무시**했다.
follower 의 각도를 0 으로 두는 이유는 `vtd/set_cams.py` 주석에 있다.

**옆에서 보기(연습 전용)** — 앞범퍼가 정지선을 밟았는지는 공식 시점으로 판별이 안 된다(차 뒤 6m·위 2m 라 범퍼 앞 1~2m 노면이 차체에 가린다).

```bash
# [제어PC]
bash ~/hlfma2026/test.sh HL_FMA_NEW_G_TR          # 주행 + 오른쪽 옆면
bash ~/hlfma2026/test.sh HL_FMA_NEW_G_TR side_l   # 왼쪽 옆면
python3 ~/hlfma2026/vtd/set_cams.py --view side --host 192.168.50.11   # 시점만 바꾸기
```

끝나면 자동으로 공식 시점으로 되돌린다. **대회 주행은 `drive.sh`** 를 쓴다.
README GIF 녹화용 시점(`--view gif`)은 [img/README](img/README.md).

## 5.3 대회 당일 — 주최측 좌표로 달리기

### 5.3.1 무엇을 받나

**주최측이 X,Y 경유지 좌표를 준다.** 시나리오 XML 은 우리가 안 본다([강의 102:02] "참가팀 분들은 제어기만 가져오시면 돼요",
[강의 102:10] "구동하기 위한 경로는 찍어드릴 거예요").

> [강의 102:56] 저희가 **좌회전, 우회전해야 되는 구간에 하나씩** 찍어드릴 거예요
> [강의 103:20] 이거에 대한 좌표값 … **X, Y, 그거를 저희가 드릴 겁니다**
> [강의 103:28] 시작점은 X 297, Y 6m / 경유지는 957에 276 / 그리고 마이너스 885.312
> [강의 103:56] 이 포인트 **반경으로 10m, 20m** 안으로 들어오면 자동으로 시험 시스템이 종료
> [강의 104:10] 중간에 이탈 … **다 감점**이 되면서 … 리스폰처럼 정상 경로로 다시 잡아 줄 겁니다

**경로 CSV 형식** — **[공지]** 2026-08-19 로 확정. `docs/ref/route_example.csv` 가 주최측 예시 그대로다.

```
seq,x,y        VTD 월드 직교좌표, m. **egoX/egoY 와 같은 좌표계**라 변환 불필요.
1  출발
2,3   교차로1 진입·진출     <- 중간 지점은 **두 개씩 짝**이다
4,5   교차로2 진입·진출
6,7   교차로3 진입·진출
8  종료
```

- 경로는 최단거리 기준으로 선정돼 있고 **seq 순서대로** 통과해야 한다. 이탈하면 감점.
- 교차로 **진입·진출을 둘 다** 주므로 어느 연결로를 타야 하는지가 좌표로 못박힌다 — 차로·화살표 선택의 모호함이 크게 줄어든다.
  우리 탐색은 원래 '주어진 점들을 순서대로 다 거치는 최단 경로'라 이 형식과 그대로 맞는다.
- **출발 방향**은 다음 지점(=첫 교차로 진입점)에서 추정한다. 실측이 있으면 `--heading`.

**물체**(같은 공지)

- `objects[].x/y/z` 도 **월드 좌표**, ego 와 같은 계.
- `id = 0` 은 **빈 슬롯**. 배열은 항상 30개 고정 — `vtd_io.parse_data` 가 이미 거른다.
- Ego 제외 **수평 80m 이내**를 가까운 순으로 최대 30개. 우리 판단거리(전방 40m, 차체감시 25m)보다 넉넉해서 손댈 것이 없다.

### 5.3.2 경로 만들기 → 검사 → 차로 계획 → 주행

경로 파일은 코스마다 다르다. 맵(LivingLab)과 신호 정지선 DB(214개, 맵 전체)는 그대로 쓴다.

```bash
X=~/hlfma2026_map/HL_FMA_VTD_LivingLab.xodr   # 맵·시나리오 원본은 여기 보관

# ① 경로 — 주최측 CSV 로
python3 vtd/plan_route.py $X --from-csv docs/ref/route_example.csv routes/day.json
#    또는 좌표를 순서대로 나열 (경유지 몇 개든 상관없음)
python3 vtd/plan_route.py $X 297 6  957 276  824.031 9.672  routes/day.json
#    연습용으로 시나리오 XML 에서 뽑을 수도 있다
python3 vtd/plan_route.py $X --from-scenario <scenario.xml> routes/day.json

# ② 주행 전 검사 (수초). '이상 0곳 — 사용 가능' 이 아니면 쓰지 말 것 → 5.4
python3 eval/check_route.py routes/day.json $X

# ③ 차로 계획
python3 vtd/build_lane_plan.py $X routes/day.json routes/day_lane.json

# ④ 주행 [제어PC]
cd src && python3 main.py --host <VTD_IP> --scenario ../routes/day.json \
  --tl-map ../routes/tl_map_livinglab.json --lane-plan ../routes/day_lane.json
```

- 검증(2026-08-16): XML 에서 뽑아 계산한 경로로 **완주 134s · 신호위반 0 · 리스폰 0 · 접촉 0 · 속도위반 0**.
  공식 v1~v7 을 계획 경로로 돌리면 **전부 리스폰 0**(녹화 경로는 판당 1건) — 좌회전 차로가 경로에 들어 있어 제어기가 나중에 옮길 일이 없다.
- 주최측 예시 CSV 는 처음 물렸을 때 **계산조차 안 됐다**(차로번호 체계·중간에 사라지는 차로·한 도로 두 번 차선변경). 고친 뒤 **592점 821m · 이상 0곳**. 경위: [부록 A](99_dev_log.md).
- ⚠️ XML 의 path 는 waypoint2 에서 끝난다(834m). 녹화본이 884m 인 것은 `EndAction="continue"` 로 교차로를 42m 더 지나간 구간을 담았기 때문이다. 더 가야 하면 목표 좌표를 직접 준다.
- ⚠️ 출발점이 반대편 차로 위라 스폰 방향으로 달릴 수 없는 코스는 **ego 좌표에서 계획**한다([5.5.10](#5510-시나리오-파일-관리)).

### 5.3.3 GT 물체 범위 — 갑자기 튀어나오는 것은 없나

전 23판 실측(2026-08-20):

| 물음 | 실측 |
| --- | --- |
| 80m 상한이 진짜인가 | **최대 80.0m**, 초과 0건 |
| 30칸이 모자란 적 있나 | **최대 4칸**(평균 1칸 미만). 넘쳐도 잘리는 건 **먼 것부터**라 가까운 위험물은 안 밀린다 |
| 차·사람만 오나 | 아니다. `EV_OBSTACLE` 의 **0.2×0.5×0.6m 라바콘**이 674프레임 내내 들어왔다 |
| 주행 중 새로 나타난 물체의 최근접 첫 등장 | **79.6m** — 전부 80m 링에서 들어왔다. 더 가까이서 생긴 적 없다 |

- 여유도 넉넉하다. 우리가 실제로 낸 감속도 대표값 **6.6 m/s²**(최대 9.0) 기준 50km/h 정지거리 **14.6m**, 30km/h **5.3m** — 80m 경계에서 예고 없이 튀어나와도 5배 여유다.
- **못 보는 것은 맵 배경물**(가로수·전봇대·가드레일)뿐이다. xodr 배경이라 플레이어가 아니어서 리스트에 없다. 다만 고정이고 차도 밖이며,
  트리거도 채점 시점도 못 잡아 **평가 이벤트로 쓸 수 없다.** "튀어나오게" 만들려면 객체여야 하고 그러면 GT 로 온다.
- ⚠️ 이건 **사후 계산으로 안 것**이라 대회 당일 한 판에서 깨져도 모른다. 그래서 `check_run.py` 가 **[GT 갑툭튀]** 줄을 매 판 찍는다(30m 안 첫 등장이면 경고).

## 5.4 경로 검사 — `eval/check_route.py`

**경로를 만들면 반드시 검사한다.**

```bash
X=~/hlfma2026_map/HL_FMA_VTD_LivingLab.xodr
python3 eval/check_route.py routes/day.json $X [검증된_기준경로.json]
```

| 검사 | 잡는 것 |
|---|---|
| ① 헤딩 반전 · ② 점 간격 · ⑤ 곡률 | 따라갈 수 없는 경로(xodr 없이도 됨) |
| ③ 도로 밖 · ④ 역주행 | 지도 대조 |
| ⑥ 차로 점유 | 도로별 (차로번호, 안쪽에서 몇 번째). **판정 안 하고 보여준다** |
| ⑦ 기준경로 대조 | **이격 중앙값 > 1.6m 면 실패** — 한 차로 밀린 것 |
| ⑧ 경로에 박힌 차선변경 | **교차로 안이면 실패.** 몇 곳인지도 보여준다. 경로 파일의 `ego_lanes`(도로,차로,s,t)로 **그 도로에서만** 잰다 |
| ⑨ 노면 화살표 | 지정차로 지시위반(좌회전 차선에서 직진 등). **세어서 보여준다** |

읽는 법:

- **기준경로 없이 "이상 0곳"을 믿지 말 것.** ⑦ 은 2026-08-18 에 전 구간 한 차로(3.20m) 밖으로 밀린 경로가 ①~⑤ 를 모두 통과해 생긴 검사다. 경위: [부록 A](99_dev_log.md).
- ⚠️ ⑦ 은 **같은 코스의 검증본**과만 대조한다. WP9(3322m)을 v1~v6 기준선과 비교하면 90m 로 나오는데 그건 오탐이다.
- ⑨ 를 판정하지 않고 세기만 하는 이유: 맵 자체가 모순인 곳이 있다(WP9 의 `road 396`·`road 190` — 지킬 차로가 없다). 막지 않고 벌점만 무는 처리가 맞다.
- ⑧⑨ 는 2026-08-19 실주행 화면에서 잡은 경로 생성기 버그 2개("사거리 한복판 차선변경", "좌회전 차선에서 직진") 뒤에 생겼다. 경위: [부록 A](99_dev_log.md).

경로를 새로 만들었으면 `bash vtd/swap_routes.sh <디렉터리>` 로 `routes/` 에 들여놓는다.
⚠️ **회귀가 도는 중에는 실행하지 말 것** — 앞판과 뒷판이 다른 경로로 돌아 비교가 무의미해진다.

## 5.5 시나리오

### 5.5.1 한눈에

⚠️ 아래는 **우리가 만든 시험 체계**다. 주최측 평가 항목과는 다르다 — 대회 전에는 **[강의 117:33]** "원래 설명회 때 10가지로 말씀을 드렸었는데
지금 한 5가지 정도 더 추가 될 것 같고요", **[강의 117:22]** "점수표는 사전에 제공은 안 되고요, 대회 당일에 알려 드릴 겁니다" 뿐이었다.
당일 확인된 15개 항목은 [1장](01_problem.md), 도교법 조문 대조는 [3장](03_traffic_law.md).

| 종류 | 개수 | 출처 | 무엇을 잡나 |
|---|---|---|---|
| **공식 코스** | 8 | 주최측 제공 **[공지]** | "정해진 코스를 제대로 도는가" |
| **이벤트** `EV_*` | 10 | **우리가 만듦** (`vtd/make_event_scenarios.py`) | "위험 상황에서 안 박는가" |
| **새 코스** `HL_FMA_NEW_*` | 6 (연습) | **우리가 만듦** (`vtd/make_course.py`) | "처음 보는 좌표로 경로를 짤 수 있는가" |
| 새 코스 + 이벤트 `<코스>_EV` | 6 | `vtd/add_events.py` | "모르는 길에서 위험 상황" |
| 취약대상 `<코스>_VRU` | — | `vtd/add_vru.py` | 정지 휠체어·자전거, 저속 선행차 |
| 횡단보도 보행자 `<코스>_TRP` | 6코스 | `vtd/add_crosswalk_peds.py` | 건너는 사람 앞 정지 |
| 주변교통 + 취약대상 `<코스>_TRV` | — | `vtd/add_vru_traffic.py` | 교통 속 보행자·달리는 자전거·휠체어 |
| 비보호 좌회전 `HL_FMA_LEFTONC` | 1 | `vtd/add_oncoming.py` | 좌회전 중 가로지르는 차 양보 |

공식 8판과 이벤트 10판이 `run_regression.sh` 의 **18판**이다(v1~v6 이 6 + v7 + WP9 = 8, 거기에 EV 10).

### 5.5.2 공식 8판

- `HL_FMA_VTD_LivingLab_v1`~`v7`, `HL_FMA_WP9`(9경유지 3.3km).
- v1~v6 은 **코스가 같고** 상대차 배치만 다르다 — 그래서 경로 파일도 `planned_from_xml_v1.json` 하나를 같이 쓴다. v7 은 다른 코스(`planned_v7.json`)다.
- v7 은 오랫동안 40km/h 상한으로 돌렸는데 **2026-08-20 에 뺐다** — 상한의 근거였던 '붕괴'는 2026-08-18 에 진범(경로 생성기)을 고쳐 이미 없어졌고,
  상한만 남아 있었다. 상한 없이 실측 **88s · 40.6km/h · 25Hz · 전부 0**(상한 있을 때 123s).
- ⚠️ **무단횡단 사고는 EV 가 아니라 공식 v6 안에 원래 들어 있던 것**이다. 우리가 심은 이벤트만 위험한 게 아니다.

### 5.5.3 이벤트 10판 `EV_*`

**공식 코스만으로는 위험 상황을 한 번도 안 겪는다.** 2026-08-14 실측: 공식 v1~v7 은 상대차가 우리 차선에 없거나(경로에서 9m 밖 주차)
아예 스폰되지 않아(v2) **끼어들기·급정거·인레인 장애물이 VTD 에서 한 번도 검증되지 않았다.**
그래서 공식 v5(ego 만 있는 깨끗한 코스)를 바탕으로 같은 코스·같은 경로 위에 이벤트를 심었다.

| | 심은 상황 | | 심은 상황 |
|---|---|---|---|
| `EV_CUTIN` | 옆 차선 차가 끼어듦 | `EV_REARPASS` | 뒤에서 추월당함 |
| `EV_LEADBRAKE` | 앞차 급정거 | `EV_BOTHBLOCK` | 양쪽 다 막힘 |
| `EV_PED` | 보행자 | `EV_CURVE` | 커브 구간 |
| `EV_OBSTACLE` | 내 차선에 정지 장애물 | `EV_COMBO_PASSPED` | 사람＋정지차 |
| `EV_ONCOMING` | 마주오는 차 | `EV_COMBO_CHAIN` | 연쇄 |

- 트리거가 `<PosRelative Pivot="Ego" Distance="N"/>`(ego 가 N m 안에 접근)이라 **매번 같은 자리에서 재현된다.**
- 배치는 `PosAbsolute` 로 실제 x,y 를 준다 — `PathRef` 로 넣으면 스폰이 안 되는 경우가 있었다(EV_CUTIN 1차 실패).

### 5.5.4 새 코스 `HL_FMA_NEW_*`

회귀 18판이 무결점이어도 그건 **코스 모양 3개**(v1~v6 공용 / v7 / WP9)에서만 확인한 것이다. 당일엔 처음 보는 좌표를 받는다.
그래서 안 밟아 본 도로로 코스를 만들어 **회귀와 따로** 돌린다. 2026-08-20 에 6개 전부 VTD 로 **두 판씩 돌렸고 둘 다 무결점**이다(합 21.1km × 2).

| 코스 | 거리 | 시간 | 커버리지 | 종점오차 | 리스폰·신호·접촉·속도위반 |
|---|---|---|---|---|---|
| A | 4257m | 681s | 100% | 1m | 전부 0 |
| B | 2605m | 415s | 100% | 1m | 전부 0 |
| D | 2753m | 372s | 100% | 1m | 전부 0 |
| E | 5238m | 644s | 100% | 1m | 전부 0 |
| G | 3426m | 540s | 100% | 1m | 전부 0 |
| H | 2802m | 407s | 100% | 1m | 전부 0 |

- **공식 코스 3개 밖에서 20.6km 를 무결점으로 돌았다.**
- 여기까지 오는 데 세 가지를 고쳐야 했다 — 깨진 XML(D·E·G), 출발점(ego 자리), 비보호 좌회전(E). **셋 다 공식 18판으로는 안 나왔을 것들이다.**
  코스 A 는 "좌표만으로는 출발 방향을 알 수 없다"는 버그를 잡았다. 경위: [부록 A](99_dev_log.md).

### 5.5.5 새 코스 + 동적 이벤트 `<코스>_EV`

새 코스 6개에는 **상대차가 하나도 없다.** 그래서 `vtd/add_events.py` 로 **6개 코스 전부**에 내 차로 정지차 + 대향 직진차를 심었다
(예: `scenarios/HL_FMA_NEW_D_EV.xml`).

| 코스 | 결과 | 최소 실여유 |
|---|---|---|
| A | 완주 · 커버리지 100% · 전부 0 | +1.55m (`OVT:PASS`) |
| B | 완주 · 커버리지 100% · 전부 0 | +1.33m (`OVT:PASS`) |
| D | 완주 · 커버리지 100% · 전부 0 | +0.93m (`SIDE_CAUTION`) |
| E | 완주 · 커버리지 100% · 전부 0 | +1.49m (`OVT:PASS`) |
| G | 완주 · 커버리지 100% · 전부 0 | +1.14m ※결함을 고친 뒤 |
| H | 완주 · 커버리지 100% · 전부 0 | +1.33m (`OVT:PASS`) |

- 정지차는 추월로 넘고, 대향차는 `SIDE_CAUTION`/서행으로 통과한다.
- 코스 G 에서 **좁은 차로에서 추월을 포기하고 갇히는** 결함이 나와 고쳤다(추월기가 비킬 폭을 상수 3.5m 대신 차로계획의 폭으로 쓴다).
- 판을 놓을 때 지킬 것: **대향차는 `sample_lane` 으로 반대 차로 중심**에(경로점 위에 놓으면 VTD 내부 플레이어는 충돌을 무시하고 뚫고 지나간다),
  **정지차는 교차로 밖**에 놓는다(교차로 안이면 갇힘 탈출이 리스폰을 부른다). 경위: [부록 A](99_dev_log.md).

### 5.5.6 취약대상 `<코스>_VRU` — 휠체어 · 자전거 · 저속 선행차

"대회장에 자전거·휠체어가 나오면 긴급제동이나 회피를 할 수 있는지"를 보려고 만들었다(2026-09-09). 오프라인 판 3개(`eval/scenarios/vru_*.json`)로
먼저 돌려 **결함 하나**를 찾았고, 이 판은 그걸 실제 코스에서 다시 보기 위한 것이다.

```bash
# [제어PC] 코스에 심는다 -> scenarios/<코스>_VRU.xml
python3 vtd/add_vru.py map/HL_FMA_VTD_LivingLab.xodr HL_FMA_NEW_A
python3 vtd/check_scenario.py map/HL_FMA_VTD_LivingLab.xodr scenarios/HL_FMA_NEW_A_VRU.xml
git add scenarios/HL_FMA_NEW_A_VRU.xml && git commit -m "..." && git push

# [VTD PC] 받아서 올린다
cd ~/hlfma2026 && git pull && cd vtd && python3 load_scenario.py HL_FMA_NEW_A_VRU

# [제어PC] 주행 — 경로는 원래 코스 그대로다
bash drive.sh HL_FMA_NEW_A
python3 eval/score_fma.py <run.csv> --route routes/HL_FMA_NEW_A.json \
                                    --lane  routes/HL_FMA_NEW_A_lane.json
```

각 자리는 교차로 밖 + 옆 여유 3.2m 이상으로 골랐으므로 "길이 없어서 못 갔다"는 변명이 성립하지 않는다.

| 자리 | 무엇이 있나 | 통과 | 실패 |
|---|---|---|---|
| ~25% | 정지 **휠체어**(H 0.92) | 옆 차로로 옮겨 통과, 실여유 1.2m 이상 | 앞에서 굳는다 / 스쳐 지나간다 |
| ~45% | 정지 **자전거**(H 1.10) | 위와 같다 | 위와 같다 |
| ~65% | **4.5m/s(16km/h) 선행차** | 추월하고 간다 | **뒤에 붙어 끝까지 따라간다** ← 오프라인에서 이미 재현됨 |

- ⚠️ 셋째는 **실패가 예상되는 자리**다. 오프라인 `vru_bicycle_slow` 가 이미 미완주(85점)로 재현했고, 막는 것을 승용차로 바꿔도 같았다.
  VTD 로 보려는 것은 '되는지'가 아니라 **실제 코스에서 얼마나 비싼가**다 — 제한시간 안에 완주가 되는지, 뒤에 정체가 쌓이는지.
- ⚠️ **모델이 화면에 안 보이면** `--def-wheelchair DummyPerson` 으로 다시 만든다. 그때는 치수가 성인 보행자라 **H 0.92·1.10 검증은 안 된 것**이니 반드시 그렇게 기록한다.
- ⚠️ 오브젝트는 **움직이지 않는다**(`add_hazards.py` 주석의 실측). VTD 로 보는 것은 '정지한 VRU 회피'뿐이고, '뛰어드는 휠체어에 급제동'은 오프라인 `vru_wheelchair_dart` 가 담당한다.
- ⚠️ 휠체어를 `<Object Definition="wheelchair_adult">` 로 넣으면 VTD 가 못 찾아 안 띄운다([5.5.8](#558-주변교통--취약대상-코스_trv)). `add_vru.py` 는 아직 물체로 넣는다.

### 5.5.7 횡단보도 보행자 `<코스>_TRP`

주변교통(TR) 판에 **횡단보도를 건너는 사람**을 심는다(2026-09-09, 6코스 모두 만들어 뒀다). 걷는 사람은 VTD 의 `Character` 라 물체와 문법이 다르다 —
VTD 자체 샘플(`TrafficDemo.xml`)에서 가져왔다. `Player`/`Description Type="Hannah"` 는 **차량** 문법이라 안 뜨고,
"Hannah" 는 캐릭터 종류가 아니라 **외형(Appearance)** 이름이다(`Distros/Current/Config/Players/Pedestrians/PedCal3DCfg.xml`).

```bash
# [제어PC] 주변교통(TR) 판에 심는다 -> scenarios/<코스>_TRP.xml
python3 vtd/add_crosswalk_peds.py map/HL_FMA_VTD_LivingLab.xodr HL_FMA_NEW_A
# [VTD PC] 올린다 — ⚠️ 반드시 이름을 준다(인자 없이 부르면 전부 올린다)
bash vtd/push_scenarios.sh HL_FMA_NEW_A_TRP
# [제어PC] 주행 — 경로는 원래 코스 그대로
bash drive.sh HL_FMA_NEW_A_TRP
```

- 무엇이 심기나: 경로가 실제로 지나는 횡단보도 가운데 서로 150m 이상 떨어진 **4곳**. 각 자리에 사람 하나가 인도 가장자리에 서 있다가
  **자차가 45m 안에 오면** 1.3m/s 로 건넌다(첫 사람은 우측 인도에서 왼쪽으로, 다음은 반대로 번갈아).
- 신호 유무를 안 가린다 — 우리 녹색에 건너는 사람은 무단횡단자지만 그래도 서야 한다[법 제27조①].
- 봐야 할 것: 사람이 길 위에 있는 동안 **앞범퍼 5m 앞(YIELD_PED)** 에 서는가, 다 건너면 출발하는가(PED_LEAVING), 채점 항목⑩(반경 3m 무정차 통과)이 0 인가.
- 2026-09-14 코스 A 판 VTD 녹화에서 사람이 뜨고 걸었다([img/README](img/README.md)). 다른 코스에서 안 보이면 `--appearance Hannah1` 처럼 바꿔 다시 만든다.
  걷지 않고 서 있기만 하면 `CharacterActions` 의 트리거(`PosRelative Pivot="Ego" Distance=45`)가 안 걸린 것이니 `--trigger 60` 으로 넓혀 본다.

### 5.5.8 주변교통 + 취약대상 `<코스>_TRV`

"대회에서 어떤 돌발 환경이 나올지 모르니 차량 있는 게 맞다"는 요청으로 만든 판이다(2026-09-09). `<코스>_TR.xml`(주변교통 50대) 위에 셋을 얹어 `<코스>_TRV.xml` 을 만든다.

| 무엇 | 어떻게 | 어디에 |
|---|---|---|
| 걷는 사람 4명 | Character · `Move="walk"` 1.3m/s · 자차 45m 접근 시 출발 | 경로가 지나는 횡단보도 |
| **달리는 자전거 2대** | Character · `Move="bicycle_ride"` 4.4m/s(16km/h) · 80m 접근 시 출발 | 내 차로, 앞으로 ~140m |
| 정지한 휠체어 1대 | 캐릭터 `male_adult` + 동작 `wheelchair_idle` | 차로 중심에서 우 1.0m |

```bash
python3 vtd/add_vru_traffic.py map/HL_FMA_VTD_LivingLab.xodr HL_FMA_NEW_A
bash vtd/push_scenarios.sh HL_FMA_NEW_A_TRV      # ⚠️ 이름 필수
bash drive.sh HL_FMA_NEW_A_TRV                   # 경로는 접미사를 떼고 원래 코스 것을 쓴다
```

봐야 할 것:

1. 횡단보도 보행자 앞에서 서는가(항목⑩ 0, 항목⑫ 0).
2. **달리는 자전거를 지나가는가** — 핵심이다. 오프라인 `vru_bicycle_slow` 에서 이미 **미완주(85점)** 로 재현됐다
   (`overtake.py` 가 움직이는 것을 전부 추월 대상에서 뺀다). 주변교통까지 있는 실제 코스에서 얼마나 비싼지 — 완주는 하나, 몇 분 걸리나.
3. 정지한 휠체어(H 0.92)를 밟지 않고 비켜 가는가.

- ⚠️ **휠체어는 물체가 아니라 사람의 동작이다**(2026-09-11 OMEN 카탈로그 실측). `Config/Players/Pedestrians/PedCal3DCfg.xml` 의 male_adult 에
  `wheelchair_idle`(0m/s)·`wheelchair_ride`(1.35m/s) 가 있고 둘 다 `wheelchair_adult` 골격을 붙이는 compound 다 — 자전거(`bicycle_ride`)와 같은 방식.
- ⚠️ VTD 에서 자전거가 뜨고 달리는지는 **실측 전이다.** 안 보이면 `--appearance Hannah1`, 서 있기만 하면 `--bike-trigger 120` 으로 다시 만든다.
- ⚠️ `Move=` 에 아무 이름이나 넣으면 안 된다. 샘플 시나리오에 실제로 쓰인 값만 쓴다 — walk · jog · run · wait · **bicycle_ride** · listen · talk. **휠체어 동작은 없다.**

### 5.5.9 비보호 좌회전 판 `HL_FMA_LEFTONC`

화살표 없는 신호에서 좌회전하며 가로지르는 차에 양보하는지 보는 판이다(2026-08-20). 우리 맵의 해당 교차로(tl 108)에는 대향차가 안 와서 따로 만들었다.

- `scenarios/HL_FMA_LEFTONC.xml` — **그 교차로만 도는 259m 코스** + 차 3대. 코스 E 로는 그 교차로까지 580초가 걸려 타이밍을 못 맞춘다.
- `vtd/add_oncoming.py` — 시나리오에 차를 심는다. `PosAbsolute` 로 놓고, 정지해 있다가 ego 가 트리거 거리 안에 들면 출발한다.
  차는 **들어갈 도로의 반대 차로 중심**(`sample_lane`)에 놓는다 — 그 교차로는 T자로라 ego 정면에 도로가 없어, 처음엔 허공에 놓여 스폰되지 않았다.
- 확인(고친 뒤 VTD): 완주 · 커버리지 100% · 리스폰 0 · 접촉 0 · **대기 중 최소 실여유 14.7m** · `STALL_ESCAPE` 0프레임.
- 비보호 판정(녹색 두 번에 화살표 없음)·양보 판정(최근접 접근 CPA)이 어떻게 정해졌는지는 [부록 A](99_dev_log.md).
  회귀 판은 `eval/scenarios/unprotected_left_oncoming.json`(수정 전 50점 · 충돌 → 100점).

### 5.5.10 시나리오 파일 관리

우리가 만든 시나리오는 **레포에 있다.** OMEN 은 배포 대상일 뿐이라 밀려도 복구된다.

| | |
|---|---|
| `scenarios/*.xml` | 우리가 만든 시나리오(2026-08-20 정리 때 17개: EV 10 + NEW 6 + OFFEX) |
| `scenarios/course_waypoints.json` | 코스별 **원래 경유지 x,y** — 이것만 있으면 코스를 다시 만들 수 있다 |
| `routes/HL_FMA_NEW_*.json` (+`_lane`) | 6개 코스의 경로·차로계획 |

- 좌표는 `plan_route.py --from-scenario` 로 XML 에서 되살렸다(Path 의 `TrackId`+`s` 를 월드 좌표로 환산한다).
- `course_waypoints.json` 에는 코스별로 `spawn_heading_deg`(실측), `ego_spawn`(그때 좌표), `start_from_ego`(ego 좌표에서 계획해야 하나)도 적어 뒀다.
  첫 경유지를 담는 차로가 스폰 방향으로 달릴 수 없는 코스(D·E·G)는 **첫 경유지 대신 ego 좌표에서 계획**한다.
  ⚠️ 그 경유지가 반대 차로 위라면 그 차로는 지나지 않는다 — ego 가 4.6m 안에 있어 커버리지 판정(6m)에는 든다.

올리기:

```bash
bash vtd/push_scenarios.sh                      # 전부 올리기
bash vtd/push_scenarios.sh EV_PED HL_FMA_NEW_D  # 골라서
```

- ⚠️ 이 스크립트는 **공식 시나리오를 올리지 않는다.** 주최측이 준 것이라 덮어쓰면 원본을 잃는다.
- ⚠️ 올리기 전에 XML 이 파싱되는지 확인한다. `make_course.py` 가 로그를 stdout 에 섞던 시절 만든 D·E·G 가 **앞 302바이트 한글 로그** 때문에
  안 올라갔고, 그게 `(0,0) 스폰` 증상의 상당 부분이었다. 경위: [부록 A](99_dev_log.md).

**OMEN 시나리오 폴더 정리** — 지우지 않고 `_attic/` 으로 옮긴다. 2026-08-20 기준 xml 이 **59개**였는데 실제로 쓰는 건 25개였다.
무엇이 무엇인지 아무 데도 안 적혀 있어 목록을 만든 것이 `vtd/tidy_scenarios.sh` 다.

```bash
bash vtd/tidy_scenarios.sh        # 무엇을 치울지 보여주기만 한다(기본)
bash vtd/tidy_scenarios.sh --go   # _attic/ 으로 옮긴다
```

- 치운 26개: `REC_v1~v7`·`REC2_*`(경로 녹화용 9) · `V7_NOCAN`·`V7_PYLON` 등(v7 붕괴 추적용 10) · `HL_FMA_OFF5/6`(2) · `HL_FMA_DEMO/DRIVE/ROUTE/WATCH/VTD_LivingLab`(5).
- 공식 시나리오와 VTD 기본 예제(TrafficDemo·Parking·Glare·Crossing8Demo…)는 건드리지 않는다. 정리 **직후 회귀 18판이 그대로 무결점**이었다.

## 5.6 검증 — 러너 · 오프라인 회귀 · 테스트

검증 층 전체(유닛테스트 37파일 · 오프라인 35판 · 실주행 채점)의 설계와 지금 규모는 [4장](04_evaluation.md)에 있다. 여기는 돌리는 법이다.

### 5.6.1 VTD 검증 러너

```bash
# [개발PC] 한 판 (로드 -> 주행 -> 채점)
bash vtd/run_scenario.sh HL_FMA_VTD_LivingLab_v1 routes/planned_from_xml_v1.json

# 붕괴 코스는 속도 상한을 걸어서 (40km/h)
bash vtd/run_scenario_slow.sh HL_FMA_VTD_LivingLab_v7 routes/planned_v7.json

# 전체 18판 (공식 8 + 이벤트 10)
bash vtd/run_regression.sh
```

**직접 `main.py` 를 돌리지 말고 이걸 쓸 것** — 아래 안전장치가 전부 빠진다.

| 안전장치 | 왜 (실측) |
|---|---|
| 매 판 `src/*.py` 배포 | 개발머신에서 고치고 제어PC 옛 코드로 검증할 뻔했다 |
| 지난 판 CSV 선삭제 | 로드 실패했는데 **4시간 전 CSV 로 채점**돼 '통과'가 나올 뻔했다 |
| 로드 실패 시 30초 후 재시도 | 강제 재시작 직후 첫 로드가 잘 실패한다(4건 중 3건) |
| **정지 60초 -> 조기 중단** | 갇힌 판이 1027초를 채워 검증 한 바퀴가 30분씩 날아갔다 |
| 타임아웃 판 뒤 `ensure_vtd.sh --force` | 부분 pkill 은 alive() 를 속여 9910 없이 다음 판이 돈다 |
| 프레임율 **최악 20초 버킷** 경고 | 중앙값은 60% 붕괴를 25.0Hz 로 가린다 |

### 5.6.2 오프라인 회귀 — VTD 없이

```bash
bash eval/run_all.sh                                     # 전 판 + 기준선 대조 — 이걸 쓴다
bash eval/run_eval.sh eval/scenarios/v3_leadbrake.json   # 한 판만
bash eval/run_eval.sh eval/scenarios/ped_crossing.json     # 한 종
for s in eval/scenarios/*.json; do bash eval/run_eval.sh "$s"; done   # 기준선 대조 없이 전부
```

- `eval/mock_vtd.py`(217줄)가 **9910 을 그대로 열고** `vtd_io.pack_data`/`parse_ctrl` 로 진짜와 **같은 1109B 패킷**을 주고받는다. 제어기는 진짜인지 가짜인지 모른다.
  ego 는 자전거 모델로 굴리고 상대차·보행자는 시나리오 대본대로 움직인다. `evaluate.py` 가 충돌·신호·속도·목표도달을 도교법 100점 감점제로 채점한다.
- `passped_junction` 은 `run_eval.sh` 한 줄로는 85점인데 **결함이 아니라 실행법 문제**다 — `--lane-plan` 을 같이 줘야 한다(그러면 100점). 2026-08-20 기준 14종 중 13종 100점.
- 목표 도달 반경은 **15m** — **[강의 103:56]** "이 포인트 반경으로 10m, 20m 안으로 들어오면 자동으로 시험 시스템이 종료" 를 따른 값이고 `check_run.py` 와 같다.
  예전엔 `evaluate.py` 만 3.0m(차 길이 4.8m 보다 작다)라, `cutin_lead` 에서 앞차가 **목표 좌표 (220,0) 위에 정차**해 우리가 그 뒤 10.1m 에 정상적으로 선 것을 미도달(−15)로 채점했다.
  **제어기는 옳았고 채점 기준이 틀렸다.** 채점기끼리 기준이 다르면 이런 걸 놓친다.
- ⚠️ 한계: 자전거 모델이고 xodr 를 안 본다. **"한 차로 밀림" 같은 건 여기서 절대 안 잡힌다**(`check_route.py` 의 몫). 프레임 붕괴·리스폰도 안 잡힌다. 최종 확인은 VTD 가 필요하다.

### 5.6.3 회귀를 읽는 법 — PASS/FAIL 이 아니라 점수

**합격선 60점은 대회 기준이지 회귀 기준이 아니다.** 2026-08-30 변이 검증: 적신호를 무시하도록 일부러 버그를 심었더니 채점기는 `-20` 을 제대로 매겼는데
**80점이라 `✅ PASS` 로 표시됐다.** "21판 전부 PASS" 만 보면 신호위반이 그냥 통과한다.

→ `eval/run_all.sh` 는 `eval/expected_scores.json`(전판 100 · `passped_junction` 85)과 대조해 **1점이라도 떨어지면 ❌** 로 낸다.
의도한 개선으로 점수가 오르면 그 파일도 같이 갱신할 것.

### 5.6.4 이 회귀가 못 잡는 것

| 못 보는 것 | 왜 | 누가 대신 보나 |
|---|---|---|
| 곡선 거동 | 21판 중 **19판이 완전 직선**, 나머지도 꺾은선 4~5점 | VTD 실주행. 그 주의 곡선 버그 3건(모퉁이 투영·커브 CPA·회전 지시등)이 전부 실주행에서만 나왔다 |
| 방향지시등 | 채점 항목에 없다 — 안 켜도 100점 | `eval/check_lanes.py` ⑤ · 유닛테스트 |
| 차선 밟기·역주행 | mock 은 1차원 도로다 | `eval/check_lanes.py` ①② |
| 일시정지 | 보행자 3m 근접만 본다 | `eval/check_crosswalks.py` |

**스택 통째 변이로는 채점기를 검증할 수 없다.** 9개 변이 중 2개만 감점됐는데, 나머지는 채점기가 눈이 멀어서가 아니라 **다른 안전층이 흡수**했기 때문이다 —
정지물 무시 변이는 추월 FSM 이 대신 피했고(`OBSTACLE_STOP` 649→0프레임인데 접촉 0), 제한속도 변이는 Behavior 가 구역제한을 독립적으로 한 번 더 잠가 속도가 안 올라갔다.
단일점 고장에 강하다는 뜻이라 좋은 소식이지만, 그래서 **채점기의 눈은 합성 궤적으로 따로 검증한다**(`tests/test_evaluate.py`).

### 5.6.5 유닛테스트

소켓·시뮬이 필요 없다. 각 항목은 그 수정을 되돌리면 실제로 실패한다 — "고쳤다"는 주장마다 그걸 잡는 테스트가 붙어 있다.

```bash
for f in tests/*.py; do python3 "$f" || echo "❌ $f"; done      # 전부 (~20초)
```

아래 표는 **321개**이던 시점의 구성이다(지금 규모는 [4장](04_evaluation.md)).

| 파일 | 개수 | 무엇을 지키나 |
|---|---|---|
| `test_behavior.py` | 91 | 신호·보행자·앞차 ACC·끼어들기·근접통과·적신호 우회전·비보호 좌회전 |
| `test_plan_route.py` | 68 | 차선변경 비용·동점깨기·완료여유·차로번호 재부여·횡단보도 회피·계단식 다차로·지시등 누락 |
| `test_nudge.py` | 65 | 장애물 회피·차로 선택·추월 금지·갇힘 탈출·회전 지시등 선행·차선변경 뒤차 양보 |
| `test_arrows.py` | 36 | 노면 화살표 지정차로·보호구역 제한·이동램프 |
| `test_route_frame.py` | 34 | 경로투영·제어·프로토콜·속도추정·수신끊김·정차유지 |
| `test_clearance.py` | 9 | 차체 실여유(SAT) |
| `test_netcfg.py` | 7 | VTD 주소 자동판별(대회망/연습망/환경변수/포트 불변) |
| `test_evaluate.py` | 11 | **채점기 자체** — 감점 항목이 하나씩 정확히 발화하는지 |

## 5.7 화면 녹화

### 5.7.1 RTSP 전방 카메라 — 개발PC 에서 장면 확인

VTD 는 **8554** 로 전방 카메라를 내보낸다(2026-08-22 확인). 서버는 MediaMTX(`gortsplib`)이고 경로는 **`/front`** —
다른 이름은 전부 400 을 뱉어서 raw DESCRIBE 로 하나씩 찔러 찾았다. 1920x1080 H.264 20fps. tailscale 로 개발PC 에서 바로 닿는다.

```bash
# [개발PC]
bash vtd/record_run.sh /tmp/runA.mp4 1200        # 주행과 동시에
python3 vtd/grab_at.py /tmp/runA.mp4 /tmp/run_X.csv /tmp/shots CROSSWALK_STOP
```

두 번째가 핵심이다 — **CSV 의 `t` 로 그 순간 프레임을 꺼낸다.** "정지선 앞 2.3m 에 섰다"는 숫자를 장면으로 확인할 수 있다.

- ⚠️ 반드시 **조각 mp4**로 쓴다(`frag_keyframe+empty_moov`). 일반 mp4 는 moov 를 마지막에 써서 녹화 중에는 열리지도 않고, ffmpeg 이 죽으면 그때까지 찍은 것도 통째로 못 쓴다.
- ⚠️ **전방 센서 카메라라 자차 방향지시등은 안 보인다.** 정지선·횡단보도·앞차는 보이지만 깜빡이 오점등은 여전히 사람 눈이 필요하다. 차체에 화면 절반이 가려 원거리 차량 식별도 약하다.
- ⚠️ 9910 은 건드리지 않는다(읽기 전용 RTSP). 대역폭 때문에 기본은 960x540@10 으로 낮췄다.

### 5.7.2 README GIF — 중계 창 녹화

차 전체가 보이는 중계 창(`mainRS_Broadcast`)을 창 ID 로 찍는 법(`vtd/record_window.sh`)과 영상·CSV 시각 맞추기는 [img/README](img/README.md).

## 5.8 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| VTD 가 안 뜬다 | 데스크톱 로그인 전 — `who` 에 `(login screen)` 뿐 | **로그인부터**. 이상하면 `who` 부터 본다 |
| 무엇을 해도 시나리오가 안 올라간다(VTD 자기 데모 TrafficDemo 조차) | X 세션 꼬임 — `X1` 과 `X1001` 두 개가 떠 VtGui 가 엉뚱한 쪽에 물림(2026-08-19) | 재부팅 후 깨끗이 로그인. SCP 경로·Apply·프로젝트 파일을 의심하기 전에 `who` |
| 시나리오가 안 올라간다 | 모듈 탭이 초록이 아니다 | ✓ Apply → Init → Start 다시([5.2.2](#522-시나리오-올리기)) |
| 독일 맵이 뜬다 | ✓ Apply 만 눌러 프로젝트 기본(`TrafficDemo.xml`)이 올라감 | `load_scenario.py <시나리오>` |
| `Ctrl+Shift+I` 가 안 먹는다 | Qt 단축키 중복 | 메뉴 Simulation ▸ Init |
| ego 좌표가 `(7429, -3086)` | 시나리오가 안 올라감 | 5.2.2 다시 |
| 차가 엉뚱한 방향으로 직진·리스폰 | 시나리오 ↔ 경로 파일 짝이 틀림(600m 차이) | [5.2.5](#525-시나리오--경로-파일-짝). `drive.sh` 는 30m 검사로 막는다 |
| ego 가 `(0,0)` 에 스폰 / 시나리오 로드 실패 | XML 이 파싱이 안 된다(앞에 로그가 섞였던 사례) | `push_scenarios.sh` 의 파싱 검사, 파일 앞부분 확인 |
| 출발 방향이 180° 반대 | 첫 경유지를 담는 차로가 스폰 방향으로 달릴 수 없다 | ego 좌표에서 계획(`start_from_ego`, [5.5.10](#5510-시나리오-파일-관리)) |
| 화면에 차가 안 보인다 | 수동 로드라 카메라가 안 붙음 | `vtd/set_cams.py`([5.2.7](#527-카메라)) |
| 고친 게 반영이 안 된 결과 | 제어PC 가 옛 `src/` | `git pull`([5.1.2](#512-제어pc-에-코드-올리기--git-pull)), 러너는 매 판 배포 |
| "속도위반 1839프레임" 같은 결과 | `check_run.py` 에 경로·차로계획을 안 줌 | 같이 준다([5.2.6](#526-채점)) |
| 휠체어가 안 뜬다 | `<Object Definition="wheelchair_adult">` 로 넣음 | Character + `wheelchair_idle`([5.5.8](#558-주변교통--취약대상-코스_trv)) |
| 회귀 판끼리 비교가 이상하다 | 회귀 도중 `swap_routes.sh` 로 경로를 바꿈 | 회귀가 끝난 뒤 바꾼다 |
| VTD 가 죽었다 / 재부팅했다 | — | [`ref/vtd_recovery.md`](ref/vtd_recovery.md) |
