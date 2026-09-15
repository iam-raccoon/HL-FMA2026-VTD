# 4장. 검증 체계 — VTD 없이 어디까지 확인하나

[← 문서 목차](README.md) · [README](../README.md)

VTD 는 한 판에 수 분이 걸리고, 같은 판을 두 번 돌려도 주변 교통이 달라진다. 그래서 **바뀐 코드가 무엇을 망가뜨렸는지**는
VTD 로 알아내기 어렵다. 이 스택은 검증을 다섯 층으로 나눠, 아래 층일수록 빠르고 결정적으로 돌게 했다.

| 층 | 무엇을 보나 | 도구 | 규모 | 한 번 도는 시간 |
|---|---|---|---|---|
| ① 유닛테스트 | 함수 하나의 판단(비킬 폭, 지시등 선행, 분류 경계…) | `tests/*.py` | **37파일** | 약 20초 |
| ② 경로 사전검사 | 계획한 경로가 **달리기 전에** 쓸 만한가 | `eval/check_route.py` | 9개 검사 | 수 초 |
| ③ 시나리오 사전검사 | 시나리오의 물체가 도로 위에 있나 | `vtd/check_scenario.py` | — | 수 초 |
| ④ 오프라인 폐루프 회귀 | 제어 스택 전체를 가상 시뮬로 **끝까지** 굴린다 | `eval/run_all.sh` + `eval/mock_vtd.py` | **35판** | 약 1시간 |
| ⑤ 실주행 채점 | VTD 실주행 CSV 를 **대회 문턱 그대로** 채점 | `eval/score_fma.py` | 15개 항목 | 판당 수 분 |

## 4.1 유닛테스트

소켓도 시뮬도 없이 `DrivingStack.step()` 과 행동층 함수를 직접 부른다. 파일 하나가 스스로 도는 스크립트라
pytest 없이 이렇게 돌린다.

```bash
for f in tests/*.py; do python3 "$f" || echo "❌ $f"; done
```

> ⚠️ `pytest tests/` 로 한꺼번에 모으면 안 된다. 일부 파일이 끝에서 `sys.exit()` 를 불러 수집 단계에서 멈춘다.

테스트는 대부분 **실주행에서 한 번 틀린 장면을 그대로 옮긴 것**이다. 예를 들어
`test_nudge.py::test_차로계획_지시등도_30m와_3초를_같이_만족한다` 는 `routes/` 의 모든 경로에서
경로에 박힌 차선변경마다 지시등이 30 m·3초 앞에 켜지는지 본다 — 새 경로를 넣으면 자동으로 함께 검사된다.

## 4.2 경로 사전검사 — `check_route.py`

경로 생성기 버그는 **VTD 를 한 판 돌려야 드러나는데, 그 한 판이 15분 중 몇 분**이다. 그래서 달리기 전에 거른다.

| 검사 | 걸러내는 것 |
|---|---|
| ① 헤딩 반전 | 경로가 60° 넘게 꺾여 되돌아가는 곳 |
| ② 점 간격 | 5 m 넘게 벌어지거나 0.05 m 로 겹친 점 |
| ③ 도로 밖 | 주행 차로 밖으로 나간 점 |
| ④ 역주행 | 진행 방향과 반대 차로를 달리는 점 |
| ⑤ 곡률 | 아이오닉 6 조향 한계(35°)를 넘는 곳 |
| ⑥ 차로 점유 | 어느 차로에 있는지의 순서 — 눈으로 확인할 목록 |
| ⑦ 기준경로 대조 | 검증된 경로와의 편차(진단용) |
| ⑧ 경로에 박힌 차선변경 | 교차로 **안에서** 차선을 바꾸는 곳 |
| ⑨ 노면 화살표 | 좌회전 전용 차로에서 직진하는 식의 **지시 위반** |

`이상 0곳 — 사용 가능` 이 아니면 쓰지 않는다. 대회용 `drive.sh <경로.csv>` 는 이 검사에서 이상이 나오면 주행을 시작하지 않는다.

## 4.3 오프라인 폐루프 회귀 — `run_all.sh`

`eval/mock_vtd.py` 가 VTD 의 9910 포트를 흉내 낸다. 자전거 모델로 ego 를 움직이고, 시나리오 JSON 에 적힌 차·사람·사물을
정해진 대로 굴린다. 제어 스택은 **대회와 똑같이 `main.py` 로** 붙는다 — 코드 경로가 실주행과 같다.

**합격선은 대회 합격선(60점)이 아니라 기준선이다.** 판마다 기대 점수를 `eval/expected_scores.json` 에 적어 두고
**1점이라도 떨어지면 실패**로 본다. 적신호를 무시하게 만든 버그가 −20을 먹고도 80점이라 "합격"으로 보였던 적이 있어서다.

```bash
bash eval/run_all.sh                            # 35판 전부 → "기준선 일치 — 회귀 통과"
bash eval/run_eval.sh eval/scenarios/school_zone.json   # 한 판만
```

### 35판의 구성

| 분류 | 판 수 | 판 |
|---|---:|---|
| 기본 주행·신호 | 5 | `v5_basic` `signal_compliance` `school_zone` `respawn_test` `unprotected_left` |
| 교차로 | 2 | `unprotected_left_oncoming` `passped_junction` |
| 앞차·끼어들기 | 4 | `cutin_lead` `v2_cutin` `v3_leadbrake` `v7_multi` |
| 돌발상황 | 7 | `hz_blocker_leaves` `hz_cyclist` `hz_jaywalk_occluded` `hz_redlight_runner` `hz_traffic` `hz_truck_cutin` `hz_wrongway` |
| 보행자·취약대상 | 9 | `ped_crossing` `jaywalk_v6` `ped_bypass_rear` `ped_bypass_squeeze` `lc_hold_for_vru` `ovt_return_after_vru` `vru_wheelchair_stop` `vru_wheelchair_dart` `vru_bicycle_slow` |
| 장애물·사고현장 | 6 | `static_obstacle` `slalom_2lane` `accident_junction` `accident_junction_legal` `accident_ped_block` `accident_ped_block_queue` |
| 차선변경 | 2 | `lc4_to_lc1_left` `lc4_to_lc1_left_tr` |

34판은 100점이 기준선이고, **`vru_bicycle_slow` 한 판만 85점**이 기준선이다 — 내 차로를 16 km/h 로 가는
자전거를 끝까지 따라가 시간 안에 못 도착하는 **알려진 한계**를 일부러 남겨 둔 판이다([6.4](06_results.md#64-한계)).

판 이름의 **`_v6` · `(코스 H …)` · `(… 재현)`** 은 그 판이 실주행에서 틀린 장면을 좌표까지 옮겨 온 것이라는 뜻이다.
고친 뒤에도 판을 남겨 같은 실수가 돌아오지 않게 한다.

### 한계

- ego 는 자전거 모델이다. **VTD 차량 동역학과 다르다** — 횡추종 오차, 제동 거리는 실주행이 더 크다.
- 주변 교통은 **정해진 대로** 움직인다. VTD 교통처럼 우리 행동에 반응하지 않는다.
- `evaluate.py` 는 대회 채점기보다 **단순하고 일부는 더 엄하다** — 예를 들어 보행자 3 m 안을 1 m/s 넘게 지나면 무조건 감점하지만,
  대회 항목 14는 "무정차 통과"를 본다. 실주행 판정은 ⑤ `score_fma.py` 로 한다.

## 4.4 실주행 채점 — `score_fma.py`

VTD 에서 달린 CSV 한 판을 [1.4 절](01_problem.md#14-채점--구간당-100점-15개-항목)의 15개 항목에 **안내문 문턱 그대로** 적용한다.
흩어진 검사기(`check_lanes` · `check_crosswalks` · `check_run`)를 눈으로 합치면 매번 다르게 세서 하나로 모았다.

```bash
python3 eval/score_fma.py /tmp/run_HL_FMA_NEW_H.csv \
        --route routes/HL_FMA_NEW_H.json --lane routes/HL_FMA_NEW_H_lane.json \
        --xodr map/HL_FMA_VTD_LivingLab.xodr --sections 5
```

`--xodr` 를 주면 항목 3·4·5·8(차선 밟기·중앙선·보도·실선 변경)까지 본다. 녹색 정차(항목 10)는 **정차 사유 문자열만 믿지 않고**,
그 구간에 실제로 그 사유에 맞는 물체가 있었는지까지 확인해야 면책한다 — 주최측 답변([1.5](01_problem.md#15-주최측이-확정해-준-해석))이 그 조건이기 때문이다.

⚠️ 대회 전에는 구간 수를 몰라 4구간으로 가정했다. **대회는 5구간이었다.** 구간이 많을수록 "항목당 구간 1회" 제한이
더 자주 열려 총점이 낮아지므로, 대회 전 기록의 총점은 **판끼리 비교하는 값**이지 대회 점수 예측이 아니다.

## 4.5 제어 연산 시간 — `bench_control.py`

기록된 입력을 되먹여 `step()` 만 잰다. VTD 는 약 20 Hz 로 보내고 예산은 40 ms(25 Hz)로 잡았다.

```bash
python3 tools/bench_control.py --synth HL_FMA_NEW_E      # CSV 없이 합성 입력으로
```

| 기계 | 코스 | 중앙 | p95 | p99 | 여유(p99) |
|---|---|---:|---:|---:|---:|
| AMD Ryzen 7 5800X (개발PC) | E · 5,242 m · 경로점 3,768 | 1.26 ms | 2.16 ms | **2.33 ms** | **17배** |

코스 E 는 경로점이 다른 코스의 2~3배라 가장 무겁다. 판정은 최대값이 아니라 **p99** 로 한다 —
최대값은 그 기계에서 같이 돈 다른 일에 좌우된다.

## 4.6 한 번에 돌리기

```bash
for f in tests/*.py; do python3 "$f" || echo "❌ $f"; done     # ① 약 20초
bash eval/run_all.sh                                             # ④ 약 1시간
```

코드를 고치는 도중에 회귀를 돌리면 판마다 다른 코드로 돈다. 긴 회귀는 **커밋을 끝낸 뒤** 돌린다.
