# VTD 가 9910 을 안 열 때 — 복구 절차

[← 문서 목차](../README.md) · [README](../../README.md)

재부팅 후나 먹통 후, 제어PC 가 VTD 에서 데이터를 못 받을 때 **급하게 펴 보는 문서**다.
2026-08-15 에 이 원인을 몰라 1시간 넘게 날렸다 — 이 순서대로면 5분에 끝난다.
위에서부터 **증상 표 → 복구 순서 → 그래도 안 될 때 → 원인** 순이다.

## 증상 → 먼저 할 것

| 증상 | 먼저 할 것 |
|---|---|
| `load_scenario.py` 가 `streaming=FAIL` | VTD 가 떠 있나(1단계) → 툴바 초록 체크(2단계) → 다시 로드(4단계) |
| 제어PC 에서 `main.py` 실행하면 `TimeoutError: timed out` (9910 연결은 되는데 데이터가 안 옴, 또는 연결 자체가 안 됨) | OMEN 에서 `ss -ltn \| grep 9910` — 없으면 1~5단계 |
| `ss -ltn \| grep 9910` 에 아무것도 없음 | 프로젝트 초기화 = 툴바 초록 체크(2단계) |
| 연결은 되고 프레임도 오는데 **객체 0개 · ego 안 움직임** | ModuleManager 가 **두 개** 떠 있다 — 3단계 ⚠️, Apply 로 다시 올린다 |
| 9910 은 열리는데 ego 가 제자리(시뮬 클럭이 안 돎) | Apply 없이 Init/Start 만 눌렀다 — **Apply → Init → Start** |
| 다음 실행이 `Could not claim requested feature` 로 죽음 | 라이선스 점유 — [그래도 안 될 때](#그래도-안-될-때) |

> **핵심 한 줄 — 시나리오를 올린 뒤 순서는 Apply(툴바 초록 체크) → Init(Ctrl+Shift+I) → Start(Ctrl+Shift+R).**
> `load_scenario.py` 가 이 셋을 xdotool 로 자동으로 눌러준다. 그래서 시나리오 연속 전환이 그냥 된다
> (검증: v1 → EV_OBSTACLE 전환 후 이동 5.32m). 자세한 이유는 [원인](#원인).

## 복구 순서

모든 단계는 **[OMEN = VTD PC]** 에서 한다. 6단계 확인만 제어PC 에서도 한다.

### 1단계 — VTD 실행 [OMEN]

```bash
~/Hexagon/VTD.2025.2/bin/vtdStart.sh
```

- 셋업은 `00_HL_VTD`, 프로젝트는 `SampleProject` 가 맞다(주최측이 거기에 시나리오를 넣었다).
- SSH로 띄우려면 `DISPLAY=:1 XAUTHORITY=/home/user/.Xauthority` 를 **export** 해야 한다
  (`:0` 아님. `who` 로 확인). 안 그러면 xterm 이 안 떠서 태스크들이 죽는다.

### 2단계 — 프로젝트 초기화 = 툴바 초록 체크 ✅ [OMEN]

VTD GUI 툴바:

```
[새로] [📂] [저장] [다른이름] | [🔧] [✅] [🔄] [▶] [⏸] [⏹] [⏭]
```

**초록 체크(✅)** 를 누른다. 이게 `initProject.sh` 를 돌려 stage 를 CONFIG → READY 로 올리고,
그때 ModuleManager 가 뜬다. 로그로 확인:

```
TaskControl: "initProject.sh: initializing project."
TaskControl: "ModuleManager is ready."
ParamServer: ... at stage READY
```

헷갈리기 쉬운 것:

- `Simulation` 메뉴의 Init/Start 는 이 단계 전에는 **회색(비활성)** 이라 못 누른다.
- `Preparation` 은 시나리오 선택이 아니라 플레이어 수동조작 패널이다.
- 툴바 `📂` 는 "Select **Project**" 창이라 시나리오가 안 보이는 게 정상이다.

원격에서 누르려면:

```bash
export DISPLAY=:1 XAUTHORITY=/home/user/.Xauthority
xdotool mousemove 269 115 click 1      # 툴바 초록 체크
```

### 3단계 — ModuleManager 를 **하나만** 띄운다 [OMEN] · Apply 로도 안 뜰 때만

**최후 수단이다.** 보통은 2단계(Apply)가 ModuleManager 를 올려 주므로 건너뛴다.
VTD 것이 떠 있는데 이걸 돌리면 VTD 것을 죽이고 내 것으로 바꿔서 두 개 충돌 상태를 만들 수 있다.

```bash
cd ~/hlfma2026/vtd && python3 start_modulemanager.py
```

`ModuleManager OK — 9910 listening` 이 나와야 한다.

> ⚠️ **두 개 띄우면 최악의 증상**이 나온다: 먼저 뜬 쪽이 9910 을 잡는데 시뮬 데이터는
> 다른 쪽에 있어서, **연결은 되고 프레임도 오는데 객체 0개 · ego 안 움직임**이 된다.
> 원인 찾기가 매우 어렵다. 스크립트가 알아서 기존 것을 죽이고 하나만 띄운다.
> (VTD 는 Apply/Init 시점에 자기 것을 띄우려다 포트 충돌로 실패한다)

### 4단계 — 시나리오 로드 [OMEN]

```bash
cd ~/hlfma2026/vtd && python3 load_scenario.py HL_FMA_VTD_LivingLab_v1
```

### 5단계 — Init → Start (GUI 단축키) [OMEN]

`load_scenario.py` 가 자동으로 누르지만, 손으로 할 때:

```bash
W=$(xdotool search --name 'Virtual Test Drive - Version' | head -1)
xdotool key --window $W ctrl+shift+i    # Init
xdotool key --window $W ctrl+shift+r    # Start
```

### 6단계 — 확인 [OMEN · 제어PC]

```bash
ss -ltn | grep 9910                      # OMEN
```

제어PC에서 — **프레임 수만 보면 안 된다.** 3단계의 '두 개' 증상이면 프레임은 정상인데
ego 가 안 움직인다. 반드시 **가속을 넣어 실제로 움직이는지**까지 확인할 것.
(아래 주소 `192.168.100.1` 은 이 문서를 쓸 때의 배선이다. 지금 직결 배선에서 VTD PC 는 `192.168.50.11` 이니 맞춰 바꾼다.)

```bash
python3 -c "
import socket,struct,time
s=socket.create_connection(('192.168.100.1',9910),timeout=5)
b=b''; xs=[]; nobj=[]; t0=time.time()
while time.time()-t0<4:
    b+=s.recv(8192)
    while len(b)>=1109:
        f,b=b[:1109],b[1109:]; xs.append(struct.unpack('<f',f[:4])[0])
        ids=[struct.unpack('<I',f[24+36*k:28+36*k])[0] for k in range(30)]
        nobj.append(sum(1 for i in ids if i))
        s.sendall(struct.pack('<ffB',0.0,2.0,0))   # 가속 +2
s.close(); print(f'{len(xs)}프레임 이동 {abs(xs[-1]-xs[0]):.2f}m 객체 {max(nobj)}개')"
```

정상이면 4초에 **80프레임 / 이동 9m 이상 / 객체 1개 이상**.

## 그래도 안 될 때

### 라이선스 점유 — `Could not claim requested feature`

`vtdStop.sh` 가 `simServer` 를 안 죽여서 다음 실행이 `Could not claim requested feature` 로 죽는다.

```bash
pkill -f 'Hexagon/VTD.2025.2/Runtime/Core/SimServer'; pkill -f startSlave.sh
```

⚠️ 라이선스 데몬(`lmgrd`)은 죽이지 말 것 — 살아 있어야 한다.

### ModuleManager 를 손으로 띄우기 (최후 수단)

환경변수를 직접 맞추지 말고 **돌고 있는 taskControl 의 환경을 그대로 복사**한다.
LD_LIBRARY_PATH·LM_LICENSE_FILE·VI_FILE_SUB_PATH 를 하나라도 빠뜨리면 플러그인 검색에 실패한다.

```python
import os, subprocess
pid = subprocess.check_output(['pgrep','-x','taskControl']).decode().split()[0]
env = dict(kv.decode().split('=',1) for kv in open(f'/proc/{pid}/environ','rb').read().split(b'\0') if b'=' in kv)
core, setup = env['VI_CORE_DIR'], env['VI_CURRENT_SETUP']
subprocess.Popen([core+'/ModuleManager/moduleManager','-f',setup+'/Config/ModuleManager/moduleManager.xml'],
                 env=env, cwd=setup+'/Bin', start_new_session=True)
```

## 원인

### ModuleManager 는 자동 실행 대상이 아니다 (실측으로 확인)

`Data/Setups/00_HL_VTD/Config/SimServer/simServer.xml` 에서

```xml
<Process name="ModuleManager" auto="false" explicitLoad="false" ... />
```

**ModuleManager 가 자동 실행 대상이 아니다.** 9910을 여는 건 이 프로세스이고(HLVTD 플러그인을 얹는다),
프로젝트가 초기화되어야 VTD가 알아서 띄운다. 프로젝트 초기화 전에는 아무리 SCP로
`LoadScenario`/`Init`을 보내도 **stage CONFIG 에서 씹힌다**(ParamServer 로그에
`No handling ... at stage CONFIG` 만 찍힘).

### 시나리오를 바꿀 때마다 죽던 이유 — Apply 를 빠뜨렸다 (해결됨)

로드 후 **Apply(툴바 초록 체크)** 를 안 눌렀다.

- `load_scenario.py` 가 맨 처음 보내는 `<SimCtrl><Stop/>` 이 ModuleManager 를 내린다
  (persistent 가 아니다).
- **Apply 가 프로젝트를 다시 초기화하면서 ModuleManager 를 올린다.** VTD 가 알아서 띄우므로
  `start_modulemanager.py` 를 따로 돌릴 필요가 없다 — 오히려 그걸 돌리면 VTD 것을 죽이고
  내 것으로 바꿔서 두 개 충돌 상태를 만든다(그게 '객체 0개 · ego 정지'의 원인이었다).
- Apply 없이 Init/Start 만 누르면 **9910 은 열리는데 시뮬 클럭이 안 돈다**(ego 제자리).

**로드 후 순서: Apply → Init(Ctrl+Shift+I) → Start(Ctrl+Shift+R)** — `load_scenario.py` 가 자동으로 누른다.

## 알아 두면 좋은 것

- 화면을 원격에서 보려면: `DISPLAY=:1 XAUTHORITY=... import -window root /tmp/s.png` 후 scp.
  GUI 조작도 `xdotool` 로 된다(둘 다 OMEN에 설치돼 있음).
- HLVTD 플러그인이 콘솔에 ego 상태(Lane ID/Off/W, Road ID, Indicator)를 찍는다.
  **차로 번호를 직접 확인할 수 있는 유일한 창구**라 디버깅에 쓸 만하다.
