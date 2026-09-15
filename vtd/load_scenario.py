#!/usr/bin/env python3
"""VTD PC(OMEN)에서 시나리오 로드 -> Init -> Start -> 카메라(공식 follower view).
사용: python3 load_scenario.py HL_FMA_VTD_LivingLab_v1
제어는 제어PC에서 별도로 main.py 로 붙인다(주소는 자동판별 — src/netcfg.py).
"""
import os, socket, struct, sys, time

# 툴바 Apply(초록 체크) 의 **창 기준** 위치.
#  ⚠️ 211,95 였는데 **툴바에서 100px 아래**(Project Configuration 패널)를 찍고 있었다.
#     모듈이 이미 붙어 있던 판에서는 SCP Init/Start 만으로 돌아가 안 들켰고,
#     스크립트로 VTD 를 재시작한 뒤(모듈 미기동) 비로소 14판 연속 로드 실패로 터졌다.
#  ★재측정 방법(창 id 는 `xdotool search --name 'Virtual Test Drive'` 중 pid 가 VtGui 인 것):
#       xdotool mousemove --window $W 0 0; xdotool getmouselocation   # 창기준 원점의 절대좌표
#       import -window root /tmp/s.png                                # 초록 체크의 절대좌표를 읽는다
#       APPLY = (체크절대x - 원점x, 체크절대y - 원점y)
#     실측 2026-08-17: 원점 (66,69), 체크 중심 (263,114) -> (197,45)
#  ⚠️ 창 **기하정보**(getwindowgeometry X/Y)를 원점으로 쓰지 말 것 — 재부모화된 창이라
#     (80,118) 을 돌려주는데 실제 mousemove --window 원점은 (66,69) 다.
APPLY_DX, APPLY_DY = 197, 45
SCEN = "/home/user/Hexagon/VTD.2025.2/Data/Projects/Current/Scenarios"


def scp(xml, recv=b"TaskControl"):
    d = xml.encode()
    h = struct.pack("<HH64s64sI", 40108, 1, b"cmd", recv, len(d))
    s = socket.create_connection(("127.0.0.1", 48179), timeout=5)
    s.sendall(h + d); s.close()


# 맵(LivingLab) 좌표 범위. ego 가 여기 밖이면 **우리 시나리오가 안 올라간 것**이다.
MAP_X = (-200.0, 1800.0)
MAP_Y = (-1400.0, 1400.0)


def ego_xy():
    """9910 에서 한 프레임 읽어 ego (x,y). 못 받으면 None."""
    try:
        s = socket.create_connection(("127.0.0.1", 9910), timeout=4); s.settimeout(3)
        b = b""
        while len(b) < 1109:
            d = s.recv(1109 - len(b))
            if not d:
                break
            b += d
        s.close()
        if len(b) != 1109:
            return None
        return struct.unpack("<2f", b[:8])
    except Exception:
        return None


def why_fail():
    """스트리밍 실패의 **원인**을 한 줄로. 오늘 이걸 몰라 한참 헤맸다(2026-08-20).

    · 9910 무응답 = moduleManager 가 안 붙었다 -> **Apply 가 안 먹은 것**(화면 조작 문제)
    · ego 가 맵 밖  = 모듈은 붙었는데 **시나리오가 안 올라간 것**
    """
    return "9910 무응답(Apply 실패 의심)" if ego_xy() is None else "ego 가 맵 밖(시나리오 미로드)"


def streaming_ok():
    """시나리오가 **정말로** 올라갔는가.

    ⚠️⚠️ 예전엔 '9910 에서 1109바이트가 오는가'만 봤다. 그건 거짓 성공을 낸다 —
       **moduleManager 는 시나리오가 없어도 스트리밍한다.** 실측 2026-08-19: 아무것도
       안 올라간 상태에서 `streaming=OK` 를 내고, ego 가 **(7429.7, -3086.9)** —
       우리 맵 밖의 기본 위치에 있었다. 로드 실패가 성공으로 새면 그 뒤 진단이 전부
       헛돈다(그날 D·E·G 실패 원인을 찾는 데 몇 시간을 썼다).
    -> ego 가 **맵 안**에 있는지까지 본다. 이게 최소 조건이다."""
    p = ego_xy()
    if p is None:
        return False
    x, y = p
    inside = MAP_X[0] <= x <= MAP_X[1] and MAP_Y[0] <= y <= MAP_Y[1]
    if not inside:
        print(f"  ⚠️ ego 가 맵 밖이다 ({x:.1f},{y:.1f}) — 시나리오가 안 올라갔다")
    return inside


def dismiss_dialog(env):
    """VtGui 의 모달 안내창을 닫는다. **열려 있으면 이후 조작이 전부 막힌다.**

    실측 2026-08-17: Apply 직후 Init 을 누르면 "Error: set scenario file before
    starting the simulation!" 모달이 뜨는데, 이게 떠 있는 동안 재시도가 아무 일도
    하지 않는다. 로그에는 그냥 `streaming=FAIL` 로만 남아 원인이 안 보였다.
    """
    import subprocess
    try:
        wins = subprocess.check_output(
            ["xdotool", "search", "--name", "^Information$"],
            env=env, text=True).split()
    except Exception:
        return False
    for w in wins:
        subprocess.run(["xdotool", "windowactivate", w], env=env)
        time.sleep(0.5)
        subprocess.run(["xdotool", "key", "--window", w, "Return"], env=env)
        time.sleep(0.5)
        print("  안내 모달을 닫았다")
    return bool(wins)


def gui_init_start(scen_name=None):
    """VTD GUI 에 Init/Start 단축키를 눌러준다. SCP Init/Start 는 stage 를 못 올린다."""
    import subprocess
    env = dict(os.environ, DISPLAY=":1", XAUTHORITY="/home/user/.Xauthority")
    # ⚠️ **가장 최근에 뜬 VtGui 의 창**을 골라야 한다.
    #    VTD 를 재시작해도 옛 세션의 VtGui 가 살아남는 경우가 있다(실측 2026-08-17:
    #    pid 3641(옛) 과 pid 210746(현재) 이 동시에 떠 있었다). `xdotool search` 는
    #    옛 창을 **먼저** 돌려주고, 거기 단축키를 보내면 아무 일도 안 일어난다
    #    -> `streaming=FAIL`. 게다가 프로젝트가 적용되지 않아 TaskControl 이 엉뚱한 맵
    #    (Germany_2018)을 물고 있게 된다.
    #    '살아있는 pid' 만으로는 못 거른다 — 옛 것도 살아있기 때문이다. **시작 시각**으로 고른다.
    #    ⚠️ 프로세스 **이름 매칭은 쓰지 않는다** — 실제 comm 은 `Vtgui`(소문자 g)라
    #       `pgrep -f VtGui` 가 옛 인스턴스를 놓쳤다(실측 2026-08-17). 창 pid 의
    #       **시작 시각**으로 고른다. 이름이 뭐든 상관없다.
    try:
        cands = subprocess.check_output(
            ["xdotool", "search", "--name", "Virtual Test Drive - Version"],
            env=env, text=True).split()
    except Exception:
        return False
    best = None
    for cand in cands:
        try:
            pid = subprocess.check_output(["xdotool", "getwindowpid", cand],
                                          env=env, text=True).strip()
            # etimes = 실행 경과초. 작을수록 최근에 뜬 것.
            age = int(subprocess.check_output(["ps", "-o", "etimes=", "-p", pid],
                                              text=True).strip())
        except Exception:
            continue
        if best is None or age < best[0]:
            best = (age, cand, pid)
    if best is None:
        print("  ⚠️ VtGui 창을 못 찾았다")
        return False
    _age, w, pid = best
    print(f"  VtGui 창 {w} (pid {pid}, {_age}s 전 기동) 사용")

    # ★순서가 중요하다(실측 2026-08-15): Apply -> Init -> Start.
    #   Apply(툴바 초록 체크)가 프로젝트를 초기화하고 ModuleManager 를 올린다.
    #   이걸 빼고 Init/Start 만 누르면 9910 은 열려도 시뮬 클럭이 안 돈다(ego 제자리).
    #    ⚠️ 클릭은 **창 기준 상대좌표**로 한다. 절대 화면좌표(269,115)를 쓰면 창이
    #       움직이는 순간 빗나간다(실측: 옛 창 58,20 / 새 창 86,118 — 28×98 px 어긋남).
    subprocess.run(["xdotool", "windowactivate", w], env=env)
    time.sleep(1)
    subprocess.run(["xdotool", "mousemove", "--window", w,
                    str(APPLY_DX), str(APPLY_DY)], env=env)
    # ★어디를 찍었는지 **절대좌표로 남긴다**. 이 한 줄이 없어서 '툴바 100px 아래를
    #   찍고 있다'는 걸 못 보고 14판을 날렸다. 툴바는 y≈114, 체크는 x≈263 근처다.
    try:
        loc = subprocess.check_output(["xdotool", "getmouselocation"],
                                      env=env, text=True).strip()
        print(f"  Apply 클릭 위치: {loc}")
    except Exception:
        pass
    subprocess.run(["xdotool", "click", "1"], env=env)
    time.sleep(8)
    dismiss_dialog(env)
    # ★Apply 는 **로드해둔 시나리오를 날린다**. 그래서 다시 깔아줘야 한다.
    #   안 그러면 Init/Start 가 "Error: set scenario file before starting the simulation!"
    #   모달을 띄우고, 그 모달이 이후 재시도를 전부 막는다(실측 2026-08-17: 회귀 14판
    #   연속 로드 실패의 마지막 조각). 주석엔 'Apply -> Init -> Start' 로만 적혀
    #   있었는데 그 사이의 재로드가 빠져 있었다.
    if scen_name:
        scp(f'<SimCtrl><LoadScenario filename="{SCEN}/{scen_name}.xml"/></SimCtrl>')
        time.sleep(11)
    for key in ("ctrl+shift+i", "ctrl+shift+r"):
        subprocess.run(["xdotool", "key", "--window", w, key], env=env)
        time.sleep(5)
    dismiss_dialog(env)
    # ★단축키가 **안 먹는 경우가 있다.** 실측 2026-08-19: VtGui 로그에
    #     QAction::eventFilter: Ambiguous shortcut overload: Ctrl+Shift+I
    #   가 반복됐다 — Qt 는 같은 단축키에 액션이 둘 이상이면 **아무것도 실행하지 않는다.**
    #   그러면 Init 이 통째로 건너뛰어지고 시나리오가 안 물린다.
    #   -> Simulation 메뉴에서 직접 누른다(메뉴 클릭은 먹는 걸 확인했다).
    #   창 기준 좌표(실측 2026-08-19): 메뉴 라벨 (116,15) / Start (133,38) / Init (133,112)
    if not streaming_ok():
        print("  단축키가 안 먹었다 -> Simulation 메뉴로 Init/Start")
        for dy in (112, 38):                       # Init 먼저, 그다음 Start
            subprocess.run(["xdotool", "mousemove", "--window", w, "116", "15"], env=env)
            time.sleep(0.4)
            subprocess.run(["xdotool", "click", "1"], env=env); time.sleep(0.9)
            subprocess.run(["xdotool", "mousemove", "--window", w, "133", str(dy)], env=env)
            time.sleep(0.4)
            subprocess.run(["xdotool", "click", "1"], env=env); time.sleep(5)
        dismiss_dialog(env)
    return True


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "HL_FMA_VTD_LivingLab_v1"
    scp("<SimCtrl><Stop/></SimCtrl>"); time.sleep(1.5)
    scp(f'<SimCtrl><LoadScenario filename="{SCEN}/{name}.xml"/></SimCtrl>')
    time.sleep(11)                       # 큰 맵 로딩 여유
    scp('<SimCtrl><Init mode="operation"/></SimCtrl>'); time.sleep(3.5)
    scp("<SimCtrl><Start/></SimCtrl>"); time.sleep(2.5)
    ok = streaming_ok()
    if not ok:
        scp("<SimCtrl><Start/></SimCtrl>"); time.sleep(2)
        ok = streaming_ok()
    # ★SCP 만으로는 시뮬이 '실행'되지 않는다(실측 2026-08-15): 데이터는 흐르는데
    #   시뮬 클럭이 안 돌아 ego 가 제자리다(주행 0m). ParamServer 가 계속
    #   "No handling START at stage READY" 를 찍는다.
    #   -> GUI 단축키로 Init(Ctrl+Shift+I) + Start(Ctrl+Shift+R) 를 눌러야 실제로 돈다.
    # ⚠️ 한 번에 안 붙는 경우가 흔하다(실측 2026-08-17: 1차 FAIL -> 2차 OK). 창 활성화나
    #    Apply 클릭이 타이밍을 타므로 **여기서 재시도**한다. 호출부마다 재시도를 짜면
    #    빠뜨리는 데가 생긴다.
    for attempt in range(3):
        if ok:
            break
        if attempt:
            print(f"  streaming 재시도 {attempt + 1}")
            scp('<SimCtrl><Init mode="operation"/></SimCtrl>'); time.sleep(2.5)
            scp("<SimCtrl><Start/></SimCtrl>"); time.sleep(2)
        gui_init_start(name)
        ok = streaming_ok()
    # 카메라(공식 follower view: 뒤6·위2·10도)
    try:
        import math
        scp('<Camera name="cam1" showOwner="true"><Frustum near="0.5" far="1500" fovHor="70"/>'
            '<PosRelative player="Ego" dx="-6" dy="0" dz="2"/>'
            f'<ViewRelative dh="0" dp="{math.radians(10)}" dr="0"/><Set/></Camera>', recv=b"any")
    except Exception:
        pass
    if not ok:
        # ★원인을 구분해 찍는다. 이게 없어서 2026-08-20 에 '왜 안 움직이지'를
        #   한참 파야 했다 — 로드는 됐는데 9910 이 안 열린 것이었다.
        print(f"  실패 원인: {why_fail()}")
        print("  ⚠️ 화면 조작(Apply/Init/Start)이 안 먹는 상태다. VTD 를 강제 재시작해도"
              " 안 풀리면 **사람이 화면에서 직접 시나리오를 올려야** 한다"
              "(2026-08-19·20 두 번 다 그렇게 복구했다).")
    print(f"loaded={name} streaming={'OK' if ok else 'FAIL'}")
    # ⚠️ FAIL 도 종료코드 0 으로 내면 호출부가 '로드 성공'으로 알고 주행을 띄운다.
    #    그러면 제어PC 가 9910 에 ConnectionRefused 를 맞고, 러너는 그걸
    #    "주행 CSV 가 없다(기동 실패)" 로만 보고한다 — 진짜 원인(로드 실패)이 가려진다.
    #    실측 2026-08-17: 19판 회귀에서 4판 연속 이렇게 새어나갔다.
    sys.exit(0 if ok else 1)


main()
