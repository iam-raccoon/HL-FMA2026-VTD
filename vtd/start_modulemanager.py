#!/usr/bin/env python3
"""VTD 의 ModuleManager 를 띄운다 (9910 을 여는 프로세스).

⚠️ 이 셋업(00_HL_VTD)의 simServer.xml 은 ModuleManager 를 auto="false" 로 두고 있어
   VTD 가 자동으로 띄우지 않는다. 9910 이 안 열리면 대개 이것 때문이다.
⚠️ 환경변수를 손으로 맞추지 말 것 — LD_LIBRARY_PATH / LM_LICENSE_FILE / VI_FILE_SUB_PATH
   중 하나만 빠져도 라이선스나 플러그인 검색에서 죽는다.
   **돌고 있는 taskControl 의 환경을 통째로 복사**하는 게 유일하게 확실한 방법이다.
⚠️ 두 개 띄우면 먼저 뜬 쪽이 9910 을 잡고 시뮬 데이터는 다른 쪽에 있어서,
   '연결은 되는데 객체 0개, ego 안 움직임' 이 된다. 반드시 기존 것을 죽이고 하나만.

사용(OMEN에서): python3 start_modulemanager.py
"""
import os
import subprocess
import sys
import time


def main():
    subprocess.run(["pkill", "-x", "moduleManager"])
    time.sleep(2)
    try:
        pid = subprocess.check_output(["pgrep", "-x", "taskControl"]).decode().split()[0]
    except subprocess.CalledProcessError:
        sys.exit("taskControl 이 안 돌고 있다 — VTD 부터 실행할 것")

    env = {}
    for kv in open(f"/proc/{pid}/environ", "rb").read().split(b"\0"):
        if b"=" in kv:
            k, v = kv.decode("utf-8", "replace").split("=", 1)
            env[k] = v

    core = env["VI_CORE_DIR"]
    setup = env["VI_CURRENT_SETUP"]
    log = open("/tmp/moduleManager.log", "wb")
    subprocess.Popen([core + "/ModuleManager/moduleManager",
                      "-f", setup + "/Config/ModuleManager/moduleManager.xml"],
                     env=env, cwd=setup + "/Bin", stdout=log, stderr=log,
                     start_new_session=True)
    for _ in range(20):
        time.sleep(1)
        out = subprocess.run(["ss", "-ltn"], capture_output=True, text=True).stdout
        if ":9910" in out:
            print("ModuleManager OK — 9910 listening")
            return
    print("FAIL — /tmp/moduleManager.log 확인")
    sys.exit(1)


main()
