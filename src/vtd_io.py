"""
HL-FMA 2026 aMAP Stride — VTD 데이터 인터페이스 (포트 9910 / 9912 / 8554)

2026-08-13 libHLVTD.so 실측 프로토콜 기반.
  9910 TCP  : DataPacket(VTD->나, 1109B) + CtrlPacket(나->VTD, 9B). headerless, little-endian.
  9912 UDP  : 라이다 원시 점군 push (VTD->나). 42B 헤더 + float32 xyz*N.
  8554 RTSP : 전방 카메라 pull (차선용).

⚠️ 인지는 9910 DataPacket의 objects[30] GT로 충분(라이다 클러스터링/YOLO 불필요).
⚠️ DataPacket 세부 레이아웃(특히 신호등 배열 크기)은 org 기본 API 모듈 오면 1:1 대조/교체할 것.
   현재는 크기(1109B)와 인터페이스 API.xlsx 필드 순서로 역산한 값.
"""
import socket
import struct
import math
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

# ---------------- 9910 프로토콜 ----------------
# DataPacket(1109B) = ego(6f=24) + objects[30](각 36B=1080) + trafficLight(iB=5)
_EGO_FMT = "ffffff"                 # egoX, egoY, egoZ (m), egoHeading, egoPitch, egoRoll (rad)
_OBJ_FMT = "Iffffffff"             # id(u32) + x,y,z,heading,speed,length,width,height (f32)
_TL_FMT  = "iB"                    # trafficLight id(i32) + state(u8)
N_OBJ = 30
DATA_FMT = "<" + _EGO_FMT + _OBJ_FMT * N_OBJ + _TL_FMT
DATA_SIZE = struct.calcsize(DATA_FMT)          # == 1109

# CtrlPacket(9B) = steering(rad,f32) + targetAccel(m/s^2,f32) + turnSignal(u8)
CTRL_FMT = "<ffB"
CTRL_SIZE = struct.calcsize(CTRL_FMT)          # == 9

# 신호등 상태 (인터페이스 API.xlsx)
TL_UNSET, TL_RED, TL_YELLOW, TL_GREEN, TL_LEFT, TL_GREEN_LEFT, TL_FLASH = range(7)
# 방향지시등
TS_OFF, TS_LEFT, TS_RIGHT = 0, 1, 2
# ★★**비상등은 없다.** 프로토콜의 turnSignal 은 0/1/2 만 받고 나머지는 전부 소등이다.
#   추측이 아니라 플러그인 디스어셈블로 확인했다(2026-08-28, libHLVTD.so.1.0.0):
#     ControlApplier::mapTurnSignalFlags(unsigned char):
#         mov eax,1 ; cmp dil,1 ; je ret          <- ts==1 -> 1 (좌)
#         xor eax,eax ; cmp dil,2 ; sete al ; add eax,eax   <- ts==2 -> 2 (우), 그 외 0
#   실측으로도 3 을 82프레임 보냈지만 화면에 아무것도 안 켜졌다.
#   -> 정차 중 표시는 **우측 지시등 유지**가 최선이다. 좌우를 번갈아 켜서 흉내내는 건
#      가짜 진로변경 신호로 보이므로 하지 않는다.


# ---------------- 객체 분류 (취약대상·차량·사물·노면) ----------------
# ★★**판정은 여기 하나뿐이다.** 예전엔 behavior·overtake·drive에 같은 식이
#   복사돼 있었고, 주석에 "한쪽만 바뀌면 모순" 이라고 적어두고도 실제로 어긋났다.
#
#   근거 = VTD 2025.2 모델 카탈로그 실측(2026-09-02, Players/{Pedestrians,Objects,Vehicles}
#   222개 전수):
#       male_adult      0.60 x 0.70 x 1.80      female_adult   0.55 x 0.63 x 1.625
#       male_child      0.50 x 0.60 x 1.35      female_child   0.50 x 0.60 x 1.425
#       DummyPerson     0.234 x 0.397 x 1.401   (우리 시나리오가 쓰는 것)
#       실측 로그의 사람 2.0 x 0.6 x 1.7        (자전거 탄 사람으로 보인다)
#       wheelchair_adult 1.01 x 0.62 x 0.92  ★  bicycle_adult  1.90 x 0.65 x 1.10  ★
#       라바콘 0.30x0.30x0.32   기름통 0.15x0.46x0.61   (사람이면 안 되는 것)
#       표지판 0.04x0.60x2.10   기둥 0.05x0.05x2.50     (사람이면 안 되는 것)
PERSON_MIN_H   = 1.2    # 서 있는 사람의 최소 키(어린이 1.35 도 넉넉히 든다)
PERSON_MAX_W   = 1.0    # 이륜차 폭 상한
PERSON_MAX_L   = 3.0    # 이륜차 길이 상한
# ★취약 도로이용자(휠체어 0.92 · 자전거 1.10)는 키가 1.2 에 못 미친다. 법으로 휠체어는
#   보행자다[도교법 제2조 17호 나목]. 문턱을 0.85 로 내려도 라바콘(0.32)·기름통(0.61)은
#   한참 아래라 2026-08-14 의 '콘을 사람으로 보고 영구정지' 는 재발하지 않는다.
VRU_MIN_H      = 0.85
VRU_MAX_W      = 0.70   # 휠체어 0.62 · 자전거 0.65 는 들고, 손수레(2.06)·공사가드(2.30)는 뺀다
VRU_MAX_L      = 2.30
# ★★**길가 기둥은 사람이 아니다.** 표지판(L=0.04)·폴(0.05)·삽(0.06)이 `W<=1.0 · L<=3.0
#   · H>=1.2` 를 통과해 사람으로 잡혔다 — 갓길 3~4m 짜리 표지판마다 18km/h 로 깎였다
#   (2026-09-02 실측: PED_STILL_ASIDE). 사람은 아무리 말라도 **앞뒤 0.23m**(DummyPerson)다.
PERSON_MIN_L   = 0.20


class ObjectClass(str, Enum):
    """9910 치수로 추론한 **주행 정책용** 객체 분류.

    패킷에 VTD 모델 타입이 없으므로 카탈로그의 원래 폴더 분류가 아니라, 우리 차가
    어떻게 대응해야 하는지를 나타낸다. 예를 들어 오토바이와 세그웨이는 VTD 에서는
    Vehicle 이지만 사람과 같은 측면 여유가 필요한 취약대상(VRU)으로 둔다.
    """

    VRU = "vru"
    VEHICLE = "vehicle"
    OBSTACLE = "obstacle"
    ROAD_SURFACE = "road_surface"


# 기본 치수식만으로는 아래 사물들이 사람/이륜차 범위와 겹친다. 9910에는 모델명이
# 없으므로 VTD 2025.2 카탈로그의 원본 치수 시그니처로 제외한다. float32 왕복 오차만
# 허용하고, 임의 스케일 모델까지 같은 물체라고 추측하지 않는다.
_DIM_TOL = 0.015
_KNOWN_NON_VRU_DIMS = (
    (1.18, 0.06, 1.09),   # MiscPicketFence01
    (0.36, 0.24, 1.22),   # MiscBoomgateCtrlBox
    (0.63, 0.58, 1.27),   # RubbishBin
    (1.98, 0.82, 1.41),   # LawnMower01
    (0.33, 0.52, 1.58),   # Rake01
    (0.20, 0.52, 1.67),   # PushBoom01
    (0.67, 0.10, 2.88),   # Ladder01
)
_KNOWN_VRU_DIMS = (
    (2.20, 1.43, 1.75),   # moose — 넓어서 기본 이륜차 폭식에서는 빠지는 동물
)


def _matches_dims(length, width, height, signatures):
    return any(abs(length - l) <= _DIM_TOL
               and abs(width - w) <= _DIM_TOL
               and abs(height - h) <= _DIM_TOL
               for l, w, h in signatures)


VRU_MAX_SPEED = 40.0 / 3.6   # 이보다 빠르면 사람·자전거일 수 없다 -> 차량(이륜차)로 본다[m/s]


def is_vru(length, width, height, speed=None):
    """사람·어린이·휠체어·이륜차·동물 등 보호할 취약대상인가.

    치수는 **원본**(패킷 그대로)을 넣어야 한다 — 경로축 외접상자는 비스듬한 자전거의
    폭을 0.6 -> 1.8m 로 부풀려 승용차와 구분이 안 된다.

    ★`speed` 를 주면 **속도로도 가른다**(2026-09-05). VTD 의 오토바이는 보행자와
      치수가 같다(2.0x0.6x1.7). 치수만 보면 62km/h 로 달리는 이륜차를 사람으로
      봐서 `YIELD_PED`(앞범퍼 5m 정지)가 걸린다. 실측 코스 A t=465~478 (1284,-165):
      이륜차가 끼어들며 62 -> 2 -> 50km/h 로 흔들리자 우리도
      FOLLOW(a=-5) -> YIELD_PED -> NARROW_BLOCK+N -> CROSSWALK_STOP -> YIELD_PED,
      38 -> 25 -> 37 -> 12 -> 20 -> 7 -> 1 -> 16km/h 로 12초 요동. 급제동 2회.
      behavior.py 에 "보행자는 60km/h 로 못 간다" 고 적혀 있었지만 게이트가 없었다.
      40km/h 면 자전거도 안 나온다. 느려진 뒤에도 차량으로 남기는 건 호출부가
      id 로 기억한다(drive.py `_fast_ids`).
    """
    if speed is not None and speed > VRU_MAX_SPEED:
        return False
    if _matches_dims(length, width, height, _KNOWN_NON_VRU_DIMS):
        return False
    if _matches_dims(length, width, height, _KNOWN_VRU_DIMS):
        return True
    if length < PERSON_MIN_L:
        return False                                    # 표지판·기둥·삽
    if height >= PERSON_MIN_H:
        return (max(length, width) < 1.2                # 서 있는 사람
                or (width <= PERSON_MAX_W and length <= PERSON_MAX_L))   # 이륜차·탄 사람
    if height >= VRU_MIN_H:
        return width <= VRU_MAX_W and length <= VRU_MAX_L                # 휠체어·자전거
    return False


def classify_object(length, width, height):
    """VTD 타입 필드가 없는 9910 객체를 주행 정책 네 종류로 분류한다."""
    if is_vru(length, width, height):
        return ObjectClass.VRU
    if height < 0.25:
        return ObjectClass.ROAD_SURFACE
    # 승용차 중 가장 작은 smart_fortwo(2.51 x 1.48)보다 약간 낮은 경계다.
    # 큰 고정 구조물이 VEHICLE 로 묶여도 둘 다 차체 충돌 대상으로 처리하므로 안전 방향이다.
    if length >= 2.30 and width >= 1.40:
        return ObjectClass.VEHICLE
    return ObjectClass.OBSTACLE


def is_person(length, width, height):
    """구 호출 호환용. 새 코드는 의미가 정확한 :func:`is_vru`를 사용한다."""
    return is_vru(length, width, height)


@dataclass
class Obj:
    id: int
    x: float; y: float; z: float
    heading: float
    speed: float
    length: float; width: float; height: float


@dataclass
class State:
    x: float = 0.0; y: float = 0.0; z: float = 0.0
    heading: float = 0.0; pitch: float = 0.0; roll: float = 0.0
    speed: float = 0.0                          # 패킷에 없음 -> 위치 미분 + 평활로 추정
    speed_raw: float = 0.0                      # 평활 전 원시 미분값(진단용)
    objects: List[Obj] = field(default_factory=list)
    tl_id: int = -1; tl_state: int = TL_UNSET
    t: float = 0.0
    respawned: bool = False                      # 경로이탈 리스폰(좌표 순간이동) 감지 플래그
    sim_scale: float = 1.0                       # sim 시간 / 벽시계 시간(주변차로 잰 값)
    sim_clock: bool = False                      # 그 값을 믿을 만한가(시계로 쓸 차가 있었나)


def parse_data(buf: bytes) -> State:
    """1109B DataPacket -> State (id==0 객체는 빈 슬롯이라 제외)."""
    v = struct.unpack(DATA_FMT, buf)
    s = State()
    s.x, s.y, s.z, s.heading, s.pitch, s.roll = v[0:6]
    i = 6
    objs = []
    for _ in range(N_OBJ):
        oid, ox, oy, oz, oh, osp, ol, ow, ohh = v[i:i + 9]; i += 9
        if oid != 0:
            objs.append(Obj(oid, ox, oy, oz, oh, osp, ol, ow, ohh))
    s.objects = objs
    s.tl_id, s.tl_state = v[i], v[i + 1]
    return s


def build_ctrl(steering: float, accel: float, turn_signal: int = TS_OFF) -> bytes:
    """9B CtrlPacket. 비유한값은 플러그인이 무시하므로 0으로 치환."""
    if not (math.isfinite(steering) and math.isfinite(accel)):
        steering, accel = 0.0, 0.0
    return struct.pack(CTRL_FMT, float(steering), float(accel), int(turn_signal) & 0xFF)


def pack_data(ex, ey, ez, eh, ep, er, objects: List[Obj],
              tl_id: int = -1, tl_state: int = TL_UNSET) -> bytes:
    """DataPacket 생성 (mock 서버/유닛테스트용)."""
    vals = [ex, ey, ez, eh, ep, er]
    for k in range(N_OBJ):
        if k < len(objects):
            o = objects[k]
            vals += [o.id, o.x, o.y, o.z, o.heading, o.speed, o.length, o.width, o.height]
        else:
            vals += [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    vals += [tl_id, tl_state]
    return struct.pack(DATA_FMT, *vals)


def parse_ctrl(buf: bytes):
    """9B CtrlPacket -> (steering, accel, turnSignal). mock 서버용."""
    return struct.unpack(CTRL_FMT, buf)


class VTDLink:
    """9910 TCP 클라이언트: 상태 수신 + 제어 송신 + 속도 추정.

    ⚠️ ego 속도는 DataPacket에 없다(객체만 speed 필드가 있음). 위치를 미분해 만든다.

       위치가 GT로 깨끗한데도 순간 미분이 흔들리는 진짜 이유:
       **위치는 sim 클럭으로 전진하는데 시간은 벽시계로 잰다.** 패킷이 뭉쳐 오면
       한 프레임에 sim step 2개분(0.34m)이 들어오는데 벽시계 dt는 0.020s여서
       16.9m/s 로 읽힌다(실측 2026-08-15 v3). 패킷에 타임스탬프가 없어 sim 시각은 못 쓴다.
       -> **WINDOW 창의 이동거리 / 걸린시간**으로 낸다. 창 안에서는 뭉침이 상쇄된다
          (앞에서 몰아 받으면 뒤에서 덜 받으므로 총합은 맞다).
          창을 벗어난 표본이 빠질 때 생기는 계단만 가벼운 EMA로 다듬는다.
       평활은 우리 제어 입력만 다듬는다. 채점 속도는 org가 시뮬 내부 값으로 잰다.
    """

    WINDOW   = 0.30      # s. 이 길이의 창에서 '이동거리 / 걸린시간'으로 속도를 낸다
    TAU      = 0.06      # s. 창을 벗어난 표본이 빠질 때의 계단만 다듬는 가벼운 평활
    JUMP_V   = 40.0      # m/s. 이 이상 = 물리적으로 불가능
    JUMP_D   = 2.5       # m.  동시에 이만큼 튀어야 리스폰으로 인정(느린 주기 오탐 방지)
    # ★rad/s. 이 이상으로 헤딩이 꺾이면 = 리스폰(차량이 낼 수 있는 요레이트가 아님).
    #   실측 2026-08-15: 추월 중 리스폰이 **1.59m 이동 + 헤딩 20° 스냅(250°/s)** 으로 왔다.
    #   거리 조건(2.5m)만 보다가 놓쳤고, 제어기 상태도 초기화되지 않았다.
    #   물리 상한: v/L*tan(δ_max) = 11/2.95*tan30° ≈ 2.15 rad/s -> 3.0이면 오탐 없음.
    JUMP_YAW = 3.0

    # ★★**주변차를 시계로 쓴다** — VTD sim 이 실시간보다 느려질 때를 대비한 계측기.
    #   ego 패킷에는 속도도 sim 시각도 없어 위치를 **벽시계**로 미분한다. VTD 가
    #   느려지면(sim 이 0.04s 가는 동안 벽시계는 0.28s) 그 미분값이 실제의 1/7 이 되고,
    #   제어기는 "느리다"고 믿고 계속 가속한다(친구 실측 E_TR 46 -> 81km/h).
    #   그런데 **객체는 speed 를 GT 로 준다.** 같은 차가 두 프레임에 다 있으면
    #       dt_sim = (그 차가 실제로 움직인 거리) / (그 차가 보고한 속도)
    #   이고, 여러 대의 **중앙값**을 쓰면 가감속·튐에 견딘다. 그러면
    #       scale = dt_sim / dt_wall,   참속도 = 벽시계 미분값 / scale.
    #   ⚠️ 실측 검증(코스 E 실주행 15051프레임, 표본 57069): 중앙 scale **1.004**,
    #      dt=0.04s 에서 1.005 · dt=0.08s 에서 1.001. 즉 **우리 랜에서 VTD sim 은
    #      실시간으로 돈다**(0.08s 간격도 sim 0.08s 어치를 움직인다 — 패킷이 뭉쳤을 뿐
    #      sim 이 느려진 게 아니다). 그래서 이 보정은 평소엔 아무것도 안 한다.
    #      그 실측이 곧 이 계측기의 검증이기도 하다 — 참값 1 을 1.004 로 맞췄다.
    SIM_CLOCK_V   = 4.0      # 이보다 빠른 차만 시계로 쓴다[m/s] — 정지차는 0/0 이다
    SIM_CLOCK_N   = 2        # 최소 이만큼 있어야 중앙값을 믿는다[대]
    SIM_CLOCK_LO  = 0.05     # scale 하한(20배 느림). 이 밑은 계측 오류로 본다
    SIM_CLOCK_HI  = 1.30     # 상한. sim 이 실시간보다 빠를 일은 없다
    SIM_CLOCK_TAU = 0.20     # scale 평활[s] — 한 프레임 튐으로 속도를 흔들지 않는다
    # ★문턱을 0.85 로 둔 근거(위 실측 3판 38,160프레임 재현): 평활 scale 의
    #   **최솟값이 0.902** 였다(중앙 1.009~1.013, p1 0.970). 0.90 이면 아슬아슬하다.
    #   진짜 sim 지연은 0.14 처럼 극적으로 나타나지 0.88 로 오지 않는다.
    #   (그 재현은 CSV 의 fx/fy 가 0.1m 로 반올림돼 있어 실제보다 잡음이 크다 —
    #    실주행 코드는 원시 float 를 쓰므로 더 조용하다.)
    SIM_CLOCK_USE = 0.85     # scale 이 이보다 작을 때만 **보정한다**(정상 주행 무개입)

    def __init__(self, host=None, port=9910, timeout=3.0):
        if host is None:
            from netcfg import default_host
            host = default_host()
        self.host, self.port, self.timeout = host, port, timeout
        self.sock: Optional[socket.socket] = None
        self._buf = b""
        self._px = self._py = self._pt = self._ph = None
        self._vf = 0.0                      # 평활된 속도(필터 상태)
        # [(t, 직전점부터의 이동거리, 그 사이 흐른 **sim** 시간)] — WINDOW 창
        self._hist = deque()
        self._pobj = {}                     # 직전 프레임 객체 위치 {id: (x, y)}
        self._scale = 1.0                   # 평활된 sim/wall 시간비
        self._scale_ok = False              # 이번 프레임에 실제로 쟀나

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.host, self.port))

    # ★수신이 잠깐 끊겨도 죽지 않는다. VTD 쪽 모듈(ghostdriver 등)이 순간 멈추면
    #   recv 가 타임아웃을 던지는데, 그대로 예외로 나가면 **제어기가 통째로 죽는다**
    #   (실측 2026-08-16 EV_COMBO_CHAIN: 13초 만에 TimeoutError 로 종료, 완주 0점).
    #   대회 중 한 번 끊겼다고 판을 버릴 수는 없으므로 이 시간까지는 계속 기다린다.
    STALE_GIVEUP = 8.0          # s. 이만큼 아무것도 안 오면 그때 링크가 죽은 걸로 본다

    def recv_state(self) -> Optional[State]:
        waited = 0.0
        while len(self._buf) < DATA_SIZE:
            try:
                d = self.sock.recv(8192)
            except socket.timeout:
                waited += self.timeout
                if waited >= self.STALE_GIVEUP:
                    return None                     # 진짜 끊긴 것 — 호출부가 정지시킨다
                continue                            # 잠깐 멈춘 것 — 계속 기다린다
            if not d:
                return None
            self._buf += d
        # ★★백로그 드레인 — 버퍼에 완성 프레임이 **4개 이상** 쌓였으면 최신 것만 쓴다.
        #   VTD 쪽이 잠깐 멎었다 몰아서 보내면(모듈 히컵) 묵은 프레임을 순서대로
        #   소화하는 동안 **과거 위치로 조향**하게 된다. 160ms(4프레임) 넘게 밀렸을
        #   때만 버린다 — 정상 지터(TCP 코얼레싱 1~2프레임)는 건드리지 않는다.
        #   속도 추정은 안전하다: 위치 차분이라 버린 프레임의 이동량도 합산되고,
        #   리스폰 오탐도 없다(dt 가 밀린 시간만큼 같이 커져 JUMP_V 를 못 넘는다).
        if len(self._buf) >= DATA_SIZE * 4:
            n_drop = len(self._buf) // DATA_SIZE - 1
            self._buf = self._buf[n_drop * DATA_SIZE:]
            print(f"[vtd_io] ⚠ 수신 백로그 {n_drop}프레임 폐기(최신만 사용)", flush=True)
        frame = self._buf[:DATA_SIZE]; self._buf = self._buf[DATA_SIZE:]
        s = parse_data(frame)
        now = time.time(); s.t = now
        self._update_speed(s, now)
        return s

    def _sim_scale(self, s: State, dt: float):
        """sim 시간 / 벽시계 시간. 주변차를 시계로 삼는다. 못 재면 ``None``.

        같은 id 가 두 프레임에 다 있는 **빠른** 차만 쓴다. 그 차가 실제로 움직인
        거리를 그 차가 보고한 속도로 나누면 그 사이 흐른 sim 시간이 나온다.
        여러 대의 중앙값을 쓰는 이유는 가감속 중인 차 한 대에 끌려가지 않기 위해서다.
        """
        prev, self._pobj = self._pobj, {o.id: (o.x, o.y) for o in s.objects}
        if not prev or dt <= 1e-6:
            return None
        r = []
        for o in s.objects:
            if o.speed < self.SIM_CLOCK_V:
                continue                            # 정지·저속차는 0/0 이라 시계가 못 된다
            p = prev.get(o.id)
            if p is None:
                continue                            # 이번에 처음 보인 차
            d = math.hypot(o.x - p[0], o.y - p[1])
            if d > o.speed * dt * 2.5:
                continue                            # 순간이동(NPC 리스폰)은 시계가 아니다
            r.append(d / (o.speed * dt))
        if len(r) < self.SIM_CLOCK_N:
            return None
        r.sort()
        m = r[len(r) // 2]
        return m if self.SIM_CLOCK_LO <= m <= self.SIM_CLOCK_HI else None

    def _update_speed(self, s: State, now: float):
        if self._px is None:                        # 첫 프레임: 기준점만 잡는다
            self._px, self._py, self._pt, self._ph = s.x, s.y, now, s.heading
            self._hist.append((now, 0.0, 0.0))      # 창의 시작점(없으면 둘째 프레임이 span=0)
            self._pobj = {o.id: (o.x, o.y) for o in s.objects}
            return

        dt = now - self._pt
        dist = math.hypot(s.x - self._px, s.y - self._py)

        # ★sim 시계는 리스폰 판정보다 **먼저** 갱신한다 — 어느 가지로 빠져나가든
        #   `_pobj` 가 이번 프레임 것으로 남아야 다음 프레임에 짝을 찾는다.
        sc = self._sim_scale(s, dt)
        self._scale_ok = sc is not None
        if sc is not None:
            a_s = 1.0 - math.exp(-dt / self.SIM_CLOCK_TAU)
            self._scale += a_s * (sc - self._scale)
        else:
            self._scale += (1.0 - self._scale) * 0.05   # 못 재면 천천히 1 로 되돌린다
        s.sim_scale, s.sim_clock = self._scale, self._scale_ok

        # 리스폰 감지 — 두 가지 신호를 모두 본다.
        #  ① 순간이동: 거리도 크고 속도도 불가능 (둘 중 하나만이면 오탐)
        #  ② 헤딩 스냅: 좌표는 조금만 움직여도 차가 낼 수 없는 각속도로 방향이 바뀜
        teleport = dist > self.JUMP_D and dist / max(dt, 1e-3) > self.JUMP_V
        dh = (s.heading - self._ph + math.pi) % (2 * math.pi) - math.pi
        snapped = abs(dh) > self.JUMP_YAW * max(dt, 0.02)
        if teleport or snapped:
            s.respawned = True
            self._vf = 0.0                          # 점프로 필터 오염 방지
            self._hist.clear()
            self._px, self._py, self._pt, self._ph = s.x, s.y, now, s.heading
            s.speed = s.speed_raw = 0.0
            return
        self._ph = s.heading
        self._px, self._py, self._pt = s.x, s.y, now

        # ★창에 **sim 시간**도 같이 넣는다. sim 이 실시간이면(평소) dt_sim == dt 라
        #   아래 계산은 예전과 글자 그대로 같은 값을 낸다 — 평소엔 무개입이다.
        dt_sim = dt * self._scale if self._scale < self.SIM_CLOCK_USE else dt
        self._hist.append((now, dist, dt_sim))
        while len(self._hist) > 1 and now - self._hist[0][0] > self.WINDOW:
            self._hist.popleft()
        win = list(self._hist)[1:]
        span = sum(ts for _, _, ts in win)
        if span <= 1e-6:
            s.speed = s.speed_raw = self._vf
            return
        raw = sum(d for _, d, _ in win) / span

        a = 1.0 - math.exp(-dt / self.TAU)          # dt가 흔들려도 응답 동일
        self._vf += a * (raw - self._vf)
        s.speed_raw = raw
        s.speed = self._vf
        return

    def send_ctrl(self, steering, accel, turn_signal=TS_OFF):
        self.sock.sendall(build_ctrl(steering, accel, turn_signal))

    def close(self):
        if self.sock:
            self.sock.close()


if __name__ == "__main__":
    # 자체 점검: 패킷 크기 확인
    assert DATA_SIZE == 1109, f"DataPacket {DATA_SIZE} != 1109 (레이아웃 재확인 필요)"
    assert CTRL_SIZE == 9, f"CtrlPacket {CTRL_SIZE} != 9"
    print(f"OK  DataPacket={DATA_SIZE}B  CtrlPacket={CTRL_SIZE}B")
