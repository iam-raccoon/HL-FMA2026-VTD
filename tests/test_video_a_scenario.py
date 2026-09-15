"""A조 영상 고정 재현판의 액터 구성과 난수 제거를 회귀 검사한다."""
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCENARIO = os.path.join(ROOT, "scenarios", "HL_FMA_2026_VIDEO_A_EXACT.xml")


def test_video_a_exact_actor_manifest():
    root = ET.parse(SCENARIO).getroot()

    players = {}
    for player in root.iter("Player"):
        desc = player.find("Description")
        players[desc.get("Name")] = desc.get("Type")
    assert set(players) == {
        "Ego", "StartCross1", "StartCross2", "StartCross3", "VideoLead",
        "MidCross1", "MidCross2", "HillCar", "HillBus",
    }
    assert players["Ego"] == "HyundaiIoniq6_23_Black"
    assert players["HillBus"] == "MercedesTravego_10_HoneyYellow"

    objects = [(o.get("Name"), o.get("Definition")) for o in root.iter("Object")]
    assert sum(definition == "WheelBarrow01" for _name, definition in objects) == 3
    assert {
        name for name, definition in objects if definition == "RdMiscPylon03-32cm"
    } == {"FinalCone1", "FinalCone2"}

    characters = {c.get("Name") for c in root.iter("Character")}
    assert characters == {"SchoolPed1", "SchoolPed2", "SchoolPed3", "FinalPed"}
    assert not list(root.iter("PulkDef")), "_EXACT 판에 난수 Pulk 교통이 다시 들어갔다"


def test_video_a_exact_actions_are_connected():
    root = ET.parse(SCENARIO).getroot()
    player_names = {p.find("Description").get("Name") for p in root.iter("Player")}
    object_names = {o.get("Name") for o in root.iter("Object")}
    character_names = {c.get("Name") for c in root.iter("Character")}

    player_actions = {a.get("Player") for a in root.iter("PlayerActions")}
    object_actions = {a.get("Object") for a in root.iter("ObjectActions")}
    character_actions = {a.get("Character") for a in root.iter("CharacterActions")}
    assert player_actions == player_names - {"Ego"}
    assert object_actions == object_names
    assert character_actions == character_names


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_"):
            continue
        try:
            fn()
            print(f"  ✅ {name}")
        except AssertionError as exc:
            fails += 1
            print(f"  ❌ {name}: {exc}")
    print("실패 없음" if not fails else f"{fails}건 실패")
    sys.exit(1 if fails else 0)
