from LinkerHand.linker_hand_api import LinkerHandApi

print("creating left...")

left = LinkerHandApi(
    hand_type="left",
    hand_joint="O6",
    can="can0",
)

print("left ok")