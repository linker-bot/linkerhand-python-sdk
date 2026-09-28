from LinkerHand.linker_hand_api import LinkerHandApi

print("创建左手...")
left = LinkerHandApi(
    hand_type="left",
    hand_joint="O6",
    can="can0",
)
print("left ok")

print("创建右手...")
right = LinkerHandApi(
    hand_type="right",
    hand_joint="O6",
    can="can0",
)
print("right ok")

print("双手实例初始化成功")