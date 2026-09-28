# from LinkerHand.linker_hand_api import LinkerHandApi

# right = LinkerHandApi(
#     hand_type="right",
#     hand_joint="O6",
#     can="can0",
# )
# print("right instance created")

import time
from LinkerHand.linker_hand_api import LinkerHandApi

right = LinkerHandApi(
    hand_type="right",
    hand_joint="O6",
    can="can0",
)

print("初始化时序列号：", repr(right.serial_number))
time.sleep(0.5)
print("稍后序列号缓存：", repr(right.hand.serial_number))
print("右手状态：", repr(right.get_state()))