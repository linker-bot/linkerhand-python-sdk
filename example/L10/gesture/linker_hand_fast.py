#!/usr/bin/env python3
import sys,os,time,argparse
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, "../../.."))
sys.path.append(target_dir)
from LinkerHand.linker_hand_api import LinkerHandApi
'''
手指快速移动
'''
def main():
    parser = argparse.ArgumentParser(description='处理手势参数')
    parser.add_argument('--hand_type', choices=['left', 'right'], required=True, help='指定左手或右手')
    parser.add_argument('--hand_joint', required=True, help='指定LinkerHand型号')
    parser.add_argument('--can', default="can0", help='指定CAN编号')
    args = parser.parse_args()
    print(f"手类型: {args.hand_type}, 关节: {args.hand_joint}")

    hand_joint = args.hand_joint
    hand_type = args.hand_type
    can = args.can
    hand = LinkerHandApi(hand_joint=hand_joint,hand_type=hand_type, can=can)
    while True:
        for i in range(10):
            if i % 2 == 0:
                pose = [255, 128, 255, 255, 255, 255, 128, 128, 128, 128]
            else:
                pose = [80, 80, 80, 80, 80, 80, 80, 80, 80, 80]
            print(f"Pose {i}: {pose}")
            # 在这里添加更新pose的代码，例如发送到硬件设备
            time.sleep(0.1)  # 等待1秒
            hand.finger_move(pose=pose)

if __name__ == "__main__":
    # python3 linker_hand_fast.py --hand_type left --hand_joint L10 --can=can0
    main()