#!/usr/bin/env python3
import sys,os,time,argparse
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, "../../.."))
sys.path.append(target_dir)
from LinkerHand.linker_hand_api import LinkerHandApi
from LinkerHand.utils.load_write_yaml import LoadWriteYaml
from LinkerHand.utils.init_linker_hand import InitLinkerHand
from LinkerHand.utils.color_msg import ColorMsg
'''
拇指与其他手指循环对指
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
    # 设置速度
    hand.set_speed(speed=[100,80,80,80,80])
    # 手指姿态数据
    poses = [
        [255.0,70.0,255.0,255.0,255.0,255.0,255.0,255.0,255.0,255.0], # 手掌张开
        [135.0,128.0,146.0,255.0,255.0,255.0,255.0,255.0,255.0,80.0], # 拇指对食指
        [255.0,70.0,255.0,255.0,255.0,255.0,255.0,255.0,255.0,255.0], # 手掌张开
        [135.0,88.0,255.0,138.0,255.0,255.0,255.0,255.0,255.0,65.0], # 拇指对中指
        [255.0,70.0,255.0,255.0,255.0,255.0,255.0,255.0,255.0,255.0], # 手掌张开
        [135.0,63.0,255.0,255.0,140.0,255.0,255.0,255.0,255.0,40.0], # 拇指对无名指
        [255.0,70.0,255.0,255.0,255.0,255.0,255.0,255.0,255.0,255.0], # 手掌张开
        [137.0,70.0,255.0,255.0,255.0,131.0,255.0,255.0,120.0,15.0], # 拇指对小拇指
        [255.0,70.0,255.0,255.0,255.0,255.0,255.0,255.0,255.0,255.0], # 手掌张开
    ]
    while True:
        for pose in poses:
            hand.finger_move(pose=pose)
            time.sleep(1.3)


if __name__ == "__main__":
    # python3 linker_hand_opposition.py --hand_type left --hand_joint L10 --can=can0
    main()