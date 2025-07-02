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
手指侧摆
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
    hand.set_speed(speed=[120,60,60,60,60])
    # 手指姿态数据
    poses = [
        [255.0,255.0,255.0,255.0,255.0,255.0,40.0,88.0,80.0,63.0], # 手掌张开
        [255.0,0.0,255.0,255.0,255.0,255.0,40.0,88.0,80.0,63.0], # 拇指侧摆
        [255.0,70.0,255.0,255.0,255.0,255.0,40.0,88.0,80.0,0.0], # *手掌张开
        [255.0,255.0,255.0,255.0,255.0,255.0,40.0,88.0,80.0,255.0], # 拇指旋转
        [255.0,255.0,255.0,255.0,255.0,255.0,40.0,88.0,80.0,63.0], # 手掌张开
        [255.0,255.0,255.0,255.0,255.0,255.0,255.0,88.0,80.0,63.0], # 食指侧摆
        [255.0,255.0,255.0,255.0,255.0,255.0,40.0,88.0,80.0,63.0], # 手掌张开
        [255.0,255.0,255.0,255.0,255.0,255.0,40.0,255.0,80.0,63.0], # 无名指侧摆
        [255.0,255.0,255.0,255.0,255.0,255.0,40.0,88.0,80.0,63.0], # 手掌张开
        [255.0,255.0,255.0,255.0,255.0,255.0,40.0,88.0,255.0,63.0], # 小拇指侧摆
        [255.0,255.0,255.0,255.0,255.0,255.0,40.0,88.0,80.0,63.0], # 手掌张开
    ]
    while True:
        for pose in poses:
            hand.finger_move(pose=pose)
            time.sleep(1)


if __name__ == "__main__":
    # python3 linker_hand_sway.py --hand_type left --hand_joint L10 --can=can0
    main()