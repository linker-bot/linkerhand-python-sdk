#!/usr/bin/env python3
import sys,os,time
import argparse
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, "../../.."))
sys.path.append(target_dir)

from LinkerHand.linker_hand_api import LinkerHandApi
from LinkerHand.utils.load_write_yaml import LoadWriteYaml
from LinkerHand.utils.init_linker_hand import InitLinkerHand
from LinkerHand.utils.color_msg import ColorMsg

class GetForce:
    def __init__(self,hand_joint="L10",hand_type="left"):
        self.hand_joint = hand_joint
        self.hand_type = hand_type
        self.touch_type = -1
        self.hand = LinkerHandApi(hand_joint=self.hand_joint,hand_type=self.hand_type)
        self.get_touch_type()
        self.get_force_data()
    
    def get_touch_type(self):
        t = self.hand.get_touch_type()
        if t == 2:
            ColorMsg(msg="压感类型为矩阵式压感", color='green')
        elif t == -1:
            ColorMsg(msg="没有压力传感器", color='red')
        self.touch_type = t
    def get_force_data(self):
        for i in range(3):
            if self.touch_type != 2 and self.touch_type != -1:
                touch = self.hand.get_force()
            else:
                touch = self.hand.get_matrix_touch()
            time.sleep(1)
            print(touch)

if __name__ == "__main__":
    # python3 get_force.py --hand_joint L10 --hand_type right
    parser = argparse.ArgumentParser(description='GetSpeed Example')
    parser.add_argument('--hand_joint', type=str, default='L10',required=True, help='手指关节类型，默认是L10')
    parser.add_argument('--hand_type', type=str, default='left',required=True, help='手的类型，默认是左手')

    args = parser.parse_args()
    GetForce(hand_joint=args.hand_joint,hand_type=args.hand_type)
