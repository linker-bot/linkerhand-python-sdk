#!/usr/bin/env python3
import sys, os, time
import argparse

# 动态加载上层路径以引入API
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, "../.."))
sys.path.append(target_dir)

from LinkerHand.linker_hand_api import LinkerHandApi

"""
L20 灵巧手动态抓取示例
支持根据物体直径动态设置 20 个自由度的抓握姿态
命令行参数：
--hand_joint: 手指关节类型（必须为 L20）
--hand_type : 左手或右手
--speed     : 各关节的速度设置（20个整数，范围 0~255）
--mm        : 物品直径（单位 mm）

示例：
python3 example/Adaptive_L20.py --hand_joint L20 --hand_type right --speed 50 50 50 50 50 40 40 40 40 40 30 0 0 0 0 60 60 60 60 60 --mm 30
"""

def main(args):
    if args.hand_joint != "L20":
        print("当前脚本仅适用于 L20 灵巧手，请设置 --hand_joint L20")
        return

    # 初始化 API
    hand = LinkerHandApi(hand_joint=args.hand_joint, hand_type=args.hand_type,can="can0")
    speed = [200] * 5
    # 设置运动速度
    hand.set_speed(speed=speed)
    time.sleep(1)

    #pose = [250, 250, 250, 250, 250, 250, 128, 128, 128, 128, 250, 0, 0, 0, 0, 250, 250, 250, 250, 250]
    # pose = [60, 70, 25, 25, 25, 25, 25, 255, 255, 88]
    pose = [100] * 20
    hand.finger_move(pose=pose)
    time.sleep(2)
    # 握拳，抓取0mm物
    return
    # 抓取策略：根据物体直径（mm）设置关节角度（i 为调整增量）
    i = args.mm * 2

    # 关节角度设置
    # 根部关节（0-4）
    pose[0] = 60 + i   # 拇指根部
    pose[1] = 25 + i   # 食指根部
    pose[2] = 25 + i   # 中指根部
    pose[3] = 25 + i   # 无名指根部
    pose[4] = 25 + i   # 小指根部

    # 侧摆关节（5-9）保持默认值（可扩展为自适应）
    pose[5] = 60       # 拇指侧摆
    pose[6] = 70       # 食指侧摆
    pose[7] = 70       # 中指侧摆
    pose[8] = 70       # 无名指侧摆
    pose[9] = 70       # 小指侧摆

    # 横摆和预留位（10-14）设为常量或保留值
    pose[10] = 40      # 拇指横摆
    pose[11] = 0       # 预留
    pose[12] = 0       # 预留
    pose[13] = 0       # 预留
    pose[14] = 0       # 预留

    # 指尖末端（15-19）动态抓取弯曲角度
    pose[15] = 60 + i  # 拇指尖部
    pose[16] = 60 + i  # 食指末端
    pose[17] = 60 + i  # 中指末端
    pose[18] = 60 + i  # 无名指末端
    pose[19] = 60 + i  # 小指末端

    # 显示最终姿态
    print(f"[INFO] Grasping pose for object of diameter {args.mm}mm:")
    for idx, val in enumerate(pose):
        print(f"  Joint {idx:02d}: {val}")

    # 执行抓握动作
    hand.finger_move(pose=pose)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="L20 Dynamic Grasping Script")
    parser.add_argument("--hand_joint", type=str, default="L20", help="Hand joint type (must be L20)")
    parser.add_argument("--hand_type", type=str, default="right", help="Hand type (left or right)")
    parser.add_argument("--speed", type=int, nargs=20, default=[50]*20, help="Speed settings (20 values, 0~255)")
    parser.add_argument("--mm", type=int, default=30, help="Object diameter (mm)")
    args = parser.parse_args()
    main(args)
