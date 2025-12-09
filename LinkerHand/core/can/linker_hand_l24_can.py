#!/usr/bin/env python3
import can
import time,sys,os
import threading
import numpy as np
from enum import Enum
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, ".."))
sys.path.append(target_dir)
from utils.color_msg import ColorMsg

class FrameProperty(Enum):
    INVALID_FRAME_PROPERTY = 0x00  # 无效的can帧属性 | 无返回
    # 并行指令区域
    ROLL_POS = 0x01  # 横滚关节位置 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    YAW_POS = 0x02  # 航向关节位置 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    ROOT1_POS = 0x03  # 指根1关节位置 | 最接近手掌的指根关节
    ROOT2_POS = 0x04  # 指根2关节位置 | 最接近手掌的指根关节
    ROOT3_POS = 0x05  # 指根3关节位置 | 最接近手掌的指根关节
    TIP_POS = 0x06  # 指尖关节位置 | 最接近手掌的指根关节

    ROLL_SPEED = 0x09  # 横滚关节速度 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    YAW_SPEED = 0x0A  # 航向关节速度 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    ROOT1_SPEED = 0x0B  # 指根1关节速度 | 最接近手掌的指根关节
    ROOT2_SPEED = 0x0C  # 指根2关节速度 | 最接近手掌的指根关节
    ROOT3_SPEED = 0x0D  # 指根3关节速度 | 最接近手掌的指根关节
    TIP_SPEED = 0x0E  # 指尖关节速度 | 最接近手掌的指根关节

    ROLL_TORQUE = 0x11  # 横滚关节扭矩 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    YAW_TORQUE = 0x12  # 航向关节扭矩 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    ROOT1_TORQUE = 0x13  # 指根1关节扭矩 | 最接近手掌的指根关节
    ROOT2_TORQUE = 0x14  # 指根2关节扭矩 | 最接近手掌的指根关节
    ROOT3_TORQUE = 0x15  # 指根3关节扭矩 | 最接近手掌的指根关节
    TIP_TORQUE = 0x16  # 指尖关节扭矩 | 最接近手掌的指根关节

    ROLL_FAULT = 0x19  # 横滚关节故障码 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    YAW_FAULT = 0x1A  # 航向关节故障码 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    ROOT1_FAULT = 0x1B  # 指根1关节故障码 | 最接近手掌的指根关节
    ROOT2_FAULT = 0x1C  # 指根2关节故障码 | 最接近手掌的指根关节
    ROOT3_FAULT = 0x1D  # 指根3关节故障码 | 最接近手掌的指根关节
    TIP_FAULT = 0x1E  # 指尖关节故障码 | 最接近手掌的指根关节

    ROLL_TEMPERATURE = 0x21  # 横滚关节温度 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    YAW_TEMPERATURE = 0x22  # 航向关节温度 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    ROOT1_TEMPERATURE = 0x23  # 指根1关节温度 | 最接近手掌的指根关节
    ROOT2_TEMPERATURE = 0x24  # 指根2关节温度 | 最接近手掌的指根关节
    ROOT3_TEMPERATURE = 0x25  # 指根3关节温度 | 最接近手掌的指根关节
    TIP_TEMPERATURE = 0x26  # 指尖关节温度 | 最接近手掌的指根关节
    # 并行指令区域

    # 串行指令区域
    THUMB_POS = 0x41  # 大拇指指关节位置 | 返回本类型数据
    INDEX_POS = 0x42  # 食指关节位置 | 返回本类型数据
    MIDDLE_POS = 0x43  # 中指关节位置 | 返回本类型数据
    RING_POS = 0x44  # 无名指关节位置 | 返回本类型数据
    LITTLE_POS = 0x45  # 小拇指关节位置 | 返回本类型数据

    THUMB_SPEED = 0x49  # 大拇指速度 | 返回本类型数据
    INDEX_SPEED = 0x4A  # 食指速度 | 返回本类型数据
    MIDDLE_SPEED = 0x4B  # 中指速度 | 返回本类型数据
    RING_SPEED = 0x4C  # 无名指速度 | 返回本类型数据
    LITTLE_SPEED = 0x4D  # 小拇指速度 | 返回本类型数据

    THUMB_TORQUE = 0x51  # 大拇指扭矩 | 返回本类型数据
    INDEX_TORQUE = 0x52  # 食指扭矩 | 返回本类型数据
    MIDDLE_TORQUE = 0x53  # 中指扭矩 | 返回本类型数据
    RING_TORQUE = 0x54  # 无名指扭矩 | 返回本类型数据
    LITTLE_TORQUE = 0x55  # 小拇指扭矩 | 返回本类型数据

    THUMB_FAULT = 0x59  # 大拇指故障码 | 返回本类型数据
    INDEX_FAULT = 0x5A  # 食指故障码 | 返回本类型数据
    MIDDLE_FAULT = 0x5B  # 中指故障码 | 返回本类型数据
    RING_FAULT = 0x5C  # 无名指故障码 | 返回本类型数据
    LITTLE_FAULT = 0x5D  # 小拇指故障码 | 返回本类型数据

    THUMB_TEMPERATURE = 0x61  # 大拇指温度 | 返回本类型数据
    INDEX_TEMPERATURE = 0x62  # 食指温度 | 返回本类型数据
    MIDDLE_TEMPERATURE = 0x63  # 中指温度 | 返回本类型数据
    RING_TEMPERATURE = 0x64  # 无名指温度 | 返回本类型数据
    LITTLE_TEMPERATURE = 0x65  # 小拇指温度 | 返回本类型数据
    # 串行指令区域

    # 合并指令区域，同一手指非必要单控数据合并
    FINGER_SPEED = 0x81  # 手指速度 | 返回本类型数据
    FINGER_TORQUE = 0x82  # 转矩 | 返回本类型数据
    FINGER_FAULT = 0x83  # 手指故障码 | 返回本类型数据

    # 指尖传感器数据组
    HAND_NORMAL_FORCE = 0x90  # 五指法向压力
    HAND_TANGENTIAL_FORCE = 0x91  # 五指切向压力
    HAND_TANGENTIAL_FORCE_DIR = 0x92  # 五指切向方向
    HAND_APPROACH_INC = 0x93  # 五指接近感应

    THUMB_ALL_DATA = 0x98  # 大拇指所有数据
    INDEX_ALL_DATA = 0x99  # 食指所有数据
    MIDDLE_ALL_DATA = 0x9A  # 中指所有数据
    RING_ALL_DATA = 0x9B  # 无名指所有数据
    LITTLE_ALL_DATA = 0x9C  # 小拇指所有数据
    # 动作指令 ·ACTION
    ACTION_PLAY = 0xA0  # 动作

    # 配置命令·CONFIG
    HAND_UID = 0xC0  # 设备唯一标识码
    HAND_HARDWARE_VERSION = 0xC1  # 硬件版本
    HAND_SOFTWARE_VERSION = 0xC2  # 软件版本
    HAND_COMM_ID = 0xC3  # 设备id
    HAND_FACTORY_RESET = 0xCE  # 恢复出厂设置
    HAND_SAVE_PARAMETER = 0xCF  # 保存参数

    WHOLE_FRAME = 0xF0  # 整帧传输 | 返回一字节帧属性+整个结构体485及网络传输专属

class LinkerHandL24Can:
    def __init__(self, config, can_channel='can0', baudrate=1000000, can_id=0x28):
        self.config = config
        self.can_id = can_id
        self.running = True
        self.x01, self.x02, self.x03, self.x04,self.x05,self.x06,self.x07, self.x08,self.x09,self.x0A,self.x0B,self.x0C,self.x0D,self.x0E,self.speed = [],[],[],[],[],[],[],[],[],[],[],[],[],[],[]
        # 速度
        self.x49, self.x4a, self.x4b, self.x4c, self.x4d = [],[],[],[],[]
        self.x41,self.x42,self.x43,self.x44,self.x45 = [],[],[],[],[]
        # 根据操作系统初始化 CAN 总线
        if sys.platform == "linux":
            self.bus = can.interface.Bus(
                channel=can_channel, interface="socketcan", bitrate=baudrate, 
                can_filters=[{"can_id": can_id, "can_mask": 0x7FF}]
            )
        elif sys.platform == "win32":
            self.bus = can.interface.Bus(
                channel=can_channel, interface='pcan', bitrate=baudrate, 
                can_filters=[{"can_id": can_id, "can_mask": 0x7FF}]
            )
        else:
            raise EnvironmentError("Unsupported platform for CAN interface")

        # 根据 can_id 初始化 publisher 和相关参数
        if can_id == 0x28:  # 左手
            self.hand_exists = config['LINKER_HAND']['LEFT_HAND']['EXISTS']
            self.hand_joint = config['LINKER_HAND']['LEFT_HAND']['JOINT']
            self.hand_names = config['LINKER_HAND']['LEFT_HAND']['NAME']
        elif can_id == 0x27:  # 右手

            self.hand_exists = config['LINKER_HAND']['RIGHT_HAND']['EXISTS']
            self.hand_joint = config['LINKER_HAND']['RIGHT_HAND']['JOINT']
            self.hand_names = config['LINKER_HAND']['RIGHT_HAND']['NAME']


        # 启动接收线程
        self.receive_thread = threading.Thread(target=self.receive_response)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def send_command(self, frame_property, data_list):
        """
        发送命令到 CAN 总线
        :param frame_property: 数据帧属性
        :param data_list: 数据载荷
        """
        frame_property_value = int(frame_property.value) if hasattr(frame_property, 'value') else frame_property
        data = [frame_property_value] + [int(val) for val in data_list]
        msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            #print(f"Message sent: ID={hex(self.can_id)}, Data={data}")
        except can.CanError as e:
            print(f"Failed to send message: {e}")
        time.sleep(0.002)

    def receive_response(self):
        """
        接收并处理 CAN 总线的响应消息
        """
        while self.running:
            try:
                msg = self.bus.recv(timeout=1.0)  # 阻塞接收，1 秒超时
                if msg:
                    self.process_response(msg)
            except can.CanError as e:
                print(f"Error receiving message: {e}")
    

    def set_joint_positions(self, joint_ranges):
        if len(joint_ranges) == 25:
            l24_pose = self.joint_map(joint_ranges)
            # 使用列表推导式将列表每6个元素切成一个子数组
            chunks = [l24_pose[i:i+6] for i in range(0, 30, 6)]
            self.send_command(FrameProperty.THUMB_POS, chunks[0])
            self.send_command(FrameProperty.INDEX_POS, chunks[1])
            self.send_command(FrameProperty.MIDDLE_POS, chunks[2])
            self.send_command(FrameProperty.RING_POS, chunks[3])
            self.send_command(FrameProperty.LITTLE_POS, chunks[4])
        #self.set_tip_positions(joint_ranges[:5])
        #print(l24_pose)
    
    # 设置所有手指横滚关节位置
    def set_roll_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROLL_POS, joint_ranges)
    # 设置所有手指航向关节位置
    def set_yaw_positions(self, joint_ranges):
        self.send_command(FrameProperty.YAW_POS, joint_ranges)
    # 设置所有手指指根1关节位置
    def set_root1_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT1_POS, joint_ranges)
    # 设置所有手指指根2关节位置
    def set_root2_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT2_POS, joint_ranges)
    # 设置所有手指指根3关节位置
    def set_root3_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT3_POS, joint_ranges)
    # 设置所有手指指尖关节位置
    def set_tip_positions(self, joint_ranges=[80]*5):
        self.send_command(FrameProperty.TIP_POS, joint_ranges)
    # 获取大拇指指关节位置
    def get_thumb_positions(self,j=[0]):
        self.send_command(FrameProperty.THUMB_POS, j)
    # 获取食指关节位置
    def get_index_positions(self, j=[0]):
        self.send_command(FrameProperty.INDEX_POS,j)
    # 获取中指关节位置
    def get_middle_positions(self, j=[0]):
        self.send_command(FrameProperty.MIDDLE_POS,j)
    # 获取无名指关节位置
    def get_ring_positions(self, j=[0]):
        self.send_command(FrameProperty.RING_POS,j)
    # 获取小拇指关节位置
    def get_little_positions(self, j=[0]):
        self.send_command(FrameProperty.LITTLE_POS, j)
    # 失能01模式
    def set_disability_mode(self, j=[1,1,1,1,1]):
        self.send_command(0x85,j)
    # 使能00模式
    def set_enable_mode(self, j=[00,00,00,00,00]):
        self.send_command(0x85,j)

    
    def set_speed(self, speed):
        self.speed = [speed]*6
        ColorMsg(msg=f"L24设置速度为:{self.speed}", color="yellow")
        self.send_command(FrameProperty.THUMB_SPEED, self.speed)
        self.send_command(FrameProperty.INDEX_SPEED, self.speed)
        self.send_command(FrameProperty.MIDDLE_SPEED, self.speed)
        self.send_command(FrameProperty.RING_SPEED, self.speed)
        self.send_command(FrameProperty.LITTLE_SPEED, self.speed)
        
    def set_finger_torque(self, torque):
        self.send_command(0x42, torque)

    def request_device_info(self):
        self.send_command(0xC0, [0])
        self.send_command(0xC1, [0])
        self.send_command(0xC2, [0])

    def save_parameters(self):
        self.send_command(0xCF, [])
    def process_response(self, msg):
        if msg.arbitration_id == self.can_id:
            frame_type = msg.data[0]
            response_data = msg.data[1:]
            if frame_type == 0x01:
                self.x01 = list(response_data)
            elif frame_type == 0x02:
                self.x02 = list(response_data)
            elif frame_type == 0x03:
                self.x03 = list(response_data)
            elif frame_type == 0x04:
                self.x04 = list(response_data)
            elif frame_type == 0x05:
                self.x05 = list(response_data)
            elif frame_type == 0x06:
                self.x06 = list(response_data)
                print("_-"*20)
                print(self.x06)
            elif frame_type == 0xC0:
                print(f"Device ID info: {response_data}")
                if self.can_id == 0x28:
                    self.right_hand_info = response_data
                elif self.can_id == 0x27:
                    self.left_hand_info = response_data
            elif frame_type == 0x08:
                self.x08 = list(response_data)
            elif frame_type == 0x09:
                self.x09 = list(response_data)
            elif frame_type == 0x0A:
                self.x0A = list(response_data)
            elif frame_type == 0x0B:
                self.x0B = list(response_data)
            elif frame_type == 0x0C:
                self.x0C = list(response_data)
            elif frame_type == 0x0D:
                self.x0D = list(response_data)
            elif frame_type == 0x22:
                #ColorMsg(msg=f"五指切向压力方向：{list(response_data)}")
                d = list(response_data)
                self.tangential_force_dir = [float(i) for i in d]
            elif frame_type == 0x23:
                #ColorMsg(msg=f"五指接近度：{list(response_data)}")
                d = list(response_data)
                self.approach_inc = [float(i) for i in d]
            elif frame_type == 0x41: # 拇指关节位置返回值
                self.x41 = list(response_data)
            elif frame_type == 0x42: # 食指关节位置返回值
                self.x42 = list(response_data)
            elif frame_type == 0x43: # 中指关节位置返回值
                self.x43 = list(response_data)
            elif frame_type == 0x44: # 无名指关节位置返回值
                self.x44 = list(response_data)
            elif frame_type == 0x45: # 小拇指关节位置返回值
                self.x45 = list(response_data)
            elif frame_type == 0x49: # 拇指速度返回值
                self.x49 = list(response_data)
            elif frame_type == 0x4a: # 食指速度返回值
                self.x4a = list(response_data)
            elif frame_type == 0x4b: # 中指速度返回值
                self.x4b = list(response_data)
            elif frame_type == 0x4c: # 无名指速度返回值
                self.x4c = list(response_data)
            elif frame_type == 0x4d: # 小拇指速度返回值
                self.x4d = list(response_data)

    # topic映射L24
    def joint_map(self, pose):
        # L24 CAN数据默认接收30个数据
        l24_pose = [0.0] * 30  # 初始化l24_pose为30个0.0

        # 映射表，通过字典简化映射关系
        mapping = {
            0: 10,  1: 5,   2: 0,   3: 15,  4: None,  5: 20,
            6: None, 7: 6,   8: 1,   9: 16,  10: None, 11: 21,
            12: None, 13: None, 14: 2,  15: 17, 16: None, 17: 22,
            18: None, 19: 8,  20: 3,   21: 18, 22: None, 23: 23,
            24: None, 25: 9,  26: 4,   27: 19, 28: None, 29: 24
        }

        # 遍历映射字典，进行值的映射
        for l24_idx, pose_idx in mapping.items():
            if pose_idx is not None:
                l24_pose[l24_idx] = pose[pose_idx]

        return l24_pose

    # 将L24的状态值转换为CMD格式的状态值
    def state_to_cmd(self, l24_state):
        # L24 CAN默认接收30个数据，初始化pose为25个0.0
        pose = [0.0] * 25  # 原来控制L24的指令数据为25个

        # 映射关系，字典中存储l24_state索引和pose索引之间的映射关系
        mapping = {
            0: 10,  1: 5,   2: 0,   3: 15,  5: 20,  7: 6,
            8: 1,   9: 16,  11: 21, 14: 2,  15: 17, 17: 22,
            19: 8,  20: 3,  21: 18, 23: 23, 25: 9,   26: 4,
            27: 19, 29: 24
        }
        # 遍历映射字典，更新pose的值
        for l24_idx, pose_idx in mapping.items():
            pose[pose_idx] = l24_state[l24_idx]
        return pose

    # 获取所有关节数据
    def get_current_status(self, j=''):
        time.sleep(0.01)
        self.send_command(FrameProperty.THUMB_POS, j)
        self.send_command(FrameProperty.INDEX_POS,j)
        self.send_command(FrameProperty.MIDDLE_POS,j)
        self.send_command(FrameProperty.RING_POS,j)
        self.send_command(FrameProperty.LITTLE_POS, j)
        #return self.x41, self.x42, self.x43, self.x44, self.x45
        time.sleep(0.1)
        state= self.x41+ self.x42+ self.x43+ self.x44+ self.x45
        if len(state) == 30:
            l24_state = self.state_to_cmd(l24_state=state)
            return l24_state
    
    def get_speed(self,j=''):
        time.sleep(0.1)
        self.send_command(FrameProperty.THUMB_SPEED, j) # 大拇指速度
        self.send_command(FrameProperty.INDEX_SPEED, j) # 食指速度
        self.send_command(FrameProperty.MIDDLE_SPEED, j) # 中指速度
        self.send_command(FrameProperty.RING_SPEED, j) # 无名指速度
        self.send_command(FrameProperty.LITTLE_SPEED, j) # 小拇指速度
        speed = self.x49+ self.x4a+ self.x4b+ self.x4c+ self.x4d
        if len(speed) == 30:
            l24_speed = self.state_to_cmd(l24_state=speed)
            return l24_speed
    
    def get_finger_torque(self):
        return self.finger_torque
    # def get_current(self):
    #     return self.x06
    # def get_fault(self):
    #     return self.x07
    def close_can_interface(self):
        if self.bus:
            self.bus.shutdown()  # 关闭 CAN 总线

    '''
    这个方法只用于展示数据关系映射，使用的话最好使用上面的方法
    '''
    def joint_map_2(self, pose):
        l24_pose = [0.0]*30 #L24 CAN默认接收30个数据 pose控制L24发送的指令数据默认25个，这里进行映射
        '''
        需要进行映射
        # L24 CAN数据格式
        #["拇指横摆0-10", "拇指侧摆1-5", "拇指根部2-0", "拇指中部3-15", "预留4-", "拇指指尖5-20", "预留6-", "食指侧摆7-6", "食指根部8-1", "食指中部9-16", "预留10-", "食指指尖11-21", "预留12-", "预留13-", "中指根部14-2", "中指中部15-17", "预留16-", "中指指尖17-22", "预留18-", "无名指侧摆19-8", "无名指根部20-3", "无名指中部21-18", "预留22-", "无名指指尖23-23", "预留24-", "小指侧摆25-9", "小指根部26-4", "小指中部27-19", "预留28-", "小指指尖29-24"]
        # CMD 接收到的数据格式
        #["拇指根部0", "食指根部1", "中指根部2", "无名指根部3","小指根部4","拇指侧摆5","食指侧摆6","中指侧摆","无名指侧摆8","小指侧摆9","拇指横摆10","预留","预留","预留","预留","拇指中部15","食指中部16","中指中部17","无名指中部18","小指中部19","拇指指尖20","食指指尖21","中指指尖22","无名指指尖23","小指指尖24"]
        '''
        l24_pose[0] = pose[10]
        l24_pose[1] = pose[5]
        l24_pose[2] = pose[0]
        l24_pose[3] = pose[15]
        l24_pose[4] = 0.0
        l24_pose[5] = pose[20]
        l24_pose[6] = 0.0
        l24_pose[7] = pose[6]
        l24_pose[8] = pose[1]
        l24_pose[9] = pose[16]
        l24_pose[10] = 0.0
        l24_pose[11] = pose[21]
        l24_pose[12] = 0.0
        l24_pose[13] = 0.0
        l24_pose[14] = pose[2]
        l24_pose[15] = pose[17]
        l24_pose[16] = 0.0
        l24_pose[17] = pose[22]
        l24_pose[18] = 0.0
        l24_pose[19] = pose[8]
        l24_pose[20] = pose[3]
        l24_pose[21] = pose[18]
        l24_pose[22] = 0.0
        l24_pose[23] = pose[23]
        l24_pose[24] = 0.0
        l24_pose[25] = pose[9]
        l24_pose[26] = pose[4]
        l24_pose[27] = pose[19]
        l24_pose[28] = 0.0
        l24_pose[29] = pose[24]
        return l24_pose
    
    def get_finger_order(self):
        return []
    def get_serial_number(self):
        return [0] * 6
    def show_fun_table(self):
        pass