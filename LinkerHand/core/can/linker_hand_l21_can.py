#!/usr/bin/env python3
import can
import time, sys, os
import threading
import numpy as np
from enum import Enum
from utils.open_can import OpenCan
from utils.color_msg import ColorMsg
from can.exceptions import CanError
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, ".."))
sys.path.append(target_dir)

class FrameProperty(Enum):
    # Finger motion control - parallel control commands
    ROLL_POS = 0x01  # Roll joint position
    YAWPOS = 0x02  # Yaw joint position
    ROOT1_POS = 0x03  # Root joint 1 position
    ROOT2_POS = 0x04  # Root joint 2 position
    ROOT3_POS = 0x05  # Root joint 3 position
    TIP_POS = 0x06  # Fingertip joint position
    # Finger motion control - serial control commands
    THUMB_POS = 0x41  # Thumb joint position
    INDEX_POS = 0x42  # Index finger joint position
    MIDDLE_POS = 0x43  # Middle finger joint position
    RING_POS = 0x44  # Ring finger joint position
    LITTLE_POS = 0x45  # Little finger joint position

    # Finger motion control - speed
    ROLL_SPEED = 0x09  # Roll joint speed
    YAW_SPEED = 0x0A  # Yaw joint speed
    ROOT1_SPEED = 0x0B  # Root joint 1 speed
    ROOT2_SPEED = 0x0C  # Root joint 2 speed
    ROOT3_SPEED = 0x0D  # Root joint 3 speed
    TIP_SPEED = 0x0E  # Fingertip joint speed
    THUMB_SPEED = 0x49  # Thumb speed
    INDEX_SPEED = 0x4A  # Index finger speed
    MIDDLE_SPEED = 0x4B  # Middle finger speed
    RING_SPEED = 0x4C  # Ring finger speed
    LITTLE_SPEED = 0x4D  # Little finger speed

    # Finger motion control - torque
    ROLL_TORQUE = 0x11  # Roll joint torque
    YAW_TORQUE = 0x12  # Yaw joint torque
    ROOT1_TORQUE = 0x13  # Root joint 1 torque
    ROOT2_TORQUE = 0x14  # Root joint 2 torque
    ROOT3_TORQUE = 0x15  # Root joint 3 torque
    TIP_TORQUE = 0x16  # Fingertip joint torque
    THUMB_TORQUE = 0x51  # Thumb torque
    INDEX_TORQUE = 0x52  # Index finger torque
    MIDDLE_TORQUE = 0x53  # Middle finger torque
    RING_TORQUE = 0x54  # Ring finger torque
    LITTLE_TORQUE = 0x55  # Little finger torque

    THUMB_FAULT = 0x59  # Thumb fault code | Returns this type of data
    INDEX_FAULT = 0x5A  # Index finger fault code | Returns this type of data
    MIDDLE_FAULT = 0x5B  # Middle finger fault code | Returns this type of data
    RING_FAULT = 0x5C  # Ring finger fault code | Returns this type of data
    LITTLE_FAULT = 0x5D  # Little finger fault code | Returns this type of data

    # Finger faults and temperature
    ROLL_FAULT = 0x19  # Roll joint fault code
    YAW_FAULT = 0x1A  # Yaw joint fault code
    ROOT1_FAULT = 0x1B  # Root joint 1 fault code
    ROOT2_FAULT = 0x1C  # Root joint 2 fault code
    ROOT3_FAULT = 0x1D  # Root joint 3 fault code
    TIP_FAULT = 0x1E  # Fingertip joint fault code
    ROLL_TEMPERATURE = 0x21  # Roll joint over-temperature protection threshold
    YAW_TEMPERATURE = 0x22  # Yaw joint over-temperature protection threshold
    ROOT1_TEMPERATURE = 0x23  # Root joint 1 over-temperature protection threshold
    ROOT2_TEMPERATURE = 0x24  # Root joint 2 over-temperature protection threshold
    ROOT3_TEMPERATURE = 0x25  # Root joint 3 over-temperature protection threshold
    TIP_TEMPERATURE = 0x26  # Fingertip joint over-temperature protection threshold
    THUMB_TEMPERATURE = 0x61  # Thumb over-temperature protection threshold
    INDEX_TEMPERATURE = 0x62  # Index finger over-temperature protection threshold
    MIDDLE_TEMPERATURE = 0x63  # Middle finger over-temperature protection threshold
    RING_TEMPERATURE = 0x64  # Ring finger over-temperature protection threshold
    LITTLE_TEMPERATURE = 0x65  # Little finger over-temperature protection threshold

    # Configuration and preset actions
    HAND_UID = 0xC0  # Device unique identifier
    HAND_HARDWARE_VERSION = 0xC1  # Hardware version
    HAND_SOFTWARE_VERSION = 0xC2  # Software version
    HAND_COMM_ID = 0xC3  # Device ID
    HAND_FACTORY_RESET = 0xCE  # Restore factory settings
    HAND_SAVE_PARAMETER = 0xCF  # Save parameters

    # Tactile sensor data
    HAND_NORMAL_FORCE = 0x90  # Normal force of five fingers
    HAND_TANGENTIAL_FORCE = 0x91  # Tangential force of five fingers
    HAND_TANGENTIAL_FORCE_DIR = 0x92  # Tangential direction of five fingers
    HAND_APPROACH_INC = 0x93  # Approach sensing of five fingers

    TOUCH_SENSOR_TYPE = 0xB0  # Sensor type
    THUMB_TOUCH = 0xB1  # Thumb tactile sensing
    INDEX_TOUCH = 0xB2  # Index finger tactile sensing
    MIDDLE_TOUCH = 0xB3  # Middle finger tactile sensing
    RING_TOUCH = 0xB4  # Ring finger tactile sensing
    LITTLE_TOUCH = 0xB5  # Little finger tactile sensing
    PALM_TOUCH = 0xB6  # Palm tactile sensing

    # Action control
    ACTION_PLAY = 0xA0  # Action

    # Combined command area
    FINGER_SPEED = 0x81  # Set maximum finger speed
    FINGER_TORQUE = 0x82  # Set maximum finger torque
    FINGER_FAULT = 0x83  # Clear finger faults and fault codes
    FINGER_TEMPERATURE = 0x84  # Finger joint temperatures

class LinkerHandL21Can:
    def __init__(self, can_channel='can0', baudrate=1000000, can_id=0x28,yaml=""):
        self.can_id = can_id
        self.can_channel = can_channel
        self.baudrate = baudrate
        self.open_can = OpenCan(load_yaml=yaml)

        self.running = True
        self.last_thumb_pos, self.last_index_pos,self.last_ring_pos,self.last_middle_pos, self.last_little_pos = None,None,None,None,None
        self.x01, self.x02, self.x03, self.x04,self.x05,self.x06,self.x07, self.x08,self.x09,self.x0A,self.x0B,self.x0C,self.x0D,self.x0E,self.speed = [],[],[],[],[],[],[],[],[],[],[],[],[],[],[]
        self.last_root1,self.last_yaw,self.last_roll,self.last_root2,self.last_tip = None,None,None,None,None
        # Speed
        self.x49, self.x4a, self.x4b, self.x4c, self.x4d,self.xc1 = [],[],[],[],[],[]
        self.x41,self.x42,self.x43,self.x44,self.x45 = [],[],[],[],[]
        self.x83 = [-1] * 5
        # Torque
        self.x51, self.x52, self.x53, self.x54,self.x55 = [],[],[],[],[]
        # Fault codes
        self.x59,self.x5a,self.x5b,self.x5c,self.x5d = [-1] * 5,[-1] * 5,[-1] * 5,[-1] * 5,[-1] * 5
        # Temperature thresholds
        self.x61,self.x62,self.x63,self.x64,self.x65 = [],[],[],[],[]
        # Pressure sensors
        self.x90,self.x91,self.x92,self.x93 = [],[],[],[]
        # New pressure sensors
        self.xb0,self.xb1,self.xb2,self.xb3,self.xb4,self.xb5,self.xb6 = [-1] * 5,[-1] * 5,[-1] * 5,[-1] * 5,[-1] * 5,[-1] * 5,[-1] * 5
        self.thumb_matrix = np.full((12, 6), -1)
        self.index_matrix = np.full((12, 6), -1)
        self.middle_matrix = np.full((12, 6), -1)
        self.ring_matrix = np.full((12, 6), -1)
        self.little_matrix = np.full((12, 6), -1)
        self.matrix_map = {
            0: 0,
            16: 1,
            32: 2,
            48: 3,
            64: 4,
            80: 5,
            96: 6,
            112: 7,
            128: 8,
            144: 9,
            160: 10,
            176: 11,
        }
        # Initialize CAN bus according to operating system
        # try:
        #     if sys.platform == "linux":
        #         self.open_can.open_can(self.can_channel)
        #         time.sleep(0.1)
        #         self.bus = can.interface.Bus(
        #             channel=can_channel, interface="socketcan", bitrate=baudrate, 
        #             can_filters=[{"can_id": can_id, "can_mask": 0x7FF}]
        #         )
        #     elif sys.platform == "win32":
        #         self.bus = can.interface.Bus(
        #             channel=can_channel, interface='pcan', bitrate=baudrate, 
        #             can_filters=[{"can_id": can_id, "can_mask": 0x7FF}]
        #         )
        #     else:
        #         raise EnvironmentError("Unsupported platform for CAN interface")
        # except:
        #     print("Please insert CAN device")
        self.bus = self.init_can_bus(channel=self.can_channel, baudrate=baudrate)

        # Start receive thread
        self.receive_thread = threading.Thread(target=self.receive_response)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def init_can_bus(self, channel, baudrate):
        """
        尝试按优先级连接 CAN 总线，并实现回退机制。
        """
        # --- 统一异常处理块开始 ---
        try:
            if sys.platform == "linux":
                # Linux 优先级：1. socketcan
                try:
                    self.open_can.open_can(self.can_channel)
                    # 尝试 socketcan
                    bus = can.interface.Bus(channel=channel, interface="socketcan", bitrate=baudrate)
                    ColorMsg(msg=f"成功连接: interface='socketcan', channel='{channel}'", color="green")
                    return bus
                except CanError as e:
                    # 如果 socketcan 失败，可以考虑在这里尝试其他 Linux 接口 (如 'pcan')
                    ColorMsg(msg=f"socketcan 接口连接失败: {e}", color="yellow")
                    raise # 重新抛出异常，让外层 try 捕获
            elif sys.platform == "win32":
                # Windows 优先级：1. pcan
                try:
                    bus = can.interface.Bus(channel=channel, interface='pcan', bitrate=baudrate)
                    ColorMsg(msg=f"成功连接: interface='pcan', channel='{channel}'", color="green")
                    return bus
                except CanError as e:
                    ColorMsg(msg=f"pcan 接口连接失败，尝试回退到 'candle': {e}", color="yellow")
                # Windows 优先级：2. candle (回退方法)
                try:
                    bus = can.Bus(interface="candle", channel=channel, bitrate=baudrate)
                    ColorMsg(msg=f"成功连接: interface='candle', channel='{channel}'", color="green")
                    return bus
                except CanError as e:
                    ColorMsg(msg=f"candle 接口连接失败: {e}", color="yellow")
                    raise # 两个接口都失败，抛出异常
            else:
                raise EnvironmentError("Unsupported platform for CAN interface")
        # --- 统一异常处理块结束 ---
        except Exception as e:
            # 如果任何一个接口尝试失败并抛出异常（包括 EnvironmentError）
            ColorMsg(msg=f"致命错误：所有 CAN 接口连接尝试均失败或平台不受支持。请检查设备连接或驱动安装和配置文件中CAN参数的配置。\n错误详情: {e}", color="red")
            # 保持 raise 动作，将错误信息传递给调用者，避免程序继续运行
            raise

    def send_command(self, frame_property, data_list,sleep_time=0.003):
        """
        Send command to CAN bus
        :param frame_property: Data frame property
        :param data_list: Data payload
        """
        frame_property_value = int(frame_property.value) if hasattr(frame_property, 'value') else frame_property
        data = [frame_property_value] + [int(val) for val in data_list]
        msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"Failed to send message: {e}")
            self.open_can.open_can(self.can_channel)
            time.sleep(1)
            self.is_can = self.open_can.is_can_up_sysfs(interface=self.can_channel)
            time.sleep(1)
            if self.is_can:
                self.bus = can.interface.Bus(channel=self.can_channel, interface="socketcan", bitrate=self.baudrate)
            else:
                print("Reconnecting CAN devices ....")
        time.sleep(sleep_time)

    def receive_response(self):
        """
        Receive and process CAN bus response messages
        """
        while self.running:
            try:
                msg = self.bus.recv(timeout=1.0)
                if msg:
                    self.process_response(msg)
            except can.CanError as e:
                print(f"Error receiving message: {e}")
    

    def set_joint_positions(self, joint_ranges):
        if len(joint_ranges) == 25:
            l21_pose = self.joint_map(joint_ranges)
            # Use list comprehension to split the list into subarrays of 6 elements each
            chunks = [l21_pose[i:i+6] for i in range(0, 30, 6)]
            for i in range(3):
                self.send_command(FrameProperty.THUMB_POS, chunks[0])
                time.sleep(0.001)
                self.send_command(FrameProperty.INDEX_POS, chunks[1])
                time.sleep(0.001)
                self.send_command(FrameProperty.MIDDLE_POS, chunks[2])
                time.sleep(0.001)
                self.send_command(FrameProperty.RING_POS, chunks[3])
                time.sleep(0.001)
                self.send_command(FrameProperty.LITTLE_POS, chunks[4])
                time.sleep(0.001)

    def set_joint_positions_by_topic(self, joint_ranges):
        if len(joint_ranges) == 25:
            l21_pose = self.slice_list(joint_ranges,5)
            if self._list_d_value(self.last_root1, l21_pose[0]):
                self.set_root1_positions(l21_pose[0])
                self.last_root1 = l21_pose[0]
            if self._list_d_value(self.last_yaw, l21_pose[1]):  
                self.set_yaw_positions(l21_pose[1])
                self.last_yaw = l21_pose[1]
            if self._list_d_value(self.last_roll, l21_pose[2]):
                self.set_roll_positions(l21_pose[2])
                self.last_roll = l21_pose[2]
            if self._list_d_value(self.last_root2, l21_pose[3]):
                self.set_root2_positions(l21_pose[3])
                self.last_root2 = l21_pose[3]
            if self._list_d_value(self.last_tip, l21_pose[4]): 
                self.set_tip_positions(l21_pose[4])
                self.last_tip = l21_pose[4]
            

    def slice_list(self, input_list, slice_size):
        """
        Slice a list into pieces of specified size.

        Args:
        input_list (list): The list to be sliced.
        slice_size (int): Number of elements per slice.

        Returns:
        list of lists: The sliced list.
        """
        sliced_list = [input_list[i:i + slice_size] for i in range(0, len(input_list), slice_size)]
        return sliced_list

    def _list_d_value(self,list1, list2):
        if list1 == None:
            return True
        for a, b in zip(list1, list2):
            if abs(b - a) > 2:
                return True
                break
        return False
    # Set all finger roll joint positions
    def set_roll_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROLL_POS, joint_ranges)
    # Set all finger yaw joint positions
    def set_yaw_positions(self, joint_ranges):
        self.send_command(FrameProperty.YAW_POS, joint_ranges)
    # Set all finger root1 joint positions
    def set_root1_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT1_POS, joint_ranges)
    # Set all finger root2 joint positions
    def set_root2_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT2_POS, joint_ranges)
    # Set all finger root3 joint positions
    def set_root3_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT3_POS, joint_ranges)
    # Set all finger tip joint positions
    def set_tip_positions(self, joint_ranges=[80]*5):
        self.send_command(FrameProperty.TIP_POS, joint_ranges)
    # Set thumb torque
    def set_thumb_torque(self, j=[]):
        self.send_command(FrameProperty.THUMB_TORQUE, j)
    # Set index finger torque
    def set_index_torque(self, j=[]):
        self.send_command(FrameProperty.INDEX_TORQUE, j)
    # Set middle finger torque
    def set_middle_torque(self, j=[]):
        self.send_command(FrameProperty.MIDDLE_TORQUE, j)
    # Set ring finger torque
    def set_ring_torque(self, j=[]):
        self.send_command(FrameProperty.RING_TORQUE, j)
    # Set little finger torque
    def set_little_torque(self, j=[]):
        self.send_command(FrameProperty.LITTLE_TORQUE, j)

    # Get thumb joint positions
    def get_thumb_positions(self,j=[0]):
        self.send_command(FrameProperty.THUMB_POS, j)
    # Get index finger joint positions
    def get_index_positions(self, j=[0]):
        self.send_command(FrameProperty.INDEX_POS,j)
    # Get middle finger joint positions
    def get_middle_positions(self, j=[0]):
        self.send_command(FrameProperty.MIDDLE_POS,j)
    # Get ring finger joint positions
    def get_ring_positions(self, j=[0]):
        self.send_command(FrameProperty.RING_POS,j)
    # Get little finger joint positions
    def get_little_positions(self, j=[0]):
        self.send_command(FrameProperty.LITTLE_POS, j)
    # Get all thumb motor fault codes
    def get_thumbn_fault(self,j=[]):
        self.send_command(FrameProperty.THUMB_FAULT,j)
    # Get all index finger motor fault codes
    def get_index_fault(self,j=[]):
        self.send_command(FrameProperty.INDEX_FAULT,j)
    # Get all middle finger motor fault codes
    def get_middle_fault(self,j=[]):
        self.send_command(FrameProperty.MIDDLE_FAULT,j)
    # Get all ring finger motor fault codes
    def get_ring_fault(self,j=[]):
        self.send_command(FrameProperty.RING_FAULT,j)
    # Get all little finger motor fault codes
    def get_little_fault(self,j=[]):
        self.send_command(FrameProperty.LITTLE_FAULT,j)
    # Get thumb temperature threshold
    def get_thumb_threshold(self,j=[]):
        self.send_command(FrameProperty.THUMB_TEMPERATURE, '')
    # Get index finger temperature threshold
    def get_index_threshold(self,j=[]):
        self.send_command(FrameProperty.INDEX_TEMPERATURE, j)
    # Get middle finger temperature threshold
    def get_middle_threshold(self,j=[]):
        self.send_command(FrameProperty.MIDDLE_TEMPERATURE, j)
    # Get ring finger temperature threshold
    def get_ring_threshold(self,j=[]):
        self.send_command(FrameProperty.RING_TEMPERATURE, j)
    # Get little finger temperature threshold
    def get_little_threshold(self,j=[]):
        self.send_command(FrameProperty.LITTLE_TEMPERATURE, j)

    # Disable mode 01
    def set_disability_mode(self, j=[1,1,1,1,1]):
        self.send_command(0x85,j)
    # Enable mode 00
    def set_enable_mode(self, j=[00,00,00,00,00]):
        self.send_command(0x85,j)
    
    # Set all finger torques
    def set_torque(self,torque=[250]*5):
        t = torque[0]
        i = torque[1]
        m = torque[2]
        r = torque[3]
        l = torque[4]
        self.set_thumb_torque(j=[t]*5)
        self.set_index_torque(j=[i]*5)
        self.set_middle_torque(j=[m]*5)
        self.set_ring_torque(j=[r]*5)
        self.set_little_torque(j=[l]*5)
    
    def set_speed(self, speed):
        self.speed = speed
        if len(speed) < 25:
            thumb_speed = [self.speed[0]]*5
            index_speed = [self.speed[1]]*5
            middle_speed = [self.speed[2]]*5
            ring_speed = [self.speed[3]]*5
            little_speed = [self.speed[4]]*5
        else:
            thumb_speed = [self.speed[0],self.speed[1],self.speed[2],self.speed[3],self.speed[4]]
            index_speed = [self.speed[5],self.speed[6],self.speed[7],self.speed[8],self.speed[9]]
            middle_speed = [self.speed[10],self.speed[11],self.speed[12],self.speed[13],self.speed[14]]
            ring_speed = [self.speed[15],self.speed[16],self.speed[17],self.speed[18],self.speed[19]]
            little_speed = [self.speed[20],self.speed[21],self.speed[22],self.speed[23],self.speed[24]]
        self.send_command(FrameProperty.THUMB_SPEED, thumb_speed)
        self.send_command(FrameProperty.INDEX_SPEED, index_speed)
        self.send_command(FrameProperty.MIDDLE_SPEED, middle_speed)
        self.send_command(FrameProperty.RING_SPEED, ring_speed)
        self.send_command(FrameProperty.LITTLE_SPEED, little_speed)
        
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
            if len(list(response_data)) == 0:
                return
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
                d = list(response_data)
                self.tangential_force_dir = [float(i) for i in d]
            elif frame_type == 0x23:
                d = list(response_data)
                self.approach_inc = [float(i) for i in d]
            elif frame_type == 0x41:
                self.x41 = list(response_data)
            elif frame_type == 0x42:
                self.x42 = list(response_data)
            elif frame_type == 0x43:
                self.x43 = list(response_data)
            elif frame_type == 0x44:
                self.x44 = list(response_data)
            elif frame_type == 0x45:
                self.x45 = list(response_data)
            elif frame_type == 0x49:
                self.x49 = list(response_data)
            elif frame_type == 0x4a:
                self.x4a = list(response_data)
            elif frame_type == 0x4b:
                self.x4b = list(response_data)
            elif frame_type == 0x4c:
                self.x4c = list(response_data)
            elif frame_type == 0x4d:
                self.x4d = list(response_data)
            elif frame_type == 0xc1:
                self.xc1 = list(response_data)
            elif frame_type == 0x51:
                self.x51 = list(response_data)
            elif frame_type == 0x52:
                self.x52 = list(response_data)
            elif frame_type == 0x53:
                self.x53 = list(response_data)
            elif frame_type == 0x54:
                self.x54 = list(response_data)
            elif frame_type == 0x55:
                self.x55 = list(response_data)
            elif frame_type == 0x59:
                self.x59 = list(response_data)
            elif frame_type == 0x5a:
                self.x5a = list(response_data)
            elif frame_type == 0x5b:
                self.x5b = list(response_data)
            elif frame_type == 0x5c:
                self.x5c = list(response_data)
            elif frame_type == 0x5d:
                self.x5d = list(response_data)
            elif frame_type == 0x61:
                self.x61 = list(response_data)
            elif frame_type == 0x62:
                self.x62 = list(response_data)
            elif frame_type == 0x63:
                self.x63 = list(response_data)
            elif frame_type == 0x64:
                self.x64 = list(response_data)
            elif frame_type == 0x65:
                self.x65 = list(response_data)
            elif frame_type == 0x83:
                self.x83 = list(response_data)
            elif frame_type == 0x90:
                self.x90 = list(response_data)
            elif frame_type == 0x91:
                self.x91 = list(response_data)
            elif frame_type == 0x92:
                self.x92 = list(response_data)
            elif frame_type == 0x93:
                self.x93 = list(response_data)
            elif frame_type == 0xb0:
                self.xb0 = list(response_data)
            elif frame_type == 0xb1:
                d = list(response_data)
                if len(d) == 2:
                    self.xb1 = d
                elif len(d) == 7:
                    index = self.matrix_map.get(d[0])
                    if index is not None:
                        self.thumb_matrix[index] = d[1:]
            elif frame_type == 0xb2:
                d = list(response_data)
                if len(d) == 2:
                    self.xb2 = d
                elif len(d) == 7:
                    index = self.matrix_map.get(d[0])
                    if index is not None:
                        self.index_matrix[index] = d[1:]
            elif frame_type == 0xb3:
                d = list(response_data)
                if len(d) == 2:
                    self.xb3 = d
                elif len(d) == 7:
                    index = self.matrix_map.get(d[0])
                    if index is not None:
                        self.middle_matrix[index] = d[1:]
            elif frame_type == 0xb4:
                d = list(response_data)
                if len(d) == 2:
                    self.xb4 = d
                elif len(d) == 7:
                    index = self.matrix_map.get(d[0])
                    if index is not None:
                        self.ring_matrix[index] = d[1:]
            elif frame_type == 0xb5:
                d = list(response_data)
                if len(d) == 2:
                    self.xb5 = d
                elif len(d) == 7:
                    index = self.matrix_map.get(d[0])
                    if index is not None:
                        self.little_matrix[index] = d[1:]
            elif frame_type == 0xb6:
                self.xb6 = list(response_data)

    def joint_map(self, pose):
        # l21 CAN data by default receives 30 data
        l21_pose = [0.0] * 30

        mapping = {
            0: 10,  1: 5,   2: 0,   3: 15,  4: None,  5: 20,
            6: None, 7: 6,   8: 1,   9: 16,  10: None, 11: 21,
            12: None, 13: 7, 14: 2,  15: 17, 16: None, 17: 22,
            18: None, 19: 8,  20: 3,   21: 18, 22: None, 23: 23,
            24: None, 25: 9,  26: 4,   27: 19, 28: None, 29: 24
        }

        for l21_idx, pose_idx in mapping.items():
            if pose_idx is not None:
                l21_pose[l21_idx] = pose[pose_idx]

        return l21_pose

    def state_to_cmd(self, l21_state):
        pose = [0.0] * 25

        mapping = {
            0: 10,  1: 5,   2: 0,   3: 15,  5: 20,  7: 6,
            8: 1,   9: 16,  11: 21, 13:7, 14: 2,  15: 17, 17: 22,
            19: 8,  20: 3,  21: 18, 23: 23, 25: 9,   26: 4,
            27: 19, 29: 24
        }
        for l21_idx, pose_idx in mapping.items():
            pose[pose_idx] = l21_state[l21_idx]
        return pose
    def action_play(self):
        self.send_command(0xA0,[])
    def get_current_status(self, j=''):
        self.send_command(FrameProperty.THUMB_POS, j,sleep_time=0.001)
        self.send_command(FrameProperty.INDEX_POS,j,sleep_time=0.001)
        self.send_command(FrameProperty.MIDDLE_POS,j,sleep_time=0.001)
        self.send_command(FrameProperty.RING_POS,j,sleep_time=0.001)
        self.send_command(FrameProperty.LITTLE_POS, j,sleep_time=0.001)
        state= self.x41+ self.x42+ self.x43+ self.x44+ self.x45
        if len(state) == 30:
            l21_state = self.state_to_cmd(l21_state=state)
            return l21_state
        
    def get_current_pub_status(self):
        state= self.x41+ self.x42+ self.x43+ self.x44+ self.x45
        if len(state) == 30:
            l21_state = self.state_to_cmd(l21_state=state)
            return l21_state
        
    def get_current_state_topic(self):
        self.send_command(0x01,[])
        self.send_command(0x02,[])
        self.send_command(0x03,[])
        self.send_command(0x04,[])
        self.send_command(0x06,[])
        state = self.x03+self.x02+self.x01+self.x04+self.x06
        return state
    
    def get_speed(self,j=''):
        self.send_command(FrameProperty.THUMB_SPEED, j)
        self.send_command(FrameProperty.INDEX_SPEED, j)
        self.send_command(FrameProperty.MIDDLE_SPEED, j)
        self.send_command(FrameProperty.RING_SPEED, j)
        self.send_command(FrameProperty.LITTLE_SPEED, j)
        speed = self.x49+ self.x4a+ self.x4b+ self.x4c+ self.x4d
        if len(speed) == 30:
            l21_speed = self.state_to_cmd(l21_state=speed)
            return l21_speed
    
    # def get_finger_torque(self):
    #     return self.finger_torque()
    def get_fault(self):
        self.get_thumbn_fault()
        self.get_index_fault()
        self.get_middle_fault()
        self.get_ring_fault()
        self.get_little_fault()
        return [self.x59]+[self.x5a]+[self.x5b]+[self.x5c]+[self.x5d]
    def get_threshold(self):
        self.get_thumb_threshold()
        self.get_index_threshold()
        self.get_middle_threshold()
        self.get_ring_threshold()
        self.get_little_threshold()
        return [self.x61]+[self.x62]+[self.x63]+[self.x64]+[self.x65]
    def get_version(self):
        if self.xc1 == []:
            self.send_command(FrameProperty.HAND_HARDWARE_VERSION,[])
        return self.xc1
    def get_normal_force(self):
        self.send_command(FrameProperty.HAND_NORMAL_FORCE,[])
        return self.x90
    def get_tangential_force(self):
        self.send_command(FrameProperty.HAND_TANGENTIAL_FORCE,[])
        return self.x91
    def get_tangential_force_dir(self):
        self.send_command(FrameProperty.HAND_TANGENTIAL_FORCE_DIR,[])
        return self.x92
    def get_approach_inc(self):
        self.send_command(FrameProperty.HAND_APPROACH_INC,[])
        return self.x93
    
    def get_touch_type(self):
        '''Get tactile sensor type data'''
        self.send_command(FrameProperty.TOUCH_SENSOR_TYPE,[])
        try:
            return self.xb0[0]
        except:
            pass
    def get_finger_torque(self):
        self.send_command(FrameProperty.THUMB_TORQUE,[])
        self.send_command(FrameProperty.INDEX_TORQUE,[])
        self.send_command(FrameProperty.MIDDLE_TORQUE,[])
        self.send_command(FrameProperty.RING_TORQUE,[])
        self.send_command(FrameProperty.LITTLE_TORQUE,[])
        return self.x51+self.x52+self.x53+self.x54+self.x55
    
    def get_torque(self):
        return self.get_finger_torque()
    
    def get_thumb_touch(self):
        '''Get thumb tactile sensor data'''
        self.send_command(FrameProperty.THUMB_TOUCH,[],sleep_time=0.015)
        return self.xb1
    
    def get_index_touch(self):
        '''Get index finger tactile sensor data'''
        self.send_command(FrameProperty.INDEX_TOUCH,[0xc6],sleep_time=0.015)
        return self.xb2
    
    def get_middle_touch(self):
        '''Get middle finger tactile sensor data'''
        self.send_command(FrameProperty.MIDDLE_TOUCH,[],sleep_time=0.015)
        return self.xb3
    
    def get_ring_touch(self):
        '''Get ring finger tactile sensor data'''
        self.send_command(FrameProperty.RING_TOUCH,[],sleep_time=0.015)
        return self.xb4
    
    def get_little_touch(self):
        '''Get little finger tactile sensor data'''
        self.send_command(FrameProperty.LITTLE_TOUCH,[],sleep_time=0.015)
        return self.xb5
    
    def get_palm_touch(self):
        '''Get palm tactile sensor data'''
        self.send_command(FrameProperty.PALM_TOUCH,[],sleep_time=0.015)
        return self.xb6
    
    def get_force(self):
        '''Get pressure sensor data'''
        return [self.x90,self.x91 , self.x92 , self.x93]
    
    def get_touch(self):
        '''Get tactile sensor data'''
        self.get_thumb_touch()
        self.get_index_touch()
        self.get_middle_touch()
        self.get_ring_touch()
        self.get_little_touch()
        self.get_palm_touch()
        try:
            return [self.xb1[1],self.xb2[1] , self.xb3[1] , self.xb4[1],self.xb5[1],self.xb6[1]]
        except:
            pass

    def get_matrix_touch(self):
        self.send_command(0xb1,[0xc6],sleep_time=0.04)
        self.send_command(0xb2,[0xc6],sleep_time=0.04)
        self.send_command(0xb3,[0xc6],sleep_time=0.04)
        self.send_command(0xb4,[0xc6],sleep_time=0.04)
        self.send_command(0xb5,[0xc6],sleep_time=0.04)
        return self.thumb_matrix , self.index_matrix , self.middle_matrix , self.ring_matrix , self.little_matrix

    def get_current(self):
        '''Not supported yet'''
        return [0] * 21
    def get_temperature(self):
        self.get_thumb_threshold()
        self.get_index_threshold()
        self.get_middle_threshold()
        self.get_ring_threshold()
        self.get_little_threshold()
        return self.x61+self.x62+self.x63+self.x64+self.x65
    
    def get_serial_number(self):
        return [0] * 6

    def get_finger_order(self):
        return [
            "thumb_root",
            "index_finger_root",
            "middle_finger_root",
            "ring_finger_root",
            "little_finger_root",
            "thumb_abduction",
            "index_finger_abduction",
            "middle_finger_abduction",
            "ring_finger_abduction",
            "little_finger_abduction",
            "thumb_roll",
            "reserved",
            "reserved",
            "reserved",
            "reserved",
            "thumb_middle_joint",
            "reserved",
            "reserved",
            "reserved",
            "reserved",
            "thumb_tip",
            "index_finger_tip",
            "middle_finger_tip",
            "ring_finger_tip",
            "little_finger_tip"
        ]

    def clear_faults(self):
        '''Clear motor faults'''
        self.send_command(0x83, [1, 1, 1, 1, 1],sleep_time=0.003)
        return self.x83

    def close_can_interface(self):
        if self.bus:
            self.bus.shutdown()  # Close CAN bus
