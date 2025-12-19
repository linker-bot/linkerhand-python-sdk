#!/usr/bin/env python3
import can
import time,sys,os
import threading
import numpy as np
from enum import Enum
from utils.open_can import OpenCan
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, ".."))
sys.path.append(target_dir)

class FrameProperty(Enum):
    INVALID_FRAME_PROPERTY = 0x00  # Invalid CAN frame property | No response
    # Parallel command area
    ROLL_POS = 0x01  # Roll joint position | The coordinate system is built at the root of each finger, and the rotation angle is defined according to the straightened state of the finger [10,11,12,13,14]
    YAW_POS = 0x02  # Yaw joint position | The coordinate system is built at the root of each finger, and the rotation angle is defined according to the straightened state of the finger [5,6,7,8,9]
    ROOT1_POS = 0x03  # Root1 joint position | The root joint closest to the palm [0,1,2,3,4]
    ROOT2_POS = 0x04  # Root2 joint position | The root joint closest to the palm [15, 16,17,18,19]
    ROOT3_POS = 0x05  # Root3 joint position | The root joint closest to the palm Not available
    TIP_POS = 0x06  # Fingertip joint position | The root joint closest to the palm [20,21,22,23,24]

    ROLL_SPEED = 0x09  # Roll joint speed | The coordinate system is built at the root of each finger, and the rotation angle is defined according to the straightened state of the finger
    YAW_SPEED = 0x0A  # Yaw joint speed | The coordinate system is built at the root of each finger, and the rotation angle is defined according to the straightened state of the finger
    ROOT1_SPEED = 0x0B  # Root1 joint speed | The root joint closest to the palm
    ROOT2_SPEED = 0x0C  # Root2 joint speed | The root joint closest to the palm
    ROOT3_SPEED = 0x0D  # Root3 joint speed | The root joint closest to the palm
    TIP_SPEED = 0x0E  # Fingertip joint speed | The root joint closest to the palm

    ROLL_TORQUE = 0x11  # Roll joint torque | The coordinate system is built at the root of each finger, and the rotation angle is defined according to the straightened state of the finger
    YAW_TORQUE = 0x12  # Yaw joint torque | The coordinate system is built at the root of each finger, and the rotation angle is defined according to the straightened state of the finger
    ROOT1_TORQUE = 0x13  # Root1 joint torque | The root joint closest to the palm
    ROOT2_TORQUE = 0x14  # Root2 joint torque | The root joint closest to the palm
    ROOT3_TORQUE = 0x15  # Root3 joint torque | The root joint closest to the palm
    TIP_TORQUE = 0x16  # Fingertip joint torque | The root joint closest to the palm

    ROLL_FAULT = 0x19  # Roll joint fault code | The coordinate system is built at the root of each finger, and the rotation angle is defined according to the straightened state of the finger
    YAW_FAULT = 0x1A  # Yaw joint fault code | The coordinate system is built at the root of each finger, and the rotation angle is defined according to the straightened state of the finger
    ROOT1_FAULT = 0x1B  # Root1 joint fault code | The root joint closest to the palm
    ROOT2_FAULT = 0x1C  # Root2 joint fault code | The root joint closest to the palm
    ROOT3_FAULT = 0x1D  # Root3 joint fault code | The root joint closest to the palm
    TIP_FAULT = 0x1E  # Fingertip joint fault code | The root joint closest to the palm

    ROLL_TEMPERATURE = 0x21  # Roll joint temperature | The coordinate system is built at the root of each finger, and the rotation angle is defined according to the straightened state of the finger
    YAW_TEMPERATURE = 0x22  # Yaw joint temperature | The coordinate system is built at the root of each finger, and the rotation angle is defined according to the straightened state of the finger
    ROOT1_TEMPERATURE = 0x23  # Root1 joint temperature | The root joint closest to the palm
    ROOT2_TEMPERATURE = 0x24  # Root2 joint temperature | The root joint closest to the palm
    ROOT3_TEMPERATURE = 0x25  # Root3 joint temperature | The root joint closest to the palm
    TIP_TEMPERATURE = 0x26  # Fingertip joint temperature | The root joint closest to the palm
    # Parallel command area

    # Serial command area
    THUMB_POS = 0x41  # Thumb joint position | Returns this type of data
    INDEX_POS = 0x42  # Index finger joint position | Returns this type of data
    MIDDLE_POS = 0x43  # Middle finger joint position | Returns this type of data
    RING_POS = 0x44  # Ring finger joint position | Returns this type of data
    LITTLE_POS = 0x45  # Little finger joint position | Returns this type of data

    THUMB_SPEED = 0x49  # Thumb speed | Returns this type of data
    INDEX_SPEED = 0x4A  # Index finger speed | Returns this type of data
    MIDDLE_SPEED = 0x4B  # Middle finger speed | Returns this type of data
    RING_SPEED = 0x4C  # Ring finger speed | Returns this type of data
    LITTLE_SPEED = 0x4D  # Little finger speed | Returns this type of data

    THUMB_TORQUE = 0x51  # Thumb torque | Returns this type of data
    INDEX_TORQUE = 0x52  # Index finger torque | Returns this type of data
    MIDDLE_TORQUE = 0x53  # Middle finger torque | Returns this type of data
    RING_TORQUE = 0x54  # Ring finger torque | Returns this type of data
    LITTLE_TORQUE = 0x55  # Little finger torque | Returns this type of data

    THUMB_FAULT = 0x59  # Thumb fault code | Returns this type of data
    INDEX_FAULT = 0x5A  # Index finger fault code | Returns this type of data
    MIDDLE_FAULT = 0x5B  # Middle finger fault code | Returns this type of data
    RING_FAULT = 0x5C  # Ring finger fault code | Returns this type of data
    LITTLE_FAULT = 0x5D  # Little finger fault code | Returns this type of data

    THUMB_TEMPERATURE = 0x61  # Thumb temperature | Returns this type of data
    INDEX_TEMPERATURE = 0x62  # Index finger temperature | Returns this type of data
    MIDDLE_TEMPERATURE = 0x63  # Middle finger temperature | Returns this type of data
    RING_TEMPERATURE = 0x64  # Ring finger temperature | Returns this type of data
    LITTLE_TEMPERATURE = 0x65  # Little finger temperature | Returns this type of data
    # Serial command area

    # Merged command area, non-essential single control data of the same finger is merged
    FINGER_SPEED = 0x81  # Finger speed | Returns this type of data
    FINGER_TORQUE = 0x82  # Torque | Returns this type of data
    FINGER_FAULT = 0x83  # Finger fault code | Returns this type of data

    # Fingertip sensor data group
    HAND_NORMAL_FORCE = 0x90  # Normal force of five fingers
    HAND_TANGENTIAL_FORCE = 0x91  # Tangential force of five fingers
    HAND_TANGENTIAL_FORCE_DIR = 0x92  # Tangential direction of five fingers
    HAND_APPROACH_INC = 0x93  # Proximity sensing of five fingers

    THUMB_ALL_DATA = 0x98  # All data of thumb
    INDEX_ALL_DATA = 0x99  # All data of index finger
    MIDDLE_ALL_DATA = 0x9A  # All data of middle finger
    RING_ALL_DATA = 0x9B  # All data of ring finger
    LITTLE_ALL_DATA = 0x9C  # All data of little finger
    # Action command ·ACTION
    ACTION_PLAY = 0xA0  # Action

    # Configuration command ·CONFIG
    HAND_UID = 0xC0  # Device unique identifier
    HAND_HARDWARE_VERSION = 0xC1  # Hardware version
    HAND_SOFTWARE_VERSION = 0xC2  # Software version
    HAND_COMM_ID = 0xC3  # Device id
    HAND_FACTORY_RESET = 0xCE  # Restore factory settings
    HAND_SAVE_PARAMETER = 0xCF  # Save parameters

    WHOLE_FRAME = 0xF0  # Whole frame transmission | Returns one byte frame property + the entire structure for 485 and network transmission only

class LinkerHandL25Can:
    def __init__(self, can_channel='can0', baudrate=1000000, can_id=0x28,yaml=""):
        self.can_id = can_id
        self.can_channel = can_channel
        self.baudrate = baudrate
        self.open_can = OpenCan(load_yaml=yaml)

        self.running = True
        self.last_thumb_pos, self.last_index_pos,self.last_ring_pos,self.last_middle_pos, self.last_little_pos = None,None,None,None,None
        self.x01, self.x02, self.x03, self.x04,self.x05,self.x06,self.x07, self.x08,self.x09,self.x0A,self.x0B,self.x0C,self.x0D,self.x0E,self.speed = [],[],[],[],[],[],[],[],[],[],[],[],[],[],[]
        self.last_root1,self.last_yaw,self.last_roll,self.last_root2,self.last_tip = None,None,None,None,None
        # 速度
        self.x49, self.x4a, self.x4b, self.x4c, self.x4d,self.xc1 = [],[],[],[],[],[]
        self.x41,self.x42,self.x43,self.x44,self.x45 = [],[],[],[],[]
        # 扭矩
        self.x51, self.x52, self.x53, self.x54,self.x55 = [],[],[],[],[]
        # 故障码
        self.x59,self.x5a,self.x5b,self.x5c,self.x5d = [],[],[],[],[]
        # 温度阈值
        self.x61,self.x62,self.x63,self.x64,self.x65 = [],[],[],[],[]
        # 压感
        self.x90,self.x91,self.x92,self.x93 = [],[],[],[]
        # 新压感
        self.xb0,self.xb1,self.xb2,self.xb3,self.xb4,self.xb5 = [-1] * 5,[-1] * 5,[-1] * 5,[-1] * 5,[-1] * 5,[-1] * 5
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
        # 根据操作系统初始化 CAN 总线
        try:
            if sys.platform == "linux":
                self.open_can.open_can(self.can_channel)
                time.sleep(0.1)
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
        except:
            print("Please insert CAN device")

        # 启动接收线程
        self.receive_thread = threading.Thread(target=self.receive_response)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def send_command(self, frame_property, data_list):
        """
        Send command to CAN bus
        :param frame_property: Data frame properties
        :param data_list: Data payload
        """
        frame_property_value = int(frame_property.value) if hasattr(frame_property, 'value') else frame_property
        data = [frame_property_value] + [int(val) for val in data_list]
        msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            #print(f"Message sent: ID={hex(self.can_id)}, Data={data}")
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
        time.sleep(0.001)

    def receive_response(self):
        """
        Receive and process response messages from CAN bus
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
            l25_pose = self.joint_map(joint_ranges)
            # 使用列表推导式将列表每6个元素切成一个子数组
            chunks = [l25_pose[i:i+6] for i in range(0, 30, 6)]
            self.send_command(FrameProperty.THUMB_POS, chunks[0])
            #time.sleep(0.001)
            self.send_command(FrameProperty.INDEX_POS, chunks[1])
            #time.sleep(0.001)
            self.send_command(FrameProperty.MIDDLE_POS, chunks[2])
            #time.sleep(0.001)
            self.send_command(FrameProperty.RING_POS, chunks[3])
            #time.sleep(0.001)
            self.send_command(FrameProperty.LITTLE_POS, chunks[4])
            #time.sleep(0.001)

    def set_joint_positions_by_topic(self, joint_ranges):
        if len(joint_ranges) == 25:
            # Finger Joint Position Constants
            #ROLL_POS = 0x01  # Roll joint position | Coordinate system based on finger base, rotation angle defined when finger is straight [10,11,12,13,14]
            #YAW_POS = 0x02   # Yaw joint position | Coordinate system based on finger base, rotation angle defined when finger is straight [5,6,7,8,9]
            #ROOT1_POS = 0x03 # Root1 joint position | Joint closest to the palm [0,1,2,3,4]
            #ROOT2_POS = 0x04 # Root2 joint position | Joint closest to the palm [15,16,17,18,19]
            #ROOT3_POS = 0x05 # Root3 joint position | Joint closest to the palm (currently unused)
            #TIP_POS = 0x06   # Tip joint position | Joint closest to the palm [20,21,22,23,24]

            # Finger joint names mapping (Chinese to English translation):
            # ["Thumb root", "Index root", "Middle root", "Ring root", "Pinky root",
            #  "Thumb yaw", "Index yaw", "Middle yaw", "Ring yaw", "Pinky yaw",
            #  "Thumb roll", "Reserved", "Reserved", "Reserved", "Reserved",
            #  "Thumb middle", "Index middle", "Middle middle", "Ring middle", "Pinky middle",
            #  "Thumb tip", "Index tip", "Middle tip", "Ring tip", "Pinky tip"]

            
            l25_pose = self.slice_list(joint_ranges,5)
            if self._list_d_value(self.last_root1, l25_pose[0]):
                self.set_root1_positions(l25_pose[0])
                self.last_root1 = l25_pose[0]
            if self._list_d_value(self.last_yaw, l25_pose[1]):  
                self.set_yaw_positions(l25_pose[1])
                self.last_yaw = l25_pose[1]
            if self._list_d_value(self.last_roll, l25_pose[2]):
                self.set_roll_positions(l25_pose[2])
                self.last_roll = l25_pose[2]
            if self._list_d_value(self.last_root2, l25_pose[3]):
                self.set_root2_positions(l25_pose[3])
                self.last_root2 = l25_pose[3]
            if self._list_d_value(self.last_tip, l25_pose[4]): 
                self.set_tip_positions(l25_pose[4])
                self.last_tip = l25_pose[4]
            

    def slice_list(self, input_list, slice_size):
        """
        Split a list into chunks of specified size.

        Parameters:
            input_list (list): The list to be chunked.
            slice_size (int): Number of elements in each chunk.

        Returns:
            list of lists: The chunked list.
        """
        # Implementation using list comprehension
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
    # Set roll joint positions for all fingers
    def set_roll_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROLL_POS, joint_ranges)
    # Set yaw joint positions for all fingers
    def set_yaw_positions(self, joint_ranges):
        print(joint_ranges)
        self.send_command(FrameProperty.YAW_POS, joint_ranges)
    # Set base joint 1 positions for all fingers
    def set_root1_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT1_POS, joint_ranges)
    # Set base joint 2 positions for all fingers
    def set_root2_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT2_POS, joint_ranges)
    # Set base joint 3 positions for all fingers
    def set_root3_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT3_POS, joint_ranges)
    # Set fingertip joint positions for all fingers
    def set_tip_positions(self, joint_ranges=[80]*5):
        self.send_command(FrameProperty.TIP_POS, joint_ranges)
    # Set thumb torque parameters
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

    # Get thumb joint position
    def get_thumb_positions(self,j=[0]):
        self.send_command(FrameProperty.THUMB_POS, j)
    # Get index finger joint positions
    def get_index_positions(self, j=[0]):
        self.send_command(FrameProperty.INDEX_POS,j)
    # Get middle finger joint position
    def get_middle_positions(self, j=[0]):
        self.send_command(FrameProperty.MIDDLE_POS,j)
    # Retrieve the position of the ring finger joint
    def get_ring_positions(self, j=[0]):
        self.send_command(FrameProperty.RING_POS,j)
    # Retrieve the position of the little finger joint
    def get_little_positions(self, j=[0]):
        self.send_command(FrameProperty.LITTLE_POS, j)
    # All fault codes of motors in the thumb
    def get_thumbn_fault(self,j=[]):
        self.send_command(FrameProperty.THUMB_FAULT,j)
    # All motor fault codes for the index finger
    def get_index_fault(self,j=[]):
        self.send_command(FrameProperty.INDEX_FAULT,j)
    # All motor fault codes for the middle finger
    def get_middle_fault(self,j=[]):
        self.send_command(FrameProperty.MIDDLE_FAULT,j)
    # All motor fault codes for the ring finger
    def get_ring_fault(self,j=[]):
        self.send_command(FrameProperty.RING_FAULT,j)
    # All motor fault codes for the little finger
    def get_little_fault(self,j=[]):
        self.send_command(FrameProperty.LITTLE_FAULT,j)
    # Temperature threshold for the thumb motors
    def get_thumb_threshold(self,j=[]):
        self.send_command(FrameProperty.THUMB_TEMPERATURE, '')
    # Temperature threshold for the index finger motors
    def get_index_threshold(self,j=[]):
        self.send_command(FrameProperty.INDEX_TEMPERATURE, j)
    # Temperature threshold for the middle finger motors
    def get_middle_threshold(self,j=[]):
        self.send_command(FrameProperty.MIDDLE_TEMPERATURE, j)
    # Temperature threshold for the ring finger motors
    def get_ring_threshold(self,j=[]):
        self.send_command(FrameProperty.RING_TEMPERATURE, j)
    # Little finger temperature threshold
    def get_little_threshold(self,j=[]):
        self.send_command(FrameProperty.LITTLE_TEMPERATURE, j)


    def set_disability_mode(self, j=[1,1,1,1,1]):
        self.send_command(0x85,j)

    def set_enable_mode(self, j=[00,00,00,00,00]):
        self.send_command(0x85,j)
    
    # Set torque for all fingers
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


    def joint_map(self, pose):
        l25_pose = [0.0] * 30

        # 映射表，通过字典简化映射关系
        mapping = {
            0: 10,  1: 5,   2: 0,   3: 15,  4: None,  5: 20,
            6: None, 7: 6,   8: 1,   9: 16,  10: None, 11: 21,
            12: None, 13: 7, 14: 2,  15: 17, 16: None, 17: 22,
            18: None, 19: 8,  20: 3,   21: 18, 22: None, 23: 23,
            24: None, 25: 9,  26: 4,   27: 19, 28: None, 29: 24
        }

        # 遍历映射字典，进行值的映射
        for l25_idx, pose_idx in mapping.items():
            if pose_idx is not None:
                l25_pose[l25_idx] = pose[pose_idx]

        return l25_pose


    def state_to_cmd(self, l25_state):
        
        pose = [0.0] * 25
        mapping = {
            0: 10,  1: 5,   2: 0,   3: 15,  5: 20,  7: 6,
            8: 1,   9: 16,  11: 21, 13:7, 14: 2,  15: 17, 17: 22,
            19: 8,  20: 3,  21: 18, 23: 23, 25: 9,   26: 4,
            27: 19, 29: 24
        }
        # 遍历映射字典，更新pose的值
        for l25_idx, pose_idx in mapping.items():
            pose[pose_idx] = l25_state[l25_idx]
        return pose
    def action_play(self):
        self.send_command(0xA0,[])

    def get_current_status(self, j=''):
        self.send_command(FrameProperty.THUMB_POS, j)
        #time.sleep(0.001)
        self.send_command(FrameProperty.INDEX_POS,j)
        #time.sleep(0.001)
        self.send_command(FrameProperty.MIDDLE_POS,j)
        #time.sleep(0.001)
        self.send_command(FrameProperty.RING_POS,j)
        #time.sleep(0.001)
        self.send_command(FrameProperty.LITTLE_POS, j)
        #time.sleep(0.001)
        state= self.x41+ self.x42+ self.x43+ self.x44+ self.x45
        if len(state) == 30:
            l25_state = self.state_to_cmd(l25_state=state)
            return l25_state

    def get_current_pub_status(self):
        state= self.x41+ self.x42+ self.x43+ self.x44+ self.x45
        if len(state) == 30:
            l25_state = self.state_to_cmd(l25_state=state)
            return l25_state
        
    def get_current_state_topic(self):
        self.send_command(0x01,[])
        #time.sleep(0.001)
        self.send_command(0x02,[])
       # time.sleep(0.001)
        self.send_command(0x03,[])
        #time.sleep(0.001)
        self.send_command(0x04,[])
        #time.sleep(0.001)
        self.send_command(0x06,[])
        #time.sleep(0.001)
        state = self.x03+self.x02+self.x01+self.x04+self.x06
        return state
    def get_speed(self,j=''):
        self.send_command(FrameProperty.THUMB_SPEED, j)
        #time.sleep(0.01)
        self.send_command(FrameProperty.INDEX_SPEED, j)
        #time.sleep(0.01)
        self.send_command(FrameProperty.MIDDLE_SPEED, j)
        #time.sleep(0.01)
        self.send_command(FrameProperty.RING_SPEED, j)
        #time.sleep(0.01)
        self.send_command(FrameProperty.LITTLE_SPEED, j)
        #time.sleep(0.01)
        speed = self.x49+ self.x4a+ self.x4b+ self.x4c+ self.x4d
        if len(speed) == 30:
            l25_speed = self.state_to_cmd(l25_state=speed)
            return l25_speed
    
    def get_finger_torque(self):
        self.send_command(FrameProperty.THUMB_TORQUE,[])
        self.send_command(FrameProperty.INDEX_TORQUE,[])
        self.send_command(FrameProperty.MIDDLE_TORQUE,[])
        self.send_command(FrameProperty.RING_TORQUE,[])
        self.send_command(FrameProperty.LITTLE_TORQUE,[])
        return self.x51+self.x52+self.x53+self.x54+self.x55
    
    def get_torque(self):
        return self.get_finger_torque()
    def get_fault(self):
        self.get_thumbn_fault()
        #time.sleep(0.001)
        self.get_index_fault()
        #time.sleep(0.001)
        self.get_middle_fault()
        #time.sleep(0.001)
        self.get_ring_fault()
        #time.sleep(0.001)
        self.get_little_fault()
        #time.sleep(0.001)
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
    def get_force(self):
        '''获取压感数据'''
        return [self.x90,self.x91 , self.x92 , self.x93]
    
    def get_matrix_touch(self):
        self.send_command(0xb1,[0xc6])
        time.sleep(0.03)
        self.send_command(0xb2,[0xc6])
        time.sleep(0.03)
        self.send_command(0xb3,[0xc6])
        time.sleep(0.03)
        self.send_command(0xb4,[0xc6])
        time.sleep(0.03)
        self.send_command(0xb5,[0xc6])
        time.sleep(0.03)
        return self.thumb_matrix , self.index_matrix , self.middle_matrix , self.ring_matrix , self.little_matrix
    
    def get_touch_type(self):
        '''Get touch type'''
        self.send_command(0xb1,[])
        time.sleep(0.03)
        if len(self.xb1) == 2:
            return 2
        else:
            return -1
    
    def get_touch(self):
        '''Get touch data (not supported yet)'''
        return [-1] * 6


    def get_current(self):
        return [0] * 21
    def get_temperature(self):
        self.get_thumb_threshold()
        self.get_index_threshold()
        self.get_middle_threshold()
        self.get_ring_threshold()
        self.get_little_threshold()
        return [self.x61]+[self.x62]+[self.x63]+[self.x64]+[self.x65]
    
    def get_finger_order(self):
        return ["Thumb root", "Index root", "Middle root", "Ring root", "Little root",
            "Thumb abduction", "Index abduction", "Middle abduction", "Ring abduction", "Little abduction",
            "Thumb roll", "Reserved", "Reserved", "Reserved", "Reserved",
            "Thumb middle", "Index middle", "Middle middle", "Ring middle", "Little middle",
            "Thumb tip", "Index tip", "Middle tip", "Ring tip", "Little tip"]
    
    def close_can_interface(self):
        if self.bus:
            self.bus.shutdown()
    
    '''
    这个方法只用于展示数据关系映射，使用的话最好使用上面的方法
    '''
    def joint_map_2(self, pose):
        l25_pose = [0.0]*30 #L25 CAN默认接收30个数据 pose控制L25发送的指令数据默认25个，这里进行映射
        '''
        需要进行映射
        # L25 CAN数据格式
        #["拇指横摆0-10", "拇指侧摆1-5", "拇指根部2-0", "拇指中部3-15", "预留4-", "拇指指尖5-20", "预留6-", "食指侧摆7-6", "食指根部8-1", "食指中部9-16", "预留10-", "食指指尖11-21", "预留12-", "预留13-", "中指根部14-2", "中指中部15-17", "预留16-", "中指指尖17-22", "预留18-", "无名指侧摆19-8", "无名指根部20-3", "无名指中部21-18", "预留22-", "无名指指尖23-23", "预留24-", "小指侧摆25-9", "小指根部26-4", "小指中部27-19", "预留28-", "小指指尖29-24"]
        # CMD 接收到的数据格式
        #["拇指根部0", "食指根部1", "中指根部2", "无名指根部3","小指根部4","拇指侧摆5","食指侧摆6","中指侧摆","无名指侧摆8","小指侧摆9","拇指横摆10","预留","预留","预留","预留","拇指中部15","食指中部16","中指中部17","无名指中部18","小指中部19","拇指指尖20","食指指尖21","中指指尖22","无名指指尖23","小指指尖24"]
        '''
        l25_pose[0] = pose[10]
        l25_pose[1] = pose[5]
        l25_pose[2] = pose[0]
        l25_pose[3] = pose[15]
        l25_pose[4] = 0.0
        l25_pose[5] = pose[20]
        l25_pose[6] = 0.0
        l25_pose[7] = pose[6]
        l25_pose[8] = pose[1]
        l25_pose[9] = pose[16]
        l25_pose[10] = 0.0
        l25_pose[11] = pose[21]
        l25_pose[12] = 0.0
        l25_pose[13] = 0.0
        l25_pose[14] = pose[2]
        l25_pose[15] = pose[17]
        l25_pose[16] = 0.0
        l25_pose[17] = pose[22]
        l25_pose[18] = 0.0
        l25_pose[19] = pose[8]
        l25_pose[20] = pose[3]
        l25_pose[21] = pose[18]
        l25_pose[22] = 0.0
        l25_pose[23] = pose[23]
        l25_pose[24] = 0.0
        l25_pose[25] = pose[9]
        l25_pose[26] = pose[4]
        l25_pose[27] = pose[19]
        l25_pose[28] = 0.0
        l25_pose[29] = pose[24]
        return l25_pose
    
    def get_serial_number(self):
        return [0] * 6
    def show_fun_table(self):
        pass