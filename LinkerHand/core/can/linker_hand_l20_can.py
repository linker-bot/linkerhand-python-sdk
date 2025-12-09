import sys
import time
import can
import threading
from enum import Enum
import numpy as np
from utils.open_can import OpenCan

class FrameProperty(Enum):
    INVALID_FRAME_PROPERTY = 0x00  # Invalid CAN frame property | No return
    JOINT_PITCH_R = 0x01           # Short frame pitch angle - finger base flexion | Returns this type of data
    JOINT_YAW_R = 0x02             # Short frame yaw angle - finger abduction/adduction | Returns this type of data
    JOINT_ROLL_R = 0x03            # Short frame roll angle - only used for thumb | Returns this type of data
    JOINT_TIP_R = 0x04             # Short frame fingertip angle control | Returns this type of data
    JOINT_SPEED_R = 0x05           # Short frame speed - motor running speed control | Returns this type of data
    JOINT_CURRENT_R = 0x06         # Short frame current - motor running current feedback | Returns this type of data
    JOINT_FAULT_R = 0x07           # Short frame fault - motor running fault feedback | Returns this type of data
    REQUEST_DATA_RETURN = 0x09     # Request data return | Returns all data
    JOINT_PITCH_NR = 0x11          # Pitch angle - finger base flexion | No return for this type of data
    JOINT_YAW_NR = 0x12            # Yaw angle - finger abduction/adduction | No return for this type of data
    JOINT_ROLL_NR = 0x13           # Roll angle - only used for thumb | No return for this type of data
    JOINT_TIP_NR = 0x14            # Fingertip angle control | No return for this type of data
    JOINT_SPEED_NR = 0x15          # Speed - motor running speed control | No return for this type of data
    JOINT_CURRENT_NR = 0x16        # Current - motor running current feedback | No return for this type of data
    JOINT_FAULT_NR = 0x17          # Fault - motor running fault feedback | No return for this type of data
    HAND_UID = 0xC0                # Device unique identifier Read only --------
    HAND_HARDWARE_VERSION = 0xC1   # Hardware version Read only --------
    HAND_SOFTWARE_VERSION = 0xC2   # Software version Read only --------
    HAND_COMM_ID = 0xC3            # Device ID Read/Write 1 byte
    HAND_SAVE_PARAMETER = 0xCF     # Save parameters Write only --------


class LinkerHandL20Can:
    def __init__(self, can_channel='can0', baudrate=1000000, can_id=0x28,yaml=""):
        self.can_id = can_id
        self.can_channel = can_channel
        self.baudrate = baudrate
        self.open_can = OpenCan(load_yaml=yaml)

        self.running = True
        self.x05 = [255] * 5
        self.x06, self.x07 = [],[]
        # New pressure sensors
        self.xb0,self.xb1,self.xb2,self.xb3,self.xb4,self.xb5 = [-1] * 5,[-1] * 5,[-1] * 5,[-1] * 5,[-1] * 5,[-1] * 5
        self.x09 = self.x0b = self.x0c = self.x0d = [-1] * 5
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
        try:
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
        except:
            print("Please insert CAN device",flush=True)

        # Initialize data storage
        self.x01, self.x02, self.x03, self.x04 = [[-1] * 5 for _ in range(4)]
        self.normal_force, self.tangential_force, self.tangential_force_dir, self.approach_inc = \
            [[-1] * 5 for _ in range(4)]

        # Start receive thread
        self.get_touch_type()
        time.sleep(0.1)
        self.receive_thread = threading.Thread(target=self.receive_response)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    # def send_command(self, frame_property, data_list):
    #     print("66666")
    #     """
    #     Send command to CAN bus
    #     :param frame_property: Data frame property
    #     :param data_list: Data payload
    #     """
    #     frame_property_value = int(frame_property.value) if hasattr(frame_property, 'value') else frame_property
    #     data = [frame_property_value] + [int(val) for val in data_list]
    #     msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False)
    #     try:
    #         self.bus.send(msg)
    #         print(f"Message sent: ID={hex(self.can_id)}, Data={data}")
    #     except can.CanError as e:
    #         print(f"Failed to send message: {e}")

    def receive_response(self):
        """
        Receive and process CAN bus response messages
        """
        while self.running:
            try:
                msg = self.bus.recv(timeout=1.0)  # Blocking receive, 1 second timeout
                if msg:
                    self.process_response(msg)
            except can.CanError as e:
                print(f"Error receiving message666: {e}",flush=True)
                

    def set_finger_base(self, angles):
        self.send_command(FrameProperty.JOINT_PITCH_NR, angles)

    def set_finger_tip(self, angles):
        self.send_command(FrameProperty.JOINT_TIP_NR, angles)

    def set_finger_middle(self, angles):
        self.send_command(FrameProperty.JOINT_YAW_NR, angles)

    def set_thumb_roll(self, angle):
        self.send_command(FrameProperty.JOINT_ROLL_NR, angle)

    def send_command(self, frame_property, data_list,sleep=0.002):
        frame_property_value = int(frame_property.value) if hasattr(frame_property, 'value') else frame_property
        data = [frame_property_value] + [int(val) for val in data_list]
        
        msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError:
            print("Message NOT sent")
            self.open_can.open_can(self.can_channel)
            time.sleep(1)
            self.is_can = self.open_can.is_can_up_sysfs(interface=self.can_channel)
            time.sleep(1)
            if self.is_can:
                self.bus = can.interface.Bus(channel=self.can_channel, interface="socketcan", bitrate=self.baudrate)
            else:
                print("Reconnecting CAN devices ....",flush=True)
        time.sleep(sleep)

    def set_joint_pitch(self, frame, angles):
        self.send_command(frame, angles)

    def set_joint_yaw(self, angles):
        self.send_command(0x02, angles)

    def set_joint_roll(self, thumb_roll):
        self.send_command(0x03, [thumb_roll, 0, 0, 0, 0])

    def set_joint_speed(self, speed):
        self.x05 = speed
        self.send_command(0x05, speed)
    def set_electric_current(self, e_c=[]):
        self.send_command(0x06, e_c)

    def get_normal_force(self):
        self.send_command(0x20,[])

    def get_tangential_force(self):
        self.send_command(0x21,[])


    def get_tangential_force_dir(self):
        self.send_command(0x22,[])

    def get_approach_inc(self):
        self.send_command(0x23,[])




    def get_electric_current(self, e_c=[]):
        self.send_command(0x06, e_c)
    
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
            elif frame_type == 0xC0:
                print(f"Device ID info: {response_data}")
                if self.can_id == 0x28:
                    self.right_hand_info = response_data
                elif self.can_id == 0x27:
                    self.left_hand_info = response_data
            elif frame_type == 0x05:
                self.x05 = list(response_data)
            elif frame_type == 0x06:
                self.x06 = list(response_data)
            elif frame_type == 0x07:
                self.x07 = list(response_data)
            elif frame_type == 0x09:
                self.x09 = list(response_data)
            elif frame_type == 0x0B:
                self.x0b = list(response_data)
            elif frame_type == 0x0C:
                self.x0c = list(response_data)
            elif frame_type == 0x0D:
                self.x0d = list(response_data)
            elif frame_type == 0x20:
                d = list(response_data)
                self.normal_force = [float(i) for i in d] 
            elif frame_type == 0x21:
                d = list(response_data)
                self.tangential_force = [float(i) for i in d]
            elif frame_type == 0x22:
                d = list(response_data)
                self.tangential_force_dir = [float(i) for i in d]
            elif frame_type == 0x23:
                d = list(response_data)
                self.approach_inc = [float(i) for i in d]
            elif frame_type == 0xb0:
                self.xb0 = list(response_data)
            elif frame_type == 0xb1:
                d = list(response_data)
                if len(d) == 2:
                    self.xb1 = d
                elif len(d) == 7:
                    index = self.matrix_map.get(d[0])
                    if index is not None:
                        self.thumb_matrix[index] = d[1:]  # Remove the first flag bit
            elif frame_type == 0xb2:
                d = list(response_data)
                if len(d) == 2:
                    self.xb2 = d
                elif len(d) == 7:
                    index = self.matrix_map.get(d[0])
                    if index is not None:
                        self.index_matrix[index] = d[1:]  # Remove the first flag bit
            elif frame_type == 0xb3:
                d = list(response_data)
                if len(d) == 2:
                    self.xb3 = d
                elif len(d) == 7:
                    index = self.matrix_map.get(d[0])
                    if index is not None:
                        self.middle_matrix[index] = d[1:]  # Remove the first flag bit
            elif frame_type == 0xb4:
                d = list(response_data)
                if len(d) == 2:
                    self.xb4 = d
                elif len(d) == 7:
                    index = self.matrix_map.get(d[0])
                    if index is not None:
                        self.ring_matrix[index] = d[1:]  # Remove the first flag bit
            elif frame_type == 0xb5:
                d = list(response_data)
                if len(d) == 2:
                    self.xb5 = d
                elif len(d) == 7:
                    index = self.matrix_map.get(d[0])
                    if index is not None:
                        self.little_matrix[index] = d[1:]  # Remove the first flag bit
    def pose_slice(self, p):
        """Slice the joint array into finger action arrays"""
        try:
            finger_base = [int(val) for val in p[0:5]]   # Finger base
            yaw_angles = [int(val) for val in p[5:10]]    # Yaw
            thumb_yaw = [int(val) for val in p[10:15]]     # Thumb yaw to palm, others are 0
            finger_tip = [int(val) for val in p[15:20]]    # Fingertip flexion
            return finger_base, yaw_angles, thumb_yaw, finger_tip
        except Exception as e:
            print(e)
    def set_joint_positions(self, position):
        if len(position) != 20:
            print("L20 finger joint length is incorrect")
            return
        finger_base, yaw_angles, thumb_yaw, finger_tip = self.pose_slice(position)
        self.set_thumb_roll(thumb_yaw) # Thumb yaw to palm movement
        self.set_finger_tip(finger_tip) # Fingertip movement
        self.set_finger_base(finger_base) # Finger base movement
        self.set_finger_middle(yaw_angles) # Yaw movement
    def set_speed(self, speed=[]):
        if len(speed) != 5:
            raise ValueError("Speed list must have 5 elements.")
            return
        self.send_command(0x05,speed)
    def set_torque(self, torque=[]):
        '''Set torque, not supported for L20'''
        print("Set torque, not supported for L20")
    def set_current(self, current=[]):
        '''Set current'''
        self.set_electric_current(e_c=current)
    def get_version(self):
        '''Get version, currently not supported'''
        return [0] * 5
    def get_current_status(self):
        '''Get current finger joint status'''
        self.send_command(0x01,[],sleep=0.01)
        self.send_command(0x02,[],sleep=0.01)
        self.send_command(0x03,[],sleep=0.01)
        self.send_command(0x04,[],sleep=0.01)
        return self.x01 + self.x02 + self.x03 + self.x04
    
    def get_current_pub_status(self):
        time.sleep(0.01)
        return self.x01 + self.x02 + self.x03 + self.x04

    def get_speed(self):
        '''Get current motor speed'''
        self.send_command(0x05, [0])
        time.sleep(0.001)
        return self.x05
    def get_current(self):
        '''Get current threshold'''
        self.send_command(0x06, [0])
        return self.x06
    def get_torque(self):
        '''Get current motor torque, not supported for L20'''
        return [0] * 5
    def get_fault(self):
        self.send_command(0x07,[])
        time.sleep(0.01)
        return self.x07
    
    def get_temperature(self):
        '''Get motor temperature'''
        self.send_command(0x09,[])
        self.send_command(0x0b,[])
        self.send_command(0x0c,[])
        self.send_command(0x0d,[])

        return self.x09+self.x0b+self.x0c+self.x0d
    def clear_faults(self):
        '''Clear motor faults'''
        self.send_command(0x07, [1, 1, 1, 1, 1])

    def get_touch_type(self):
        '''Get touch type'''
        t = []
        for i in range(3):
            self.send_command(0xb0,[],sleep=0.03)
        if self.xb0 == [2]:
            return 2
        elif self.xb0 == [1]:
            return 1
        else:
            self.send_command(0x20,[],sleep=0.03)
            time.sleep(0.01)
            if self.normal_force[0] == -1:
                return -1
    
    def get_touch(self):
        '''Get touch data'''
        self.send_command(0xb1,[],sleep=0.03)
        self.send_command(0xb2,[],sleep=0.03)
        self.send_command(0xb3,[],sleep=0.03)
        self.send_command(0xb4,[],sleep=0.03)
        self.send_command(0xb5,[],sleep=0.03)
        return [self.xb1[1],self.xb2[1],self.xb3[1],self.xb4[1],self.xb5[1],0] # The last digit is palm, currently not available

    def get_matrix_touch(self):
        self.send_command(0xb1,[0xc6],sleep=0.04)
        self.send_command(0xb2,[0xc6],sleep=0.04)
        self.send_command(0xb3,[0xc6],sleep=0.04)
        self.send_command(0xb4,[0xc6],sleep=0.04)
        self.send_command(0xb5,[0xc6],sleep=0.04)
        return self.thumb_matrix , self.index_matrix , self.middle_matrix , self.ring_matrix , self.little_matrix


    def get_thumb_matrix_touch(self,sleep_time=0.009):
        self.send_command(0xb1,[0xc6],sleep=sleep_time)
        return self.thumb_matrix
    
    def get_index_matrix_touch(self,sleep_time=0.009):
        self.send_command(0xb2,[0xc6],sleep=sleep_time)
        return self.index_matrix
    
    def get_middle_matrix_touch(self,sleep_time=0.009):
        self.send_command(0xb3,[0xc6],sleep=sleep_time)
        return self.middle_matrix
    
    def get_ring_matrix_touch(self,sleep_time=0.009):
        self.send_command(0xb4,[0xc6],sleep=sleep_time)
        return self.ring_matrix
    
    def get_little_matrix_touch(self,sleep_time=0.009):
        self.send_command(0xb5,[0xc6],sleep=sleep_time)
        return self.little_matrix


    def get_faults(self):
        '''Get motor fault codes'''
        self.send_command(0x07, [])
        return self.x07
    def get_force(self):
        '''Get pressure sensor data'''
        return [self.normal_force,self.tangential_force,self.tangential_force_dir,self.approach_inc]
    
    def get_serial_number(self):
        return [0] * 6

    def show_fun_table(self):
        pass
    
    def get_finger_order(self):
        return []
    
    def close_can_interface(self):
        if self.bus:
            self.bus.shutdown()  # Close CAN bus
