#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
import can
import time,sys
import threading
import numpy as np
#from tabulate import tabulate
from enum import Enum
from utils.open_can import OpenCan
from utils.color_msg import ColorMsg



class FrameProperty(Enum):
    INVALID_FRAME_PROPERTY = 0x00
    JOINT_POSITION_RCO = 0x01
    MAX_PRESS_RCO = 0x02
    MAX_PRESS_RCO2 = 0x03
    JOINT_POSITION2_RCO = 0x04
    JOINT_SPEED = 0x05
    JOINT_SPEED2 = 0x06
    REQUEST_DATA_RETURN = 0x09
    JOINT_POSITION_N = 0x11
    MAX_PRESS_N = 0x12
    HAND_NORMAL_FORCE = 0X20
    HAND_TANGENTIAL_FORCE = 0X21
    HAND_TANGENTIAL_FORCE_DIR = 0X22
    HAND_APPROACH_INC = 0X23
    MOTOR_TEMPERATURE_1 = 0x33
    MOTOR_TEMPERATURE_2 = 0x34

class LinkerHandL10Can:
    def __init__(self,can_id, can_channel='can0', baudrate=1000000, yaml=""):
        self.can_id = can_id
        self.can_channel = can_channel
        self.baudrate = baudrate
        self.open_can = OpenCan(load_yaml=yaml)
        self.is_cmd = False
        self.x01 = [-1] * 5
        self.x02 = [-1] * 5
        self.x03 = [-1] * 5
        self.x04 = [-1] * 5
        self.x05 = [-1] * 5
        self.x06 = [-1] * 5
        self.x33 = self.x34 = [0] * 5
        # Fault codes
        self.x35,self.x36 = [0] * 5,[0] * 5
        # New pressure sensors
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
        self.serial_number = []
        self.serial_number_map = {
            0: 0,
            1: 1,
            2: 2,
            3: 3,
        }
        self.can_id = can_id
        self.joint_angles = [0] * 10
        self.pressures = [200] * 5  # Default torque 200
        self.bus = self.init_can_bus(can_channel, baudrate)
        self.normal_force, self.tangential_force, self.tangential_force_dir, self.approach_inc = [[-1] * 5 for _ in range(4)]
        self.version = None
        # Start receiving thread
        self.running = True
        self.receive_thread = threading.Thread(target=self.receive_response)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        self.version = self.get_version()

    def init_can_bus(self, channel, baudrate):
        try:
            if sys.platform == "linux":
                return can.interface.Bus(channel=channel, interface="socketcan", bitrate=baudrate)
            elif sys.platform == "win32":
                return can.interface.Bus(channel=channel, interface='pcan', bitrate=baudrate)
            else:
                raise EnvironmentError("Unsupported platform for CAN interface")
        except:
            #print("Please insert CAN device")
            ColorMsg(msg="Warning: Please insert CAN device", color="red")

    def send_frame(self, frame_property, data_list,sleep=0.003):
        """Send a single CAN frame with specified properties and data."""
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
            # time.sleep(1)
            # 
        time.sleep(sleep)

    def set_joint_positions(self, joint_angles):
        """Set the positions of 10 joints (joint_angles: list of 10 values)."""
        self.joint_angles = joint_angles
        self.is_cmd = True
        # Send angle control in frames, L10 protocol splits into first 6 and last 4
        self.send_frame(FrameProperty.JOINT_POSITION2_RCO, self.joint_angles[6:])
        time.sleep(0.001)
        self.send_frame(FrameProperty.JOINT_POSITION_RCO, self.joint_angles[:6])
        time.sleep(0.002)
        self.is_cmd = False
        

    def set_max_torque_limits(self, pressures,type="get"):
        """Set maximum torque limits"""
        if type == "get":
            self.pressures = [0.0]
        else:
            self.pressures = pressures[:5]
        #self.send_frame(FrameProperty.MAX_PRESS_RCO, self.pressures)
        
        
    def set_joint_speed_l10(self,speed=[180]*5):
        self.x05 = speed
        for i in range(2):
            time.sleep(0.01)
            self.send_frame(0x05, speed)
    def set_speed(self,speed=[180]*5):
        if len(speed) == 5:
            self.x05 = speed
            for i in range(2):
                time.sleep(0.01)
                self.send_frame(0x05, speed)
        elif len(speed) == 10:
            for i in range(2):
                time.sleep(0.01)
                self.send_frame(0x05, speed[:5])
                self.send_frame(0x06, speed[5:])
        else:
            raise ValueError("Speed list must have 10 elements.")
    def request_all_status(self):
        """Get all joint positions and pressures."""
        self.send_frame(FrameProperty.REQUEST_DATA_RETURN, [])
    ''' -------------------Pressure Sensors---------------------- '''
    def get_normal_force(self):
        self.send_frame(FrameProperty.HAND_NORMAL_FORCE,[],sleep=0.004)

    def get_tangential_force(self):
        self.send_frame(FrameProperty.HAND_TANGENTIAL_FORCE,[],sleep=0.004)

    def get_tangential_force_dir(self):
        self.send_frame(FrameProperty.HAND_TANGENTIAL_FORCE_DIR,[],sleep=0.004)
    def get_approach_inc(self):
        self.send_frame(FrameProperty.HAND_APPROACH_INC,[],sleep=0.004)
    ''' -------------------Motor Temperature---------------------- '''
    def get_motor_temperature(self):
        self.send_frame(FrameProperty.MOTOR_TEMPERATURE_1,[],sleep=0.01)
        self.send_frame(FrameProperty.MOTOR_TEMPERATURE_2,[],sleep=0.01)
    # Motor fault codes
    def get_motor_fault_code(self):
        self.send_frame(0x35,[],sleep=0.1)
        self.send_frame(0x36,[],sleep=0.1)
    def receive_response(self):
        """Receive CAN responses and process them."""
        while self.running:
            try:
                msg = self.bus.recv(timeout=1.0)
                if msg:
                    self.process_response(msg)
            except can.CanError as e:
                print(f"Error receiving CAN message: {e}")

    def process_response(self, msg):
        """Process received CAN messages."""
        if msg.arbitration_id == self.can_id:
            if len(msg.data) == 0:
                return
            frame_type = msg.data[0]
            response_data = msg.data[1:]
            if frame_type == FrameProperty.JOINT_POSITION_RCO.value:   # 0x01
                self.x01 = list(response_data)  
            elif frame_type == FrameProperty.MAX_PRESS_RCO.value:    # 0x02
                self.x02 = list(response_data)
            elif frame_type == FrameProperty.MAX_PRESS_RCO2.value:    # 0x03
                self.x03 = list(response_data)
            elif frame_type == FrameProperty.JOINT_POSITION2_RCO.value:    # 0x04
                self.x04 = list(response_data)
            elif frame_type == 0x05:
                self.x05 = list(response_data)
            elif frame_type == 0x06:
                self.x06 = list(response_data)
            elif frame_type == 0x20:
                # Five-finger normal force
                d = list(response_data)
                self.normal_force = [float(i) for i in d]
            elif frame_type == 0x21:
                # Five-finger tangential force
                d = list(response_data)
                self.tangential_force = [float(i) for i in d]
            elif frame_type == 0x22:
                # Five-finger tangential force direction
                d = list(response_data)
                self.tangential_force_dir = [float(i) for i in d]
            elif frame_type == 0x23:
                # Five-finger approach increment
                d = list(response_data)
                self.approach_inc = [float(i) for i in d]
            elif frame_type == 0x33:
                self.x33 = list(response_data)
            elif frame_type == 0x34:
                self.x34 = list(response_data)
            elif frame_type == 0x35:
                self.x35 = list(response_data)
            elif frame_type == 0x36:
                self.x36 = list(response_data)
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
            elif frame_type == 0x64:
                self.version =  list(response_data)
            elif frame_type == 0xC2: # version number
                self.version = list(response_data)
            elif frame_type == 0xC0:
                d = list(response_data)
                index = self.serial_number_map.get(d[0])
                if index is not None:
                    self.serial_number=self.serial_number + d[1:]
                else:
                    self.serial_number=self.serial_number + [-1] * 6

    def get_version(self):
        self.send_frame(0x64, [], sleep=0.1)
        time.sleep(0.1)
        if self.version is None:
            self.send_frame(0xC2, [], sleep=0.1)
            time.sleep(0.1)
        return self.version

    def set_torque(self,torque=[]):
        '''Set maximum torque'''
        if len(torque) == 5:
            self.send_frame(0x02, torque)
            time.sleep(0.002)
            self.send_frame(0x03,torque)
        elif len(torque) > 5:
            self.send_frame(0x02, torque[:5])
            time.sleep(0.002)
            self.send_frame(0x03,torque[5:])

    
    def get_current_status(self):
        '''Get current joint status'''
        if self.is_cmd == False:
            #if self.version != None and self.version[4] > 35:
            self.send_frame(0x01,[],sleep=0.003)
            self.send_frame(0x04,[],sleep=0.003)
            state = self.x01 + self.x04
            return state
        else:
            state = self.x01 + self.x04
            return state
        
    def get_current_pub_status(self):
        state = self.x01 + self.x04
        return state
        
    def get_speed(self):
        '''Get current speed'''
        self.send_frame(0x05,[],sleep=0.003)
        self.send_frame(0x06,[],sleep=0.003)
        return self.x05 + self.x06
        
    def get_force(self):
        '''Get pressure sensor data'''
        return [self.normal_force,self.tangential_force , self.tangential_force_dir , self.approach_inc]
    def get_temperature(self):
        '''Get current motor temperature'''
        self.get_motor_temperature()
        return self.x33+self.x34

    def get_touch_type(self):
        '''Get touch type'''
        self.send_frame(0xb0,[],sleep=0.03)
        self.send_frame(0xb1,[],sleep=0.03)
        t = []
        for i in range(3):
            t = self.xb1
            time.sleep(0.01)
        if len(t) == 2:
            return 2
        else:
            self.send_frame(0x20,[],sleep=0.03)
            time.sleep(0.01)
            if self.normal_force[0] == -1:
                return -1
            else:
                return 1
    
    def get_touch(self):
        '''Get touch data'''
        self.send_frame(0xb1,[],sleep=0.03)
        self.send_frame(0xb2,[],sleep=0.03)
        self.send_frame(0xb3,[],sleep=0.03)
        self.send_frame(0xb4,[],sleep=0.03)
        self.send_frame(0xb5,[],sleep=0.03)
        return [self.xb1[1],self.xb2[1],self.xb3[1],self.xb4[1],self.xb5[1],0] # The last digit is palm, currently not available

    def get_matrix_touch(self):
        self.send_frame(0xb1,[0xc6],sleep=0.06)
        self.send_frame(0xb2,[0xc6],sleep=0.06)
        self.send_frame(0xb3,[0xc6],sleep=0.06)
        self.send_frame(0xb4,[0xc6],sleep=0.06)
        self.send_frame(0xb5,[0xc6],sleep=0.06)
        return self.thumb_matrix , self.index_matrix , self.middle_matrix , self.ring_matrix , self.little_matrix
    
    def get_matrix_touch_v2(self):
        self.send_frame(0xb1,[0xc6],sleep=0.005)
        self.send_frame(0xb2,[0xc6],sleep=0.005)
        self.send_frame(0xb3,[0xc6],sleep=0.005)
        self.send_frame(0xb4,[0xc6],sleep=0.005)
        self.send_frame(0xb5,[0xc6],sleep=0.005)
        return self.thumb_matrix , self.index_matrix , self.middle_matrix , self.ring_matrix , self.little_matrix

    def get_thumb_matrix_touch(self,sleep_time=0.005):
        self.send_frame(0xb1,[0xc6],sleep=sleep_time)
        return self.thumb_matrix
    
    def get_index_matrix_touch(self,sleep_time=0.005):
        self.send_frame(0xb2,[0xc6],sleep=sleep_time)
        return self.index_matrix
    
    def get_middle_matrix_touch(self,sleep_time=0.005):
        self.send_frame(0xb3,[0xc6],sleep=sleep_time)
        return self.middle_matrix
    
    def get_ring_matrix_touch(self,sleep_time=0.005):
        self.send_frame(0xb4,[0xc6],sleep=sleep_time)
        return self.ring_matrix
    
    def get_little_matrix_touch(self,sleep_time=0.005):
        self.send_frame(0xb5,[0xc6],sleep=sleep_time)
        return self.little_matrix


    def get_torque(self):
        '''Get current motor torque'''
        if self.version != None and self.version[4]< 36:
            return [-1] * 5
        else:
            self.send_frame(0x02, [])
            time.sleep(0.002)
            self.send_frame(0x03,[])
            time.sleep(0.002)
            return self.x02+self.x03
    
    def get_fault(self):
        '''Get motor fault'''
        self.get_motor_fault_code()
        return self.x35+self.x36
    
    def get_current(self):
        '''Get current'''
        #return [-1] * 5
        self.send_frame(0x02, [])
        time.sleep(0.002)
        self.send_frame(0x03,[])
        return self.x02+self.x03
    
    def get_serial_number(self):
        try:
            self.send_frame(0xC0,[],sleep=0.005)
            # 1. 使用 bytes() 函数将整数列表转换为字节对象
            #    bytes() 接收一个由 0-255 之间的整数组成的列表。
            byte_data = bytes(self.serial_number)
            # 2. 使用 .decode() 方法将字节对象解码为 ASCII 字符串
            result_string = byte_data.decode('ascii')
            if result_string == "":
                return "-1"
            else:
                # print(f"原始 ASCII 码列表: {self.serial_number}")
                # print(f"解码后的字符串: {result_string}")
                return result_string
        except:
            return "-1"

    def get_finger_order(self):
        return ["thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch", "middle_mcp_pitch", "ring_mcp_pitch", "pinky_mcp_pitch",
                        "index_mcp_roll", "ring_mcp_roll", "pinky_mcp_roll", "thumb_cmc_roll"]

    def show_fun_table(self):
        # if len(data) != 8 or data[0] != 0x64:
        #     raise ValueError("数据格式不正确")
        data = self.version
        result = {
            "自由度": data[0],
            "机械版本": data[1],
            "版本序号": data[2],
            "手方向": chr(data[3]),  # ASCII 转字符
            "软件版本": f"V{data[4] >> 4}.{data[4] & 0x0F}",
            "硬件版本": f"V{data[5] >> 4}.{data[5] & 0x0F}",
            "修订标志": data[6],
            "set_position": "Y",
            "set_torque": "Y",
            "set_speed": "Y",
            "get_version": "Y",
            "get_current_status": "Y",
            "get_speed": "Y",
            "get_temperature": "Y",
            "get_touch_type": "Y",
            "get_matrix_touch": "Y",
            "get_fault": "Y",
            "get_current": "current == torque"
        }
        
        #return [data[0],data[1],data[2],chr(data[3]),f"V{data[4] >> 4}.{data[4] & 0x0F}",f"V{data[5] >> 4}.{data[5] & 0x0F}",data[6]]
        table = [[k, v] for k, v in result.items()]
        #print(tabulate(table, tablefmt="grid"), flush=True)


    # # 示例数据
    # data = [0x64, 0x15, 0x03, 0x0A, 0x4C, 0x11, 0x22, 0x01]
    # parsed = parse_version_data(data)

    # # 打印结果
    # for k, v in parsed.items():
    #     print(f"{k}: {v}")


    def close_can_interface(self):
        """Stop the CAN communication."""
        self.running = False
        if self.receive_thread.is_alive():
            self.receive_thread.join()
        if self.bus:
            self.bus.shutdown()
