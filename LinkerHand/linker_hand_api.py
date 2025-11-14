#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
import sys, os, time,threading
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils.mapping import *
from utils.color_msg import ColorMsg
from utils.load_write_yaml import LoadWriteYaml
from utils.open_can import OpenCan

class LinkerHandApi:
    def __init__(self, hand_type="left", hand_joint="L10", modbus = "None",can="can0"):  # Ubuntu:can0   win:PCAN_USBBUS1
        self.last_position = []
        self.yaml = LoadWriteYaml()
        self.config = self.yaml.load_setting_yaml()
        self.version = self.config["VERSION"]
        self.can = can
        ColorMsg(msg=f"Current SDK version: {self.version}", color="green")
        self.hand_joint = hand_joint
        self.hand_type = hand_type
        if self.hand_type == "left":
            self.hand_id = 0x28  # Left hand
        if self.hand_type == "right":
            self.hand_id = 0x27  # Right hand
        if self.hand_joint.upper() == "O6":
            if modbus != "None":
                from core.rs485.linker_hand_o6_rs485 import LinkerHandO6RS485
                self.hand = LinkerHandO6RS485(hand_id=self.hand_id,modbus_port=modbus,baudrate=115200)
            else:
                from core.can.linker_hand_o6_can import LinkerHandO6Can
                self.hand = LinkerHandO6Can(can_id=self.hand_id,can_channel=self.can, yaml=self.yaml)
        if self.hand_joint == "L6" or self.hand_joint == "L6P":
            if modbus != "None":
                from core.rs485.linker_hand_l6_rs485 import LinkerHandL6RS485
                self.hand = LinkerHandL6RS485(hand_id=self.hand_id,modbus_port=modbus,baudrate=115200)
            else:
                from core.can.linker_hand_o6_can import LinkerHandO6Can
                self.hand = LinkerHandO6Can(can_id=self.hand_id,can_channel=self.can, yaml=self.yaml)
        if self.hand_joint == "L7":
            from core.can.linker_hand_l7_can import LinkerHandL7Can
            self.hand = LinkerHandL7Can(can_id=self.hand_id,can_channel=self.can, yaml=self.yaml)
        if self.hand_joint == "L10":
            if modbus != "None":
                from core.rs485.linker_hand_l10_rs485 import LinkerHandL10RS485
                self.hand = LinkerHandL10RS485(hand_id=self.hand_id,modbus_port=modbus,baudrate=115200)
            else:
                from core.can.linker_hand_l10_can import LinkerHandL10Can
                self.hand = LinkerHandL10Can(can_id=self.hand_id,can_channel=self.can, yaml=self.yaml)
        if self.hand_joint == "L20":
            from core.can.linker_hand_l20_can import LinkerHandL20Can
            self.hand = LinkerHandL20Can(can_id=self.hand_id,can_channel=self.can, yaml=self.yaml)
        if self.hand_joint == "G20":
            from core.can.linker_hand_g20_can import LinkerHandG20Can
            self.hand = LinkerHandG20Can(can_id=self.hand_id,can_channel=self.can, yaml=self.yaml)
        if self.hand_joint == "L21":
            from core.can.linker_hand_l21_can import LinkerHandL21Can
            self.hand = LinkerHandL21Can(can_id=self.hand_id,can_channel=self.can, yaml=self.yaml)
        if self.hand_joint == "L25":
            from core.can.linker_hand_l25_can import LinkerHandL25Can
            self.hand = LinkerHandL25Can(can_id=self.hand_id,can_channel=self.can, yaml=self.yaml)
        # Open can0
        if sys.platform == "linux" and modbus=="None":
            self.open_can = OpenCan(load_yaml=self.yaml)
            self.open_can.open_can(self.can)
            self.is_can = self.open_can.is_can_up_sysfs(interface=self.can)
            if not self.is_can:
                ColorMsg(msg=f"{self.can} interface is not open", color="red")
                sys.exit(1)
        version = self.get_embedded_version()
        if version == None or len(version) == 0:
            ColorMsg(msg="Warning: Hardware version number not recognized, it is recommended to terminate the program and re insert USB to CAN conversion", color="yellow")
        else:
            ColorMsg(msg=f"Embedded:{version}", color="green")
    
    # Five-finger movement
    def finger_move(self, pose=[]):
        '''
        Five-finger movement
        @params: pose list L7 len(7) | L10 len(10) | L20 len(20) | L25 len(25) 0~255
        '''
        
        if len(pose) == 0:
            return
        if any(not isinstance(x, (int, float)) or x < 0 or x > 255 for x in pose):
            ColorMsg(msg=f"The numerical range cannot be less than 0 or greater than 255",color="red")
            return
        if (self.hand_joint.upper() == "O6" or self.hand_joint.upper() == "L6" or self.hand_joint.upper() == "L6P") and len(pose) == 6:
            self.hand.set_joint_positions(pose)
        elif self.hand_joint == "L7" and len(pose) == 7:
            self.hand.set_joint_positions(pose)
        elif self.hand_joint == "L10" and len(pose) == 10:
            self.hand.set_joint_positions(pose)
        elif self.hand_joint == "L20" and len(pose) == 20:
            self.hand.set_joint_positions(pose)
        elif self.hand_joint == "G20" and len(pose) == 20:
            self.hand.set_joint_positions(pose)
        elif self.hand_joint == "L21" and len(pose) == 25:
            self.hand.set_joint_positions(pose)
        elif self.hand_joint == "L25" and len(pose) == 25:
            self.hand.set_joint_positions(pose)
        else:
            ColorMsg(msg=f"Current LinkerHand is {self.hand_type}{self.hand_joint}, action sequence is {pose}, does not match", color="red")
        self.last_position = pose

    def _get_normal_force(self):
        '''# Get normal force'''
        self.hand.get_normal_force()
    
    def _get_tangential_force(self):
        '''# Get tangential force'''
        self.hand.get_tangential_force()
    
    def _get_tangential_force_dir(self):
        '''# Get tangential force direction'''
        self.hand.get_tangential_force_dir()
    
    def _get_approach_inc(self):
        '''# Get approach increment'''
        self.hand.get_approach_inc()
    
    def set_speed(self, speed=[100]*5):
        '''# Set speed'''
        has_non_int = any(not isinstance(x, (int, float)) or x < 0 or x > 255 for x in speed)
        if has_non_int:
            print("Set Speed The numerical range can only be positive integers or floating-point numbers between 0 and 255", flush=True)
            return
        if len(speed) < 5:
            print("数据长度不够,至少5个元素", flush=True)
            return
        if self.hand_joint == "L7" and len(speed) < 7:
            print("数据长度不够,至少7个元素", flush=True)
            return
        ColorMsg(msg=f"{self.hand_type} {self.hand_joint} set speed to {speed}", color="green")
        self.hand.set_speed(speed=speed)
    
    def set_joint_speed(self, speed=[100]*5):
        '''Set speed by topic'''
        if len(speed) == 0:
            return
        if any(not isinstance(x, (int, float)) or x < 10 or x > 255 for x in speed):
            ColorMsg(msg=f"The numerical range cannot be less than 10 or greater than 255",color="red")
            return
        self.hand.set_speed(speed=speed)
    
    def set_torque(self, torque=[180] * 5):
        '''Set maximum torque'''
        has_non_int = any(not isinstance(x, (int, float)) or x < 0 or x > 255 for x in torque)
        if has_non_int:
            print("Set Torque The numerical range can only be positive integers or floating-point numbers between 0 and 255", flush=True)
            return
        if len(torque) < 5:
            print("数据长度不够,至少5个元素", flush=True)
            return
        if self.hand_joint == "L7" and len(torque) < 7:
            print("数据长度不够,至少7个元素", flush=True)
            return
        if (self.hand_joint == "L6" or self.hand_joint == "O6" or self.hand_joint.upper() == "L6P") and len(torque) != 6:
            print("L6 or O6数据长度错误,至少6个元素", flush=True)
            return
        ColorMsg(msg=f"{self.hand_type} {self.hand_joint} set maximum torque to {torque}", color="green")
        return self.hand.set_torque(torque=torque)
    
    
    def set_current(self, current=[250] * 5):
        '''Set current L7/L10/L25 not supported'''
        if any(not isinstance(x, (int, float)) or x < 0 or x > 255 for x in current):
            print("Set Current The numerical range can only be positive integers or floating-point numbers between 0 and 255", flush=True)
            return
        if self.hand_joint == "L20":
            return self.hand.set_current(current=current)
        else:
            pass

    def get_embedded_version(self):
        '''Get embedded version'''
        return self.hand.get_version()
    
    def get_current(self):
        '''Get current'''
        return self.hand.get_current()
    
    def get_state(self):
        '''Get current joint state'''
        return self.hand.get_current_status()

    
    def get_state_for_pub(self):
        return self.hand.get_current_pub_status()
    
    def get_speed(self):
        '''Get speed'''
        return self.hand.get_speed()

    
    def get_joint_speed(self):
        speed = []
        if self.hand_joint.upper() == "O6" or self.hand_joint.upper() == "L6" or self.hand_joint.upper() == "L6P":
            return self.hand.get_speed()
        elif self.hand_joint == "L7":
            return self.hand.get_speed()
        elif self.hand_joint == "L10":
            speed = self.hand.get_speed()
            return speed
        elif self.hand_joint == "G20":
            return self.hand.get_speed()
        elif self.hand_joint == "L20":
            speed = self.hand.get_speed()
            return [255, speed[1], speed[2], speed[3], speed[4], 255, 255, 255, 255, 255, speed[0], 255, 255, 255, 255, 255, 255, 255, 255, 255]
        elif self.hand_joint == "L21":
            return self.hand.get_speed()
        elif self.hand_joint == "L25":
            return self.hand.get_speed()

    def get_touch_type(self):
        '''Get touch type'''
        return self.hand.get_touch_type()
    
    def get_force(self):
        '''Get normal force, tangential force, tangential force direction, approach sensing data'''
        self._get_normal_force()
        self._get_tangential_force()
        self._get_tangential_force_dir()
        self._get_approach_inc()
        return self.hand.get_force()

    def get_touch(self):
        '''Get touch data'''
        return self.hand.get_touch()
    
    def get_matrix_touch(self):
        return self.hand.get_matrix_touch()
    
    def get_matrix_touch_v2(self):
        return self.hand.get_matrix_touch_v2()
    

    def get_thumb_matrix_touch(self):
        return self.hand.get_thumb_matrix_touch()
    
    def get_index_matrix_touch(self):
        return self.hand.get_index_matrix_touch()
    
    def get_middle_matrix_touch(self):
        return self.hand.get_middle_matrix_touch()
    
    def get_ring_matrix_touch(self):
        return self.hand.get_ring_matrix_touch()
    
    def get_little_matrix_touch(self):
        return self.hand.get_little_matrix_touch()

    def get_torque(self):
        '''Get current maximum torque'''
        return self.hand.get_torque()
    
    def get_temperature(self):
        '''Get current motor temperature'''
        return self.hand.get_temperature()
    
    def get_fault(self):
        '''Get motor fault code'''
        return self.hand.get_fault()
    
    def clear_faults(self):
        '''Clear motor fault codes Not supported yet, currently only supports L20'''
        if self.hand_joint == "L20":
            self.hand.clear_faults()
        else:
            return [0] * 5

    def set_enable(self):
        '''Set motor enable Only supports L25'''
        if self.hand_joint == "L25":
            self.hand.set_enable_mode()
        else:
            pass

    def set_disable(self):
        '''Set motor disable Only supports L25'''
        if self.hand_joint == "L25":
            self.hand.set_disability_mode()
        else:
            pass

    def get_finger_order(self):
        '''Get finger motor order'''
        if self.hand_joint == "L21" or self.hand_joint == "L25" or self.hand_joint == "G20":
            return self.hand.get_finger_order()
        else:
            return []
        
    def range_to_arc_left(self, state, hand_joint):
        return range_to_arc_left(left_range=state, hand_joint=hand_joint)
    
    def range_to_arc_right(self, state, hand_joint):
        return range_to_arc_right(right_range=state, hand_joint=hand_joint)
    
    def arc_to_range_left(self,state,hand_joint):
        return arc_to_range_left(hand_arc_l=state,hand_joint=hand_joint)
    
    def arc_to_range_right(self,state,hand_joint):
        return arc_to_range_right(right_arc=state,hand_joint=hand_joint)
    
    def show_fun_table(self):
        self.hand.show_fun_table()
    def close_can(self):
        self.open_can.close_can0()                         

if __name__ == "__main__":
    hand = LinkerHandApi(hand_type="right", hand_joint="L10")