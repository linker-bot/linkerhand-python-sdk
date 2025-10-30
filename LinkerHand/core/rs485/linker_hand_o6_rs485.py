#!/usr/bin/env python3
"""
O6 机械手 Modbus-RTU 控制类
Ubuntu 20.04 + Python3 测试通过
author : hejianxin
"""

import minimalmodbus
import serial
import time
import logging
import numpy as np

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)-8s %(message)s",
    datefmt="%H:%M:%S"
)

# ------------------------------------------------------------------
# 读输入寄存器地址枚举（功能码 04，只读）
# ------------------------------------------------------------------
REG_RD_CURRENT_THUMB_PITCH      = 0   # 大拇指弯曲角度（0-255，小=弯，大=伸）
REG_RD_CURRENT_THUMB_YAW        = 1   # 大拇指横摆角度（0-255，小=靠掌心，大=远离）
REG_RD_CURRENT_INDEX_PITCH      = 2   # 食指弯曲角度
REG_RD_CURRENT_MIDDLE_PITCH     = 3   # 中指弯曲角度
REG_RD_CURRENT_RING_PITCH       = 4   # 无名指弯曲角度
REG_RD_CURRENT_LITTLE_PITCH     = 5   # 小拇指弯曲角度
REG_RD_CURRENT_THUMB_TORQUE     = 6   # 大拇指弯曲转矩（0-255）
REG_RD_CURRENT_THUMB_YAW_TORQUE = 7   # 大拇指横摆转矩
REG_RD_CURRENT_INDEX_TORQUE     = 8   # 食指转矩
REG_RD_CURRENT_MIDDLE_TORQUE    = 9   # 中指转矩
REG_RD_CURRENT_RING_TORQUE      = 10  # 无名指转矩
REG_RD_CURRENT_LITTLE_TORQUE    = 11  # 小拇指转矩
REG_RD_CURRENT_THUMB_SPEED      = 12  # 大拇指弯曲速度（0-255）
REG_RD_CURRENT_THUMB_YAW_SPEED  = 13  # 大拇指横摆速度
REG_RD_CURRENT_INDEX_SPEED      = 14  # 食指速度
REG_RD_CURRENT_MIDDLE_SPEED     = 15  # 中指速度
REG_RD_CURRENT_RING_SPEED       = 16  # 无名指速度
REG_RD_CURRENT_LITTLE_SPEED     = 17  # 小拇指速度
REG_RD_THUMB_TEMP               = 18  # 大拇指弯曲温度（0-70℃）
REG_RD_THUMB_YAW_TEMP           = 19  # 大拇指横摆温度
REG_RD_INDEX_TEMP               = 20  # 食指温度
REG_RD_MIDDLE_TEMP              = 21  # 中指温度
REG_RD_RING_TEMP                = 22  # 无名指温度
REG_RD_LITTLE_TEMP              = 23  # 小拇指温度
REG_RD_THUMB_ERROR              = 24  # 大拇指错误码
REG_RD_THUMB_YAW_ERROR          = 25  # 大拇指横摆错误码
REG_RD_INDEX_ERROR              = 26  # 食指错误码
REG_RD_MIDDLE_ERROR             = 27  # 中指错误码
REG_RD_RING_ERROR               = 28  # 无名指错误码
REG_RD_LITTLE_ERROR             = 29  # 小拇指错误码
REG_RD_HAND_FREEDOM             = 30  # 版本号（与机械手标签相同）
REG_RD_HAND_VERSION             = 31  # 手版本
REG_RD_HAND_NUMBER              = 32  # 手编号
REG_RD_HAND_DIRECTION           = 33  # 手方向（左/右）
REG_RD_SOFTWARE_VERSION         = 34  # 软件版本
REG_RD_HARDWARE_VERSION         = 35  # 硬件版本

# ------------------------------------------------------------------
# 写保持寄存器地址枚举（功能码 16，读写）
# ------------------------------------------------------------------
REG_WR_THUMB_PITCH       = 0   # 大拇指弯曲角度（0-255）
REG_WR_THUMB_YAW         = 1   # 大拇指横摆角度
REG_WR_INDEX_PITCH       = 2   # 食指弯曲角度
REG_WR_MIDDLE_PITCH      = 3   # 中指弯曲角度
REG_WR_RING_PITCH        = 4   # 无名指弯曲角度
REG_WR_LITTLE_PITCH      = 5   # 小拇指弯曲角度
REG_WR_THUMB_TORQUE      = 6   # 大拇指弯曲转矩
REG_WR_THUMB_YAW_TORQUE  = 7   # 大拇指横摆转矩
REG_WR_INDEX_TORQUE      = 8   # 食指转矩
REG_WR_MIDDLE_TORQUE     = 9   # 中指转矩
REG_WR_RING_TORQUE       = 10  # 无名指转矩
REG_WR_LITTLE_TORQUE     = 11  # 小拇指转矩
REG_WR_THUMB_SPEED       = 12  # 大拇指弯曲速度
REG_WR_THUMB_YAW_SPEED   = 13  # 大拇指横摆速度
REG_WR_INDEX_SPEED       = 14  # 食指速度
REG_WR_MIDDLE_SPEED      = 15  # 中指速度
REG_WR_RING_SPEED        = 16  # 无名指速度
REG_WR_LITTLE_SPEED      = 17  # 小拇指速度



class LinkerHandO6RS485:
    """O6 机械手 Modbus-RTU 控制类，支持左右手"""

    # RIGHT_ID = 0x27          # 右手站号
    # LEFT_ID  = 0x28          # 左手站号
    # BAUD     = 115200        # 固定波特率
    TTL_TIMEOUT = 0.15       # 串口超时
    FRAME_GAP = 0.030          # 30 ms
    _last_ts  = 0              # 上一次帧结束时间

    def __init__(self, hand_id=0x27,modbus_port="/dev/ttyUSB0",baudrate=115200):
        """
        modbus_port     : 串口设备，如 /dev/ttyUSB0
        hand_id : 右手0x27，左手0x28
        """
        self._id = hand_id

        self.joint_name=["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲", "小拇指弯曲"]
        try:
            self.instr = minimalmodbus.Instrument(modbus_port, self._id, mode='rtu')
            self.instr.serial.baudrate = baudrate
            self.instr.serial.bytesize = 8
            self.instr.serial.parity   = serial.PARITY_NONE
            self.instr.serial.stopbits = 1
            self.instr.serial.timeout  = self.TTL_TIMEOUT
            self.instr.close_port_after_each_call = True
            self.instr.clear_buffers_before_each_transaction = True
        except Exception as e:
            logging.error(f"初始化失败: {e}")
            raise

    # ----------------------------------------------------------
    # 底层读写封装
    # ----------------------------------------------------------
    def _bus_free(self):
        """保证距离上一帧 ≥ 30 ms"""
        elapse = time.perf_counter() - self._last_ts
        if elapse < self.FRAME_GAP:
            time.sleep(self.FRAME_GAP - elapse)

    def _read_reg(self, addr: int) -> int:
        """读单个输入寄存器（功能码 04），带 30 ms 帧间隔"""
        self._bus_free()
        try:
            return self.instr.read_register(addr, functioncode=4)
        finally:
            self._last_ts = time.perf_counter()   # 记录帧结束时刻

    def _write_reg(self, addr: int, value: int):
        """写单个保持寄存器（功能码 16），带 30 ms 帧间隔"""
        if not 0 <= value <= 255:
            raise ValueError("value must be 0-255")
        self._bus_free()
        try:
            self.instr.write_register(addr, value, functioncode=16)
        finally:
            self._last_ts = time.perf_counter()

    # ----------------------------------------------------------
    # 只读属性（实时读取）
    # ----------------------------------------------------------
    def get_thumb_pitch(self) -> int:          return self._read_reg(REG_RD_CURRENT_THUMB_PITCH)      # 大拇指弯曲角度
    def get_thumb_yaw(self) -> int:            return self._read_reg(REG_RD_CURRENT_THUMB_YAW)        # 大拇指横摆角度
    def get_index_pitch(self) -> int:          return self._read_reg(REG_RD_CURRENT_INDEX_PITCH)      # 食指弯曲角度
    def get_middle_pitch(self) -> int:         return self._read_reg(REG_RD_CURRENT_MIDDLE_PITCH)     # 中指弯曲角度
    def get_ring_pitch(self) -> int:           return self._read_reg(REG_RD_CURRENT_RING_PITCH)       # 无名指弯曲角度
    def get_little_pitch(self) -> int:         return self._read_reg(REG_RD_CURRENT_LITTLE_PITCH)     # 小拇指弯曲角度

    def get_thumb_torque(self) -> int:         return self._read_reg(REG_RD_CURRENT_THUMB_TORQUE)     # 大拇指弯曲转矩
    def get_thumb_yaw_torque(self) -> int:     return self._read_reg(REG_RD_CURRENT_THUMB_YAW_TORQUE) # 大拇指横摆转矩
    def get_index_torque(self) -> int:         return self._read_reg(REG_RD_CURRENT_INDEX_TORQUE)     # 食指转矩
    def get_middle_torque(self) -> int:        return self._read_reg(REG_RD_CURRENT_MIDDLE_TORQUE)    # 中指转矩
    def get_ring_torque(self) -> int:          return self._read_reg(REG_RD_CURRENT_RING_TORQUE)      # 无名指转矩
    def get_little_torque(self) -> int:        return self._read_reg(REG_RD_CURRENT_LITTLE_TORQUE)    # 小拇指转矩

    def get_thumb_speed(self) -> int:          return self._read_reg(REG_RD_CURRENT_THUMB_SPEED)      # 大拇指弯曲速度
    def get_thumb_yaw_speed(self) -> int:      return self._read_reg(REG_RD_CURRENT_THUMB_YAW_SPEED)  # 大拇指横摆速度
    def get_index_speed(self) -> int:          return self._read_reg(REG_RD_CURRENT_INDEX_SPEED)      # 食指速度
    def get_middle_speed(self) -> int:         return self._read_reg(REG_RD_CURRENT_MIDDLE_SPEED)     # 中指速度
    def get_ring_speed(self) -> int:           return self._read_reg(REG_RD_CURRENT_RING_SPEED)       # 无名指速度
    def get_little_speed(self) -> int:         return self._read_reg(REG_RD_CURRENT_LITTLE_SPEED)     # 小拇指速度

    def get_thumb_temp(self) -> int:           return self._read_reg(REG_RD_THUMB_TEMP)               # 大拇指温度(℃)
    def get_thumb_yaw_temp(self) -> int:       return self._read_reg(REG_RD_THUMB_YAW_TEMP)           # 大拇指横摆温度
    def get_index_temp(self) -> int:           return self._read_reg(REG_RD_INDEX_TEMP)               # 食指温度
    def get_middle_temp(self) -> int:          return self._read_reg(REG_RD_MIDDLE_TEMP)              # 中指温度
    def get_ring_temp(self) -> int:            return self._read_reg(REG_RD_RING_TEMP)                # 无名指温度
    def get_little_temp(self) -> int:          return self._read_reg(REG_RD_LITTLE_TEMP)              # 小拇指温度

    def get_thumb_error(self) -> int:          return self._read_reg(REG_RD_THUMB_ERROR)              # 大拇指错误码
    def get_thumb_yaw_error(self) -> int:      return self._read_reg(REG_RD_THUMB_YAW_ERROR)          # 大拇指横摆错误码
    def get_index_error(self) -> int:          return self._read_reg(REG_RD_INDEX_ERROR)              # 食指错误码
    def get_middle_error(self) -> int:         return self._read_reg(REG_RD_MIDDLE_ERROR)             # 中指错误码
    def get_ring_error(self) -> int:           return self._read_reg(REG_RD_RING_ERROR)               # 无名指错误码
    def get_little_error(self) -> int:         return self._read_reg(REG_RD_LITTLE_ERROR)             # 小拇指错误码

    def get_hand_freedom(self) -> int:         return self._read_reg(REG_RD_HAND_FREEDOM)             # 版本号（标签）
    def get_hand_version(self) -> int:         return self._read_reg(REG_RD_HAND_VERSION)             # 手版本
    def get_hand_number(self) -> int:          return self._read_reg(REG_RD_HAND_NUMBER)              # 手编号
    def get_hand_direction(self) -> int:       return self._read_reg(REG_RD_HAND_DIRECTION)           # 手方向
    def get_software_version(self) -> int:     return self._read_reg(REG_RD_SOFTWARE_VERSION)         # 软件版本
    def get_hardware_version(self) -> int:     return self._read_reg(REG_RD_HARDWARE_VERSION)         # 硬件版本

    # ----------------------------------------------------------
    # 写保持寄存器
    # ----------------------------------------------------------
    def set_thumb_pitch(self, v: int):          self._write_reg(REG_WR_THUMB_PITCH, v)       # 设置大拇指弯曲角度
    def set_thumb_yaw(self, v: int):            self._write_reg(REG_WR_THUMB_YAW, v)         # 设置大拇指横摆角度
    def set_index_pitch(self, v: int):          self._write_reg(REG_WR_INDEX_PITCH, v)       # 设置食指弯曲角度
    def set_middle_pitch(self, v: int):         self._write_reg(REG_WR_MIDDLE_PITCH, v)      # 设置中指弯曲角度
    def set_ring_pitch(self, v: int):           self._write_reg(REG_WR_RING_PITCH, v)        # 设置无名指弯曲角度
    def set_little_pitch(self, v: int):         self._write_reg(REG_WR_LITTLE_PITCH, v)      # 设置小拇指弯曲角度

    def set_thumb_torque(self, v: int):         self._write_reg(REG_WR_THUMB_TORQUE, v)      # 设置大拇指弯曲转矩
    def set_thumb_yaw_torque(self, v: int):     self._write_reg(REG_WR_THUMB_YAW_TORQUE, v)  # 设置大拇指横摆转矩
    def set_index_torque(self, v: int):         self._write_reg(REG_WR_INDEX_TORQUE, v)      # 设置食指转矩
    def set_middle_torque(self, v: int):        self._write_reg(REG_WR_MIDDLE_TORQUE, v)     # 设置中指转矩
    def set_ring_torque(self, v: int):          self._write_reg(REG_WR_RING_TORQUE, v)       # 设置无名指转矩
    def set_little_torque(self, v: int):        self._write_reg(REG_WR_LITTLE_TORQUE, v)     # 设置小拇指转矩

    def set_thumb_speed(self, v: int):          self._write_reg(REG_WR_THUMB_SPEED, v)       # 设置大拇指弯曲速度
    def set_thumb_yaw_speed(self, v: int):      self._write_reg(REG_WR_THUMB_YAW_SPEED, v)   # 设置大拇指横摆速度
    def set_index_speed(self, v: int):          self._write_reg(REG_WR_INDEX_SPEED, v)       # 设置食指速度
    def set_middle_speed(self, v: int):         self._write_reg(REG_WR_MIDDLE_SPEED, v)      # 设置中指速度
    def set_ring_speed(self, v: int):           self._write_reg(REG_WR_RING_SPEED, v)        # 设置无名指速度
    def set_little_speed(self, v: int):         self._write_reg(REG_WR_LITTLE_SPEED, v)      # 设置小拇指速度

    # ----------------------------------------------------------
    # 固定函数
    # ----------------------------------------------------------
    def is_valid_6xuint8(self, lst) -> bool:
        lst = [int(x) for x in lst]
        return (
            isinstance(lst, list) and
            len(lst) == 6 and
            all(type(x) is int and 0 <= x <= 255 for x in lst)
        )
    
    def set_joint_positions(self, joint_angles=[0] * 6):
        if self.is_valid_6xuint8(joint_angles):
            self.set_thumb_pitch(joint_angles[0])
            self.set_thumb_yaw(joint_angles[1])
            self.set_index_pitch(joint_angles[2])
            self.set_middle_pitch(joint_angles[3])
            self.set_ring_pitch(joint_angles[4])
            self.set_little_pitch(joint_angles[5])

    def set_speed(self, speed=[200] * 6):
        """设置速度 params: list len=6"""
        if self.is_valid_6xuint8(speed):
            try:
                self.set_thumb_speed(speed[0])
                self.set_thumb_yaw_speed(speed[1])
                self.set_index_speed(speed[2])
                self.set_middle_speed(speed[3])
                self.set_ring_speed(speed[4])
                self.set_little_speed(speed[5])
            except:
                pass
    
    def set_torque(self, torque=[200] * 6):
        """设置扭矩 params: list len=6"""
        if self.is_valid_6xuint8(torque):
            self.set_thumb_torque(torque[0])
            self.set_thumb_yaw_torque(torque[1])
            self.set_index_torque(torque[2])
            self.set_middle_torque(torque[3])
            self.set_ring_torque(torque[4])
            self.set_little_torque(torque[5])

    def set_current(self, current=[200] * 6):
        """设置电流 params: list len=6"""
        print("当前O6不支持设置电流", flush=True)
        pass

    def get_version(self) -> list:
        """获取当前固件版本号"""
        return [self.get_hand_freedom(),self.get_hand_version(),self.get_hand_number(),self.get_hand_direction(), self.get_software_version(),self.get_hardware_version()]
    
    def get_current(self):
        """获取电流"""
        print("当前O6不支持获取电流", flush=True)
        pass

    def get_state(self) -> list:
        """获取手指电机状态"""
        return [self.get_thumb_pitch(),self.get_thumb_yaw(), self.get_index_pitch(), self.get_middle_pitch(),
                   self.get_ring_pitch(), self.get_little_pitch()]
    
    def get_state_for_pub(self) -> list:
        return self.get_state()

    def get_current_status(self) -> list:
        return self.get_state()
    
    def get_speed(self) -> list:
        """获取当前速度"""
        return [self.get_thumb_speed(), self.get_thumb_yaw_speed(), self.get_index_speed(), self.get_middle_speed(), self.get_ring_speed(), self.get_little_speed()]
    
    def get_joint_speed(self) -> list:
        return self.get_speed()
    
    def get_touch_type(self) -> list:
        """获取压感类型，当前O6不具备压感"""
        return -1
    
    def get_normal_force(self) -> list:
        """获取压感数据：点式"""
        return [-1] * 5
    
    def get_tangential_force(self) -> list:
        """获取压感数据：点式"""
        return [-1] * 5
    
    def get_approach_inc(self) -> list:
        """获取压感数据：点式"""
        return [-1] * 5
    
    def get_touch(self) -> list:
        return [-1] * 5
    
    def get_matrix_touch(self) -> list:
        """获取压感数据：矩阵式"""
        thumb_matrix = np.full((12, 6), -1)
        index_matrix = np.full((12, 6), -1)
        middle_matrix = np.full((12, 6), -1)
        ring_matrix = np.full((12, 6), -1)
        little_matrix = np.full((12, 6), -1)
        return thumb_matrix , index_matrix , middle_matrix , ring_matrix , little_matrix
    
    def get_matrix_touch_v2(self) -> list:
        """获取压感数据：矩阵式"""
        return self.get_matrix_touch()
    
    def get_torque(self) -> list:
        """获取当前扭矩"""
        return [self.get_thumb_torque(), self.get_thumb_yaw_torque(), self.get_index_torque(), self.get_middle_torque(), self.get_ring_torque(), self.get_little_torque()]
    
    def get_temperature(self) -> list:
        """获取当前电机温度"""
        return [self.get_thumb_temp(), self.get_thumb_yaw_temp(), self.get_index_temp(), self.get_middle_temp(), self.get_ring_temp(), self.get_little_temp()]
    
    def get_fault(self) -> list:
        """获取当前电机故障码"""
        return [self.get_thumb_error(), self.get_thumb_yaw_error(), self.get_index_error(), self.get_middle_error(), self.get_ring_error(), self.get_little_error()]




    # ----------------------------------------------------------
    # 便捷函数
    # ----------------------------------------------------------
    def set_all_fingers(self, pitch: int):
        """同时设置五指弯曲角度（0-255）"""
        for fn in (self.set_thumb_pitch,self.set_thumb_yaw, self.set_index_pitch, self.set_middle_pitch,
                   self.set_ring_pitch, self.set_little_pitch):
            fn(pitch)

    def relax(self):
        """全部手指伸直（255）"""
        self.set_all_fingers(255)

    def fist(self):
        """全部手指弯曲（0）"""
        self.set_all_fingers(0)

    def dump_status(self):
        """打印当前所有可读状态"""
        print("--------- O6 Hand Status ---------")
        print(f"hand_state  state={self.get_state()}")
        print(f"Temperature  {self.get_temperature()}℃")
        print(f"Error code   {self.get_fault()}")
        print(f"version version={self.get_embedded_version()}")
        print("----------------------------------")


# ------------------------------------------------------------------
# 命令行快速测试
# ------------------------------------------------------------------
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="O6 Hand Modbus tester")
    parser.add_argument("-p", "--port", required=True, help="串口, 如 /dev/ttyUSB0")
    parser.add_argument("-l", "--left", action="store_true", help="左手，默认右手")
    args = parser.parse_args()

    hand = LinkerHandO6RS485(hand_id=0x28,modbus_port="/dev/ttyUSB0",baudrate=115200)
    hand.dump_status()
    print("执行 relax → 伸直")
    hand.relax()
    time.sleep(1)
    print("执行 fist → 握拳")
    hand.fist()
    time.sleep(1)
    hand.relax()
    print("演示完成")
