#!/usr/bin/env python3
"""
O6 机械手 Modbus-RTU 控制类 (基于 pymodbus 3.5.1)
"""

import os
import time
from typing import List, Dict, Any # 引入 Any 来表示灵活的输入类型
import numpy as np
import logging
from threading import Lock # 用于线程安全和总线仲裁

# 导入 pymodbus 客户端
from pymodbus.client import ModbusSerialClient
from struct import error as StructError 

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)-8s %(message)s",
    datefmt="%H:%M:%S"
)

# ------------------------------------------------------------------
# 读输入寄存器地址枚举（功能码 04，只读）- 保持原样
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
# 写保持寄存器地址枚举（功能码 16，读写）- 保持原样
# ------------------------------------------------------------------
REG_WR_THUMB_PITCH      = 0   # 大拇指弯曲角度（0-255）
REG_WR_THUMB_YAW        = 1   # 大拇指横摆角度
REG_WR_INDEX_PITCH      = 2   # 食指弯曲角度
REG_WR_MIDDLE_PITCH     = 3   # 中指弯曲角度
REG_WR_RING_PITCH       = 4   # 无名指弯曲角度
REG_WR_LITTLE_PITCH     = 5   # 小拇指弯曲角度
REG_WR_THUMB_TORQUE     = 6   # 大拇指弯曲转矩
REG_WR_THUMB_YAW_TORQUE = 7   # 大拇指横摆转矩
REG_WR_INDEX_TORQUE     = 8   # 食指转矩
REG_WR_MIDDLE_TORQUE    = 9   # 中指转矩
REG_WR_RING_TORQUE      = 10  # 无名指转矩
REG_WR_LITTLE_TORQUE    = 11  # 小拇指转矩
REG_WR_THUMB_SPEED      = 12  # 大拇指弯曲速度
REG_WR_THUMB_YAW_SPEED  = 13  # 大拇指横摆速度
REG_WR_INDEX_SPEED      = 14  # 食指速度
REG_WR_MIDDLE_SPEED     = 15  # 中指速度
REG_WR_RING_SPEED       = 16  # 无名指速度
REG_WR_LITTLE_SPEED     = 17  # 小拇指速度


class LinkerHandO6RS485:
    """O6 机械手 Modbus-RTU 控制类，使用 pymodbus 3.5.1"""

    TTL_TIMEOUT = 0.15     # 串口超时
    FRAME_GAP = 0.030      # 30 ms
    
    # KEYS for easy indexing
    JOINT_KEYS = ["thumb_pitch", "thumb_yaw", "index_pitch", 
                  "middle_pitch", "ring_pitch", "little_pitch"]

    def __init__(self, hand_id=0x27, modbus_port="/dev/ttyUSB0", baudrate=115200):
        self._id = hand_id
        self._last_ts = 0.0  # 上一次帧结束时间
        self._lock = Lock()  # 总线访问锁

        # 使用 pymodbus 3.x 客户端
        self.cli = ModbusSerialClient(
            port=modbus_port,
            baudrate=baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self.TTL_TIMEOUT,
            handle_local_echo=False
        )

        try:
            logging.info(f"Connecting to Modbus RTU on {modbus_port}...")
            self.connected = self.cli.connect()
            if not self.connected:
                raise ConnectionError(f"RS485 connect fail to {modbus_port}")
            logging.info("Connection successful.")
        except Exception as e:
            logging.error(f"Initialization failed: {e}")
            raise

    # ----------------------------------------------------------
    # 辅助方法
    # ----------------------------------------------------------
    def _bus_free(self):
        """保证距离上一帧 ≥ 30 ms"""
        with self._lock:
            elapse = time.perf_counter() - self._last_ts
            if elapse < self.FRAME_GAP:
                time.sleep(self.FRAME_GAP - elapse)

    def _execute_read(self, address: int, count: int) -> List[int]:
        """执行 Modbus 读取操作 (功能码 04), 带总线仲裁。"""
        self._bus_free()
        
        rsp = self.cli.read_input_registers(
            address=address, 
            count=count, 
            slave=self._id
        )
        
        self._last_ts = time.perf_counter()
        
        if rsp.isError():
            raise RuntimeError(f"Modbus Read Failed (Addr={address}, Count={count}): {rsp}")
        
        # 确保返回的值是 Python 原生整数
        return [int(reg) for reg in rsp.registers]

    def _execute_write(self, address: int, values: List[int]):
        """执行 Modbus 批量写入操作 (功能码 16), 带总线仲裁。"""
        self._bus_free()
        
        # values 必须是 Python 原生整数列表
        rsp = self.cli.write_registers(
            address=address, 
            values=values, 
            slave=self._id
        )
        
        self._last_ts = time.perf_counter()
        
        if rsp.isError():
            raise RuntimeError(f"Modbus Write Failed (Addr={address}, Values={values}): {rsp}")

    # ----------------------------------------------------------
    # 批量读取和数据封装（优化通信效率）
    # ----------------------------------------------------------
    def read_all_angles(self) -> List[int]:
        return self._execute_read(REG_RD_CURRENT_THUMB_PITCH, 6)
    
    def read_all_torques(self) -> List[int]:
        return self._execute_read(REG_RD_CURRENT_THUMB_TORQUE, 6)

    def read_all_speeds(self) -> List[int]:
        return self._execute_read(REG_RD_CURRENT_THUMB_SPEED, 6)
    
    def read_all_temperatures(self) -> List[int]:
        return self._execute_read(REG_RD_THUMB_TEMP, 6)
    
    def read_all_errors(self) -> List[int]:
        return self._execute_read(REG_RD_THUMB_ERROR, 6)

    def read_all_versions(self) -> List[int]:
        return self._execute_read(REG_RD_HAND_FREEDOM, 6)


    # ----------------------------------------------------------
    # 只读属性（单个寄存器读取）
    # ----------------------------------------------------------
    def _read_reg(self, addr: int) -> int:
        """读单个输入寄存器（功能码 04），带 30 ms 帧间隔"""
        return self._execute_read(addr, 1)[0]
    
    def get_thumb_pitch(self) -> int:       return self._read_reg(REG_RD_CURRENT_THUMB_PITCH)
    def get_thumb_yaw(self) -> int:         return self._read_reg(REG_RD_CURRENT_THUMB_YAW)
    def get_index_pitch(self) -> int:       return self._read_reg(REG_RD_CURRENT_INDEX_PITCH)
    def get_middle_pitch(self) -> int:      return self._read_reg(REG_RD_CURRENT_MIDDLE_PITCH)
    def get_ring_pitch(self) -> int:        return self._read_reg(REG_RD_CURRENT_RING_PITCH)
    def get_little_pitch(self) -> int:      return self._read_reg(REG_RD_CURRENT_LITTLE_PITCH)

    def get_thumb_torque(self) -> int:      return self._read_reg(REG_RD_CURRENT_THUMB_TORQUE)
    def get_thumb_yaw_torque(self) -> int:  return self._read_reg(REG_RD_CURRENT_THUMB_YAW_TORQUE)
    def get_index_torque(self) -> int:      return self._read_reg(REG_RD_CURRENT_INDEX_TORQUE)
    def get_middle_torque(self) -> int:     return self._read_reg(REG_RD_CURRENT_MIDDLE_TORQUE)
    def get_ring_torque(self) -> int:       return self._read_reg(REG_RD_CURRENT_RING_TORQUE)
    def get_little_torque(self) -> int:     return self._read_reg(REG_RD_CURRENT_LITTLE_TORQUE)

    def get_thumb_speed(self) -> int:       return self._read_reg(REG_RD_CURRENT_THUMB_SPEED)
    def get_thumb_yaw_speed(self) -> int:   return self._read_reg(REG_RD_CURRENT_THUMB_YAW_SPEED)
    def get_index_speed(self) -> int:       return self._read_reg(REG_RD_CURRENT_INDEX_SPEED)
    def get_middle_speed(self) -> int:      return self._read_reg(REG_RD_CURRENT_MIDDLE_SPEED)
    def get_ring_speed(self) -> int:        return self._read_reg(REG_RD_CURRENT_RING_SPEED)
    def get_little_speed(self) -> int:      return self._read_reg(REG_RD_CURRENT_LITTLE_SPEED)

    def get_thumb_temp(self) -> int:        return self._read_reg(REG_RD_THUMB_TEMP)
    def get_thumb_yaw_temp(self) -> int:    return self._read_reg(REG_RD_THUMB_YAW_TEMP)
    def get_index_temp(self) -> int:        return self._read_reg(REG_RD_INDEX_TEMP)
    def get_middle_temp(self) -> int:       return self._read_reg(REG_RD_MIDDLE_TEMP)
    def get_ring_temp(self) -> int:         return self._read_reg(REG_RD_RING_TEMP)
    def get_little_temp(self) -> int:       return self._read_reg(REG_RD_LITTLE_TEMP)

    def get_thumb_error(self) -> int:       return self._read_reg(REG_RD_THUMB_ERROR)
    def get_thumb_yaw_error(self) -> int:   return self._read_reg(REG_RD_THUMB_YAW_ERROR)
    def get_index_error(self) -> int:       return self._read_reg(REG_RD_INDEX_ERROR)
    def get_middle_error(self) -> int:      return self._read_reg(REG_RD_MIDDLE_ERROR)
    def get_ring_error(self) -> int:        return self._read_reg(REG_RD_RING_ERROR)
    def get_little_error(self) -> int:      return self._read_reg(REG_RD_LITTLE_ERROR)

    def get_hand_freedom(self) -> int:      return self._read_reg(REG_RD_HAND_FREEDOM)
    def get_hand_version(self) -> int:      return self._read_reg(REG_RD_HAND_VERSION)
    def get_hand_number(self) -> int:       return self._read_reg(REG_RD_HAND_NUMBER)
    def get_hand_direction(self) -> int:    return self._read_reg(REG_RD_HAND_DIRECTION)
    def get_software_version(self) -> int:  return self._read_reg(REG_RD_SOFTWARE_VERSION)
    def get_hardware_version(self) -> int:  return self._read_reg(REG_RD_HARDWARE_VERSION)
    
    # ----------------------------------------------------------
    # 批量 Getter (使用 read_all_... 方法)
    # ----------------------------------------------------------
    def get_state(self) -> List[int]:
        """获取手指电机状态 (角度)"""
        return self.read_all_angles()

    def get_torque(self) -> List[int]:
        """获取当前扭矩"""
        return self.read_all_torques()

    def get_speed(self) -> List[int]:
        """获取当前速度"""
        return self.read_all_speeds()

    def get_temperature(self) -> List[int]:
        """获取当前电机温度"""
        return self.read_all_temperatures()
    
    def get_fault(self) -> List[int]:
        """获取当前电机故障码"""
        return self.read_all_errors()
    
    def get_version(self) -> list:
        """获取当前固件版本号"""
        return self.read_all_versions()


    # ----------------------------------------------------------
    # 写保持寄存器 (单个寄存器写入)
    # ----------------------------------------------------------
    def _write_reg(self, addr: int, value: int):
        """写单个保持寄存器（功能码 16），带 30 ms 帧间隔"""
        if not 0 <= value <= 255:
            raise ValueError("value must be 0-255")
        
        # 确保 value 是 Python 原生 int
        self._execute_write(addr, [int(value)])

    def _write_regs(self, addr: int, values: List[int]):
        """写多个保持寄存器（功能码 16），带 30 ms 帧间隔"""
        # 此时 values 应该已经是经过 is_valid_6xuint8 验证并转换的 Python int 列表
        if not all(0 <= v <= 255 for v in values):
            # 这行理论上不应触发，因为上层调用已校验
            raise ValueError("All values must be 0-255")
        self._execute_write(addr, values)


    def set_thumb_pitch(self, v: int):           self._write_reg(REG_WR_THUMB_PITCH, v)
    def set_thumb_yaw(self, v: int):             self._write_reg(REG_WR_THUMB_YAW, v)
    def set_index_pitch(self, v: int):           self._write_reg(REG_WR_INDEX_PITCH, v)
    def set_middle_pitch(self, v: int):          self._write_reg(REG_WR_MIDDLE_PITCH, v)
    def set_ring_pitch(self, v: int):            self._write_reg(REG_WR_RING_PITCH, v)
    def set_little_pitch(self, v: int):          self._write_reg(REG_WR_LITTLE_PITCH, v)

    def set_thumb_torque(self, v: int):          self._write_reg(REG_WR_THUMB_TORQUE, v)
    def set_thumb_yaw_torque(self, v: int):      self._write_reg(REG_WR_THUMB_YAW_TORQUE, v)
    def set_index_torque(self, v: int):          self._write_reg(REG_WR_INDEX_TORQUE, v)
    def set_middle_torque(self, v: int):         self._write_reg(REG_WR_MIDDLE_TORQUE, v)
    def set_ring_torque(self, v: int):           self._write_reg(REG_WR_RING_TORQUE, v)
    def set_little_torque(self, v: int):         self._write_reg(REG_WR_LITTLE_TORQUE, v)

    def set_thumb_speed(self, v: int):           self._write_reg(REG_WR_THUMB_SPEED, v)
    def set_thumb_yaw_speed(self, v: int):       self._write_reg(REG_WR_THUMB_YAW_SPEED, v)
    def set_index_speed(self, v: int):           self._write_reg(REG_WR_INDEX_SPEED, v)
    def set_middle_speed(self, v: int):          self._write_reg(REG_WR_MIDDLE_SPEED, v)
    def set_ring_speed(self, v: int):            self._write_reg(REG_WR_RING_SPEED, v)
    def set_little_speed(self, v: int):          self._write_reg(REG_WR_LITTLE_SPEED, v)

    # ----------------------------------------------------------
    # 固定函数 (采用批量写入优化)
    # ----------------------------------------------------------
    def is_valid_6xuint8(self, lst: List[Any]) -> bool:
        """
        验证6个0-255的整数列表。
        允许输入包含浮点数、NumPy整数等可转换为 int 的类型，并进行范围校验。
        """
        if not (isinstance(lst, list) and len(lst) == 6):
            return False
            
        try:
            # 关键：尝试将所有元素转换为 Python 原生 int
            int_values = [int(v) for v in lst]
        except (ValueError, TypeError):
            # 转换失败，列表中包含不可转换的元素
            return False

        # 校验转换后的整数列表是否在 0-255 范围内
        return all(0 <= x <= 255 for x in int_values)
    
    def set_joint_positions(self, joint_angles: List[Any] = None):
        joint_angles = joint_angles or [0] * 6
        
        if not self.is_valid_6xuint8(joint_angles):
            logging.error(f"Invalid joint angles received: {joint_angles}")
            raise ValueError("Joint angles must be a list of 6 values between 0 and 255 (convertible to int).")

        # 强制转换为 Modbus 兼容的 Python 原生 int 列表
        int_angles = [int(v) for v in joint_angles] 
        
        # 批量写入 6 个角度寄存器 (从 REG_WR_THUMB_PITCH 地址 0 开始, count=6)
        self._write_regs(REG_WR_THUMB_PITCH, int_angles)

    def set_speed(self, speed: List[Any] = None):
        speed = speed or [200] * 6
        if not self.is_valid_6xuint8(speed):
            logging.error(f"Invalid speed values received: {speed}")
            raise ValueError("Speed values must be a list of 6 values between 0 and 255 (convertible to int).")
        
        int_speed = [int(v) for v in speed]
        self._write_regs(REG_WR_THUMB_SPEED, int_speed)
    
    def set_torque(self, torque: List[Any] = None):
        torque = torque or [200] * 6
        if not self.is_valid_6xuint8(torque):
            logging.error(f"Invalid torque values received: {torque}")
            raise ValueError("Torque values must be a list of 6 values between 0 and 255 (convertible to int).")
            
        int_torque = [int(v) for v in torque]
        self._write_regs(REG_WR_THUMB_TORQUE, int_torque)

    # ... (其他固定函数保持不变) ...

    def set_current(self, current: List[int] = None):
        print("当前O6不支持设置电流", flush=True)
        pass

    def get_state_for_pub(self) -> list:
        return self.get_state()

    def get_current_status(self) -> list:
        return self.get_state()
    
    def get_joint_speed(self) -> list:
        return self.get_speed()
    
    def get_touch_type(self) -> list:
        return -1
    
    def get_normal_force(self) -> list:
        return [-1] * 5
    
    def get_tangential_force(self) -> list:
        return [-1] * 5
    
    def get_approach_inc(self) -> list:
        return [-1] * 5
    
    def get_touch(self) -> list:
        return [-1] * 5
    
    def get_thumb_matrix_touch(self,sleep_time=0):
        return np.full((12, 6), -1)

    def get_index_matrix_touch(self,sleep_time=0):
        return np.full((12, 6), -1)

    def get_middle_matrix_touch(self,sleep_time=0):
        return np.full((12, 6), -1)

    def get_ring_matrix_touch(self,sleep_time=0):
        return np.full((12, 6), -1)

    def get_little_matrix_touch(self,sleep_time=0):
        return np.full((12, 6), -1)

    def get_matrix_touch(self) -> list:
        thumb_matrix = np.full((12, 6), -1)
        index_matrix = np.full((12, 6), -1)
        middle_matrix = np.full((12, 6), -1)
        ring_matrix = np.full((12, 6), -1)
        little_matrix = np.full((12, 6), -1)
        return thumb_matrix , index_matrix , middle_matrix , ring_matrix , little_matrix
    
    def get_serial_number(self):
        return [0] * 6
    
    def get_matrix_touch_v2(self) -> list:
        return self.get_matrix_touch()
    
    # ----------------------------------------------------------
    # 上下文管理
    # ----------------------------------------------------------
    def close(self):
        if hasattr(self, 'connected') and self.connected:
            self.cli.close()
            self.connected = False
            logging.info("Modbus connection closed.")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # ----------------------------------------------------------
    # 便捷函数
    # ----------------------------------------------------------
    def set_all_fingers(self, pitch: int):
        """同时设置五指弯曲角度（0-255），使用批量写入"""
        # 允许传入 float/numpy int 等可转换为 int 的类型
        try:
            pitch_int = int(pitch)
        except (ValueError, TypeError):
             raise ValueError("Pitch value must be a number convertible to int (0-255)")

        if not 0 <= pitch_int <= 255:
            raise ValueError("Pitch value must be 0-255")
        
        # 批量设置所有 6 个关节的角度
        self.set_joint_positions([pitch_int] * 6)

    def relax(self):
        """全部手指伸直（255）"""
        self.set_all_fingers(255)

    def fist(self):
        """全部手指弯曲（0）"""
        self.set_all_fingers(0)

    def dump_status(self):
        """打印当前所有可读状态 (使用批量读取优化)"""
        print("--------- O6 Hand Status ---------")
        
        angles = self.get_state()
        temps = self.get_temperature()
        errors = self.get_fault()
        versions = self.get_version()

        print(f"Joint Angles: {angles}")
        print(f"Temperature:  {temps}℃")
        print(f"Error Codes:  {errors}")
        print(f"Versions:     {versions}")
        print("----------------------------------")

# ------------------------------------------------------------------
# 命令行快速测试
# ------------------------------------------------------------------
if __name__ == "__main__":
    import argparse
    
    # 假设默认站号是 0x27 (39)
    DEFAULT_HAND_ID = 0x27

    parser = argparse.ArgumentParser(description="O6 Hand Modbus tester (using pymodbus 3.5.1)")
    parser.add_argument("-p", "--port", required=True, help="串口, 如 /dev/ttyUSB0")
    parser.add_argument("-l", "--left", action="store_const", const=0x28, default=DEFAULT_HAND_ID, dest='hand_id', help="左手 (0x28)，默认右手 (0x27)")
    
    args = parser.parse_args()

    try:
        # 使用 with 语句确保连接关闭，这是 pymodbus 的推荐用法
        with LinkerHandO6RS485(hand_id=args.hand_id, modbus_port=args.port, baudrate=115200) as hand:
            hand.dump_status()
            print("执行 relax → 伸直")
            hand.relax()
            time.sleep(1)
            print("执行 fist → 握拳")
            hand.fist()
            time.sleep(1)
            hand.relax()
            print("演示完成")
            
    except ConnectionError as e:
        print(f"连接错误: {e}")
    except RuntimeError as e:
        print(f"Modbus 运行时错误: {e}")
    except StructError as e:
        print(f"数据结构错误 (请检查输入数据类型是否为原生int): {e}")
    except Exception as e:
        print(f"发生其他错误: {e}")