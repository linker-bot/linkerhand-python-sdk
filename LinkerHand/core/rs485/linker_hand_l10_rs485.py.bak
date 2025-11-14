#!/usr/bin/env python3
import os, time
from pymodbus.client import ModbusSerialClient
from typing import List
import numpy as np
_INTERVAL = 0.008  # 8 ms
class LinkerHandL10RS485:

    # 10 电机顺序固定
    KEYS = ["thumb_cmc_pitch", "thumb_cmc_roll", "index_mcp_pitch", "middle_mcp_pitch",
            "ring_mcp_pitch", "pinky_mcp_pitch", "index_mcp_roll", "ring_mcp_roll",
            "pinky_mcp_roll", "thumb_cmc_yaw"]

    def __init__(self, hand_id=0x27, modbus_port="/dev/ttyUSB0", baudrate=115200):
        self.slave = hand_id
        self.cli = ModbusSerialClient(port=modbus_port, baudrate=baudrate,
                                      bytesize=8, parity="N", stopbits=1)
        if not self.cli.connect():
            raise ConnectionError("RS485 connect fail")

    # --------------------------------------------------
    # 批量接口
    # --------------------------------------------------
    # ------------------ 读取 ------------------
    def read_angles(self) -> List[int]:
        time.sleep(_INTERVAL)
        return self.cli.read_input_registers(0, count=10, device_id=self.slave).registers

    def read_torques(self) -> List[int]:
        time.sleep(_INTERVAL)
        return self.cli.read_input_registers(10, count=10, device_id=self.slave).registers

    # ---------- 速度 ----------
    def read_speeds(self) -> List[int]:
        """读当前速度（输入 20-29）"""
        time.sleep(_INTERVAL)
        return self.cli.read_input_registers(20, count=10, device_id=self.slave).registers

    # ---------- 温度 ----------
    def read_temperatures(self) -> List[int]:
        """读当前温度（输入 40-49，单位 ℃）"""
        time.sleep(_INTERVAL)
        return self.cli.read_input_registers(40, count=10, device_id=self.slave).registers

    # ---------- 错误码 ----------
    def read_error_codes(self) -> List[int]:
        """读错误码（输入 50-59，0=正常）"""
        time.sleep(_INTERVAL)
        return self.cli.read_input_registers(50, count=10, device_id=self.slave).registers
    
    # ---------- 版本信息 ----------
    def read_versions(self) -> dict:
        """一次性读取所有版本信息（158-163）"""
        time.sleep(_INTERVAL)
        regs = self.cli.read_input_registers(158, count=6, device_id=self.slave).registers
        keys = ["hand_freedom", "hand_version", "hand_number",
                "hand_direction", "software_version", "hardware_version"]
        return regs

    # --------------------------------------------------
    # 5 个压力传感器单独接口
    # --------------------------------------------------
    def read_pressure_thumb(self) -> np.ndarray:
        return np.array(self._pressure(1), dtype=np.uint8)

    def read_pressure_index(self) -> np.ndarray:
        return np.array(self._pressure(2), dtype=np.uint8)

    def read_pressure_middle(self) -> np.ndarray:
        return np.array(self._pressure(3), dtype=np.uint8)

    def read_pressure_ring(self) -> np.ndarray:
        return np.array(self._pressure(4), dtype=np.uint8)

    def read_pressure_pinky(self) -> np.ndarray:
        return np.array(self._pressure(5), dtype=np.uint8)

    def _pressure(self, finger: int) -> List[int]:
        """内部：选手指 → 读 62-157 共 96 字节"""
        time.sleep(_INTERVAL)
        self.cli.write_register(60, finger, device_id=self.slave)
        time.sleep(_INTERVAL)
        return np.array(self.cli.read_input_registers(62, count=96, device_id=self.slave).registers)
    

    # ------------------ 写入 ------------------
    def write_angles(self, vals: List[int]):
        """设置 10 个电机角度（保持 0-9）"""
        vals = [int(x) for x in vals]
        time.sleep(_INTERVAL)
        self.cli.write_registers(0, values=vals, device_id=self.slave)

    
    def write_speeds(self, vals: List[int]) -> None:
        """设置 10 个电机速度（保持 20-29）"""
        vals = [int(x) for x in vals]
        assert self.is_valid_10xuint8(vals), "需要 10 个 0-255 整数"
        time.sleep(_INTERVAL)
        self.cli.write_registers(20, values=vals, device_id=self.slave)

    def write_torques(self, vals: List[int]) -> None:
        """设置 10 个电机转矩（保持 10-19）"""
        vals = [int(x) for x in vals]
        assert self.is_valid_10xuint8(vals), "需要 10 个 0-255 整数"
        time.sleep(_INTERVAL)
        self.cli.write_registers(10, values=vals, device_id=self.slave)

    # --------------------------------------------------
    # 上下文
    # --------------------------------------------------
    def close(self): self.cli.close()
    def __enter__(self): return self
    def __exit__(self, *_): self.close()

    # --------------------------------------------------
    # API固定接口函数
    # --------------------------------------------------
    def is_valid_10xuint8(self, lst) -> bool:
        if len(lst) < 10:
            lst = [lst[0]] * 10
        lst = [int(x) for x in lst]
        return (
            isinstance(lst, list) and
            len(lst) == 10 and
            all(type(x) is int and 0 <= x <= 255 for x in lst)
        )
    
    def set_joint_positions(self, joint_angles=[0] * 10):
        if self.is_valid_10xuint8(joint_angles):
            self.write_angles(joint_angles)
        else:
            raise ValueError("invalid angles")

    def set_speed(self, speed=[200] * 10):
        """设置速度 params: list len=10"""
        self.write_speeds(speed)
            
    
    def set_torque(self, torque=[200] * 10):
        """设置扭矩 params: list len=10"""
        self.write_torques(torque)

    def set_current(self, current=[200] * 10):
        """设置电流 params: list len=10"""
        print("当前L10不支持设置电流", flush=True)
        pass

    def get_version(self) -> list:
        """获取当前固件版本号"""
        return self.read_versions()
    def get_current(self):
        """获取电流"""
        print("当前L10不支持获取电流", flush=True)
        pass

    def get_state(self) -> list:
        """获取手指电机状态"""
        return self.read_angles()
    
    def get_state_for_pub(self) -> list:
        return self.get_state()

    def get_current_status(self) -> list:
        return self.get_state()
    
    def get_speed(self) -> list:
        """获取当前速度"""
        return self.read_speeds()
    
    def get_joint_speed(self) -> list:
        return self.get_speed()
    
    def get_touch_type(self) -> list:
        """获取压感类型"""
        return 2
    
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
        
    def get_thumb_matrix_touch(self):
        return self._pressure(finger=1)

    def get_index_matrix_touch(self):
        return self._pressure(finger=2)

    def get_middle_matrix_touch(self):
        return self._pressure(finger=3)

    def get_ring_matrix_touch(self):
        return self._pressure(finger=4)

    def get_little_matrix_touch(self):
        return self._pressure(finger=5)
        
    def get_matrix_touch(self) -> list:
        """获取压感数据：矩阵式"""
        return [self._pressure(finger=1), self._pressure(finger=2), self._pressure(finger=3), self._pressure(finger=4), self._pressure(finger=5)]
    
    def get_matrix_touch_v2(self) -> list:
        """获取压感数据：矩阵式"""
        return self.get_matrix_touch()
    
    def get_torque(self) -> list:
        """获取当前扭矩"""
        return self.read_torques()
    
    def get_temperature(self) -> list:
        """获取当前电机温度"""
        return self.read_temperatures()
    
    def get_fault(self) -> list:
        """获取当前电机故障码"""
        return self.read_error_codes()


# ------------------- demo -------------------
if __name__ == "__main__":
    with LinkerHandL10RS485(hand_id=0x27, modbus_port="/dev/ttyUSB0", baudrate=115200) as hand:
        angles = hand.read_angles()
        print(dict(zip(LinkerHandL10RS485.KEYS, angles)))

        # 批量设角度
        hand.write_angles([250]*10)

        # 读拇指压力
        print("thumb pressure:", hand.read_pressure_thumb())
        print("角度 :", dict(zip(LinkerHandL10RS485.KEYS, hand.read_angles())))
        print("电流 :", dict(zip(LinkerHandL10RS485.KEYS, hand.read_torques())))
        print("速度 :", dict(zip(LinkerHandL10RS485.KEYS, hand.read_speeds())))
        print("温度 :", dict(zip(LinkerHandL10RS485.KEYS, hand.read_temperatures())))
        print("错误码:", dict(zip(LinkerHandL10RS485.KEYS, hand.read_error_codes())))
        print("+=" * 30)
        ver = hand.get_version()
        print(list(ver))
