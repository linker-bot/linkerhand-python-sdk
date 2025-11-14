#!/usr/bin/env python3
import os
import time
from typing import List
import numpy as np
from pymodbus.client import ModbusSerialClient
_INTERVAL = 0.008  # 8 ms


class LinkerHandL10RS485:
    KEYS = ["thumb_cmc_pitch", "thumb_cmc_roll", "index_mcp_pitch", "middle_mcp_pitch",
            "ring_mcp_pitch", "pinky_mcp_pitch", "index_mcp_roll", "ring_mcp_roll",
            "pinky_mcp_roll", "thumb_cmc_yaw"]

    def __init__(self, hand_id=0x27, modbus_port="/dev/ttyUSB0", baudrate=115200):
        self.slave = hand_id
        self.cli = ModbusSerialClient(
            port=modbus_port,
            baudrate=baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.05,          # 50 ms 超时
            retries=3,             # 重试次数
            retry_on_empty=True,
            handle_local_echo=False
        )
        # 在 pymodbus 3.5.1 中，连接需要显式调用 connect()
        self.connected = self.cli.connect()
        if not self.connected:
            raise ConnectionError(f"RS485 connect fail to {modbus_port}")

    # --------------------------------------------------
    # 批量读取接口
    # --------------------------------------------------
    def read_angles(self) -> List[int]:
        time.sleep(_INTERVAL)
        rsp = self.cli.read_input_registers(address=0, count=10, slave=self.slave)
        if rsp.isError():
            raise RuntimeError(f"read_angles failed: {rsp}")
        return rsp.registers

    def read_torques(self) -> List[int]:
        time.sleep(_INTERVAL)
        rsp = self.cli.read_input_registers(address=10, count=10, slave=self.slave)
        if rsp.isError():
            raise RuntimeError(f"read_torques failed: {rsp}")
        return rsp.registers

    def read_speeds(self) -> List[int]:
        time.sleep(_INTERVAL)
        rsp = self.cli.read_input_registers(address=20, count=10, slave=self.slave)
        if rsp.isError():
            raise RuntimeError(f"read_speeds failed: {rsp}")
        return rsp.registers

    def read_temperatures(self) -> List[int]:
        time.sleep(_INTERVAL)
        rsp = self.cli.read_input_registers(address=40, count=10, slave=self.slave)
        if rsp.isError():
            raise RuntimeError(f"read_temperatures failed: {rsp}")
        return rsp.registers

    def read_error_codes(self) -> List[int]:
        time.sleep(_INTERVAL)
        rsp = self.cli.read_input_registers(address=50, count=10, slave=self.slave)
        if rsp.isError():
            raise RuntimeError(f"read_error_codes failed: {rsp}")
        return rsp.registers

    def read_versions(self) -> dict:
        time.sleep(_INTERVAL)
        rsp = self.cli.read_input_registers(address=158, count=6, slave=self.slave)
        if rsp.isError():
            raise RuntimeError(f"read_versions failed: {rsp}")
        keys = ["hand_freedom", "hand_version", "hand_number",
                "hand_direction", "software_version", "hardware_version"]
        #return dict(zip(keys, rsp.registers))
        return rsp.registers

    # --------------------------------------------------
    # 5 个压力传感器
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
        time.sleep(_INTERVAL)
        # 先选择手指
        wrsp = self.cli.write_register(address=60, value=finger, slave=self.slave)
        if wrsp.isError():
            raise RuntimeError(f"write finger select {finger} failed: {wrsp}")
        
        time.sleep(_INTERVAL)
        # 读取压力传感器数据 (96个寄存器)
        rrsp = self.cli.read_input_registers(address=62, count=96, slave=self.slave)
        if rrsp.isError():
            raise RuntimeError(f"read pressure finger={finger} failed: {rrsp}")
        return np.array(rrsp.registers, dtype=np.uint8)

    # --------------------------------------------------
    # 批量写入接口
    # --------------------------------------------------
    def write_angles(self, vals: List[int]):
        vals = [int(x) for x in vals]
        if not self.is_valid_10xuint8(vals):
            raise ValueError("需要 10 个 0-255 整数")
        
        time.sleep(_INTERVAL)
        rsp = self.cli.write_registers(address=0, values=vals, slave=self.slave)
        if rsp.isError():
            raise RuntimeError(f"write_angles failed: {rsp}")

    def write_speeds(self, vals: List[int]):
        vals = [int(x) for x in vals]
        if not self.is_valid_10xuint8(vals):
            raise ValueError("需要 10 个 0-255 整数")
            
        time.sleep(_INTERVAL)
        rsp = self.cli.write_registers(address=20, values=vals, slave=self.slave)
        if rsp.isError():
            raise RuntimeError(f"write_speeds failed: {rsp}")

    def write_torques(self, vals: List[int]):
        vals = [int(x) for x in vals]
        if not self.is_valid_10xuint8(vals):
            raise ValueError("需要 10 个 0-255 整数")
            
        time.sleep(_INTERVAL)
        rsp = self.cli.write_registers(address=10, values=vals, slave=self.slave)
        if rsp.isError():
            raise RuntimeError(f"write_torques failed: {rsp}")

    # --------------------------------------------------
    # 上下文管理
    # --------------------------------------------------
    def close(self):
        if self.connected:
            self.cli.close()
            self.connected = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # --------------------------------------------------
    # 工具函数
    # --------------------------------------------------
    def is_valid_10xuint8(self, lst) -> bool:
        if len(lst) != 10:
            return False
        return all(isinstance(x, int) and 0 <= x <= 255 for x in lst)

    # --------------------------------------------------
    # 固定 API 接口
    # --------------------------------------------------
    def set_joint_positions(self, joint_angles=None):
        joint_angles = joint_angles or [0] * 10
        self.write_angles(joint_angles)

    def set_speed(self, speed=None):
        speed = speed or [200] * 10
        self.write_speeds(speed)

    def set_torque(self, torque=None):
        torque = torque or [200] * 10
        self.write_torques(torque)

    def set_current(self, current=None):
        print("当前L10不支持设置电流", flush=True)

    def get_version(self) -> dict:
        return self.read_versions()

    def get_current(self):
        print("当前L10不支持获取电流", flush=True)

    def get_state(self) -> List[int]:
        return self.read_angles()

    def get_state_for_pub(self) -> List[int]:
        return self.get_state()

    def get_current_status(self) -> List[int]:
        return self.get_state()

    def get_speed(self) -> List[int]:
        return self.read_speeds()

    def get_joint_speed(self) -> List[int]:
        return self.get_speed()

    def get_touch_type(self) -> int:
        return 2

    def get_normal_force(self) -> List[int]:
        return [-1] * 5

    def get_tangential_force(self) -> List[int]:
        return [-1] * 5

    def get_approach_inc(self) -> List[int]:
        return [-1] * 5

    def get_touch(self) -> List[int]:
        return [-1] * 5

    def get_thumb_matrix_touch(self):
        return self._pressure(1)

    def get_index_matrix_touch(self):
        return self._pressure(2)

    def get_middle_matrix_touch(self):
        return self._pressure(3)

    def get_ring_matrix_touch(self):
        return self._pressure(4)

    def get_little_matrix_touch(self):
        return self._pressure(5)

    def get_matrix_touch(self) -> List[List[int]]:
        return self.get_thumb_matrix_touch(),self.get_index_matrix_touch(), self.get_middle_matrix_touch(), self.get_ring_matrix_touch(), self.get_little_matrix_touch()

    def get_matrix_touch_v2(self) -> List[List[int]]:
        return self.get_matrix_touch()

    def get_torque(self) -> List[int]:
        return self.read_torques()

    def get_temperature(self) -> List[int]:
        return self.read_temperatures()

    def get_fault(self) -> List[int]:
        return self.read_error_codes()


# ------------------- demo -------------------
if __name__ == "__main__":
    try:
        with LinkerHandL10RS485(hand_id=0x27, modbus_port="/dev/ttyUSB0", baudrate=115200) as hand:
            print("连接成功!")
            
            # 测试读取角度
            angles = hand.read_angles()
            print("角度:", dict(zip(LinkerHandL10RS485.KEYS, angles)))
            
            # 测试读取版本信息
            ver = hand.get_version()
            print("版本信息:", ver)
            
            # 测试压力传感器
            print("拇指压力传感器数据长度:", len(hand.read_pressure_thumb()))
            
            # 测试其他读取功能
            print("电流:", hand.read_torques())
            print("速度:", hand.read_speeds())
            print("温度:", hand.read_temperatures())
            print("错误码:", hand.read_error_codes())
            
    except Exception as e:
        print(f"错误: {e}")