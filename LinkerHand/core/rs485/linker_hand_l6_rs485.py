#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
L6P 机械手 Modbus-RTU 控制类
完全依据《L6-485协议说明》实现
"""
import minimalmodbus
import serial
import time
import logging
from typing import List
import numpy as np

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)-8s %(message)s",
    datefmt="%H:%M:%S"
)

# ------------------------------------------------------------------
# 寄存器地址 —— 与协议完全一致
# ------------------------------------------------------------------
# 输入寄存器（功能码 04，只读）
REG_RD_CURRENT_BASE   = 0   # 0-5  当前角度
REG_RD_TORQUE_BASE    = 6   # 6-11 当前力矩
REG_RD_SPEED_BASE     = 12  # 12-17当前速度
REG_RD_TEMP_BASE      = 18  # 18-23温度
REG_RD_ERR_BASE       = 24  # 24-29错误码
REG_RD_PRESS_ID       = 50  # 压力传感器 ID（0-5）
REG_RD_PRESS_SPEC     = 51  # 规格：高4位=行，低4位=列
REG_RD_PRESS_DATA     = 52  # 数据起始地址，连续 rows*cols 个
REG_RD_INFO_BASE      = 148 # 148-155 硬件信息

# 保持寄存器（功能码 16，读写）
REG_WR_ANGLE_BASE     = 0   # 0-5  目标角度
REG_WR_TORQUE_BASE    = 6   # 6-11 目标力矩
REG_WR_SPEED_BASE     = 12  # 12-17目标速度
REG_WR_LROT_TH_BASE   = 18  # 18-23堵转阈值
REG_WR_LROT_TIME_BASE = 24  # 24-29堵转时间
REG_WR_LROT_TOR_BASE  = 30  # 30-35堵转扭矩
REG_WR_PRESS_SEL      = 36  # 压力传感器选择（0-5）


class LinkerHandL6RS485:
    """L6P 机械手 RS485 控制类（全 list 接口）"""

    # RIGHT_ID = 0x27
    # LEFT_ID  = 0x28
    # BAUD     = 115200
    TTY_TIMEOUT = 0.15
    FRAME_GAP   = 0.030
    _last_ts    = 0

    def __init__(self, hand_id: int = 0x27, modbus_port: str = "/dev/ttyUSB0", baudrate: int = 115200):
        self._id = hand_id
        self.joint_name = ["拇指弯曲", "拇指横摆", "食指", "中指", "无名指", "小指"]
        self.lock = False
        try:
            self.instr = minimalmodbus.Instrument(modbus_port, self._id, mode="rtu")
            self.instr.serial.baudrate = baudrate
            self.instr.serial.bytesize = 8
            self.instr.serial.parity   = serial.PARITY_NONE
            self.instr.serial.stopbits = 1
            self.instr.serial.timeout  = self.TTY_TIMEOUT

            self.instr.close_port_after_each_call = False   # ① 不复用
            self.instr.clear_buffers_before_each_transaction = True  # ② 每次清缓存
        except Exception as e:
            logging.error(f"初始化失败: {e}")
            raise

    # ----------------------- 上下文管理器 -----------------------
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.instr.serial.close()
        except Exception:
            pass

    # ----------------------- 底层封装 -----------------------
    def _bus_free(self):
        elapse = time.perf_counter() - self._last_ts
        if elapse < self.FRAME_GAP:
            time.sleep(self.FRAME_GAP - elapse)
        self.instr.serial.reset_input_buffer()   # ← 关键

    def _read_regs(self, addr: int, count: int) -> List[int]:
        self._bus_free()
        try:
            return self.instr.read_registers(addr, count, functioncode=4)
        finally:
            self._last_ts = time.perf_counter()

    def _write_regs(self, addr: int, values: List[int]):
        if not all(0 <= v <= 255 for v in values):
            raise ValueError("所有值必须在 0-255 之间")
        self.lock = True
        try:
            self._bus_free()
            try:
                self.instr.write_registers(addr, values)
            finally:
                self._last_ts = time.perf_counter()
        except:
            print("撞帧.....")
        self.lock = False

    def _write_reg(self, addr: int, value: int):
        if not 0 <= value <= 255:
            raise ValueError("值必须在 0-255 之间")
        self._bus_free()
        try:
            self.instr.write_register(addr, value, functioncode=16)
        finally:
            self._last_ts = time.perf_counter()

    # ----------------------- 只读 —— 全 list -----------------------
    def get_current_angles(self) -> List[int]:
        return self._read_regs(REG_RD_CURRENT_BASE, 6)

    def get_current_torques(self) -> List[int]:
        return self._read_regs(REG_RD_TORQUE_BASE, 6)

    def get_current_speeds(self) -> List[int]:
        return self._read_regs(REG_RD_SPEED_BASE, 6)

    def get_temperatures(self) -> List[int]:
        return self._read_regs(REG_RD_TEMP_BASE, 6)

    def get_error_codes(self) -> List[int]:
        return self._read_regs(REG_RD_ERR_BASE, 6)

    def get_hand_info(self) -> List[int]:
        """返回 8 个寄存器：自由度、版本、编号、方向、软件主/次/修订、硬件"""
        return self._read_regs(REG_RD_INFO_BASE, 8)

    # ----------------------- 写入 —— 全 list -----------------------
    def set_target_angles(self, angles: List[int]):
        if self.lock == True:
            return
        if len(angles) != 6:
            raise ValueError("需要长度为 6 的列表")
        self.lock = True
        self._write_regs(REG_WR_ANGLE_BASE, angles)
        time.sleep(0.05)
        self.lock = False

    def set_target_torques(self, torques: List[int]):
        self.lock = True
        if len(torques) != 6:
            raise ValueError("需要长度为 6 的列表")
        self._write_regs(REG_WR_TORQUE_BASE, torques)
        time.sleep(1)
        self.lock = False

    def set_target_speeds(self, speeds: List[int]):
        self.lock = True
        if len(speeds) != 6:
            raise ValueError("需要长度为 6 的列表")
        self._write_regs(REG_WR_SPEED_BASE, speeds)
        time.sleep(1)
        self.lock = False

    # ----------------------- 压力传感器 —— 2 维矩阵 -----------------------
    def select_pressure_sensor(self, finger_id: int):
        """0=关闭  1-5 对应拇指到小指"""
        if not 0 <= finger_id <= 5:
            raise ValueError("finger_id 必须在 0-5 之间")
        self._write_reg(REG_WR_PRESS_SEL, finger_id)

    def get_pressure_matrix(self, touch_id: int = None) -> List[List[int]]:
        """返回当前选中传感器的 2 维矩阵 [rows][cols]"""
        if touch_id is not None:
            self.select_pressure_sensor(touch_id)
        id_, spec = self._read_regs(REG_RD_PRESS_ID, 2)  # ← 只读 2 个
        rows = (spec >> 4) & 0xF
        cols = spec & 0xF
        if rows == 0 or cols == 0:
            return []
        flat = self._read_regs(REG_RD_PRESS_DATA, rows * cols)
        return [flat[i * cols:(i + 1) * cols] for i in range(rows)]
    
    def get_pressure_id(self) -> int:
        """返回当前实际选中的压力传感器 ID（0-5）"""
        return self._read_regs(REG_RD_PRESS_ID, 1)[0]

    def get_all_finger_pressure_matrices(self) -> tuple:
        """
        一次性读取所有 5 根手指的压感矩阵
        返回: (thumb, index, middle, ring, little)  # 每个都是 2 维 list
        """
        results = []
        for fid in range(1, 6):               # 1-5 对应拇指到小指
            self.select_pressure_sensor(fid)
            time.sleep(0.006)                  # 留切换时间
            if self.get_pressure_id() != fid:
                logging.warning(f"手指 {fid} 传感器切换失败，返回空矩阵")
                results.append([])
                continue
            mat = self.get_pressure_matrix()
            results.append(mat if mat else [])
        return tuple(results)

    # ----------------------- 堵转参数 —— 全 list -----------------------
    def get_lock_rotor_thresholds(self) -> List[int]:
        return self._read_regs(REG_WR_LROT_TH_BASE, 6)

    def set_lock_rotor_thresholds(self, vals: List[int]):
        if len(vals) != 6:
            raise ValueError("需要长度为 6 的列表")
        self._write_regs(REG_WR_LROT_TH_BASE, vals)

    def get_lock_rotor_times(self) -> List[int]:
        return self._read_regs(REG_WR_LROT_TIME_BASE, 6)

    def set_lock_rotor_times(self, vals: List[int]):
        if len(vals) != 6:
            raise ValueError("需要长度为 6 的列表")
        self._write_regs(REG_WR_LROT_TIME_BASE, vals)

    def get_lock_rotor_torques(self) -> List[int]:
        return self._read_regs(REG_WR_LROT_TOR_BASE, 6)

    def set_lock_rotor_torques(self, vals: List[int]):
        if len(vals) != 6:
            raise ValueError("需要长度为 6 的列表")
        self._write_regs(REG_WR_LROT_TOR_BASE, vals)


    
    def is_valid_6xuint8(self, lst) -> bool:
        lst = [int(x) for x in lst]
        return (
            isinstance(lst, list) and
            len(lst) == 6 and
            all(type(x) is int and 0 <= x <= 255 for x in lst)
        )
    # ----------------------------------------------------------
    # API对应函数
    # ----------------------------------------------------------
    def set_joint_positions(self, joint_angles=[0] * 6):
        if self.is_valid_6xuint8(joint_angles):
            self.set_target_angles(joint_angles)

    def set_speed(self, speed=[200] * 6):
        """设置速度 params: list len=6"""
        if self.is_valid_6xuint8(speed):
            try:
                self.set_target_speeds(speed)
            except:
                pass
    
    def set_torque(self, torque=[200] * 6):
        """设置扭矩 params: list len=6"""
        if self.is_valid_6xuint8(torque):
            self.set_target_torques(torque)
            

    def set_current(self, current=[200] * 6):
        """设置电流 params: list len=6"""
        print("当前L6不支持设置电流", flush=True)
        pass

    def get_version(self) -> list:
        """获取当前固件版本号"""
        return self.get_hand_info()
    
    def get_current(self):
        """获取电流"""
        print("当前O6不支持获取电流", flush=True)
        pass

    def get_state(self) -> list:
        """获取手指电机状态"""
        return self.get_current_angles()
    
    def get_state_for_pub(self) -> list:
        return self.get_state()

    def get_current_status(self) -> list:
        return self.get_state()
    
    def get_speed(self) -> list:
        """获取当前速度"""
        return self.get_current_speeds()
    
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
    
    def get_matrix_touch(self) -> list:
        """获取压感数据：矩阵式"""
        thumb_matrix , index_matrix , middle_matrix , ring_matrix , little_matrix = self.get_all_finger_pressure_matrices()
        return np.asarray(thumb_matrix),np.asarray(index_matrix),np.asarray(middle_matrix),np.asarray(ring_matrix),np.asarray(little_matrix)
    
    def get_matrix_touch_v2(self) -> list:
        """获取压感数据：矩阵式"""
        return self.get_matrix_touch()
    
    def get_torque(self) -> list:
        """获取当前扭矩"""
        return self.get_current_torques()
    
    def get_temperature(self) -> list:
        """获取当前电机温度"""
        return self.get_temperatures()
    
    def get_fault(self) -> list:
        """获取当前电机故障码"""
        return self.get_error_codes()


# ------------------------------------------------------------------
# 快速测试
# ------------------------------------------------------------------
if __name__ == "__main__":
    with LinkerHandL6RS485(hand_id=0x27, modbus_port="/dev/ttyUSB0") as hand:
        print("设置速度:", hand.set_target_speeds([5, 5, 5, 5, 5, 5]))
        print("设置力矩:", hand.set_target_torques([5, 5, 5, 5, 5, 5]))
        hand.set_target_angles([102, 18, 0, 0, 0, 0])
        time.sleep(1)
        print("运动后角度:", hand.get_current_angles())

        # 压力传感器示例
        thumb_matrix , index_matrix , middle_matrix , ring_matrix , little_matrix = hand.get_matrix_touch_v2()
        # print("压力矩阵行×列:", len(mat), "×", len(mat[0]) if mat else 0)
        # for row in mat:
        #     print(row)
        print(index_matrix)