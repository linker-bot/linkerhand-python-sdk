#!/usr/bin/env python3
import os
import time
from pymodbus.client import ModbusSerialClient
from typing import List, Dict
import numpy as np

_INTERVAL = 0.006  # 8 ms

class LinkerHandL6RS485:
    """L6机械手 Modbus-RTU 控制类"""
    
    # 6个关节名称
    JOINT_NAMES = ["thumb_pitch", "thumb_yaw", "index_pitch", 
                   "middle_pitch", "ring_pitch", "little_pitch"]
    
    # 手指名称
    FINGER_NAMES = ["thumb", "index", "middle", "ring", "little"]
    
    def __init__(self, hand_id=0x27, modbus_port="/dev/ttyUSB0", baudrate=115200):
        """
        初始化L6机械手
        hand_id: 右手0x27(39), 左手0x28(40)
        modbus_port: 串口设备路径
        baudrate: 波特率，固定115200
        """
        self.slave = hand_id
        self.cli = ModbusSerialClient(
            port=modbus_port, 
            baudrate=baudrate,
            bytesize=8, 
            parity="N", 
            stopbits=1,
            timeout=0.05
        )
        # pymodbus 3.5.1 需要显式连接
        self.connected = self.cli.connect()
        if not self.connected:
            raise ConnectionError(f"RS485连接失败，端口: {modbus_port}")

    def _read_input_registers(self, address: int, count: int) -> List[int]:
        """读取输入寄存器"""
        time.sleep(_INTERVAL)
        result = self.cli.read_input_registers(address=address, count=count, slave=self.slave)
        if result.isError():
            raise RuntimeError(f"读取输入寄存器失败: address={address}, count={count}")
        return result.registers

    def _write_register(self, address: int, value: int):
        """写入单个寄存器"""
        time.sleep(_INTERVAL)
        result = self.cli.write_register(address=address, value=value, slave=self.slave)
        if result.isError():
            raise RuntimeError(f"写入寄存器失败: address={address}, value={value}")

    def _write_registers(self, address: int, values: List[int]):
        """写入多个寄存器"""
        time.sleep(_INTERVAL)
        result = self.cli.write_registers(address=address, values=values, slave=self.slave)
        if result.isError():
            raise RuntimeError(f"写入多个寄存器失败: address={address}, values={values}")

    # --------------------------------------------------
    # 基础读取接口
    # --------------------------------------------------
    
    def read_angles(self) -> List[int]:
        """读取6个关节角度 (输入寄存器 0-5)"""
        return self._read_input_registers(0, 6)

    def read_torques(self) -> List[int]:
        """读取6个关节转矩 (输入寄存器 6-11)"""
        return self._read_input_registers(6, 6)

    def read_speeds(self) -> List[int]:
        """读取6个关节速度 (输入寄存器 12-17)"""
        return self._read_input_registers(12, 6)

    def read_temperatures(self) -> List[int]:
        """读取6个关节温度 (输入寄存器 18-23)"""
        return self._read_input_registers(18, 6)

    def read_error_codes(self) -> List[int]:
        """读取6个关节错误码 (输入寄存器 24-29)"""
        return self._read_input_registers(24, 6)

    # --------------------------------------------------
    # 压力传感器接口
    # --------------------------------------------------
    
    # def _pressure(self, finger: int) -> List[int]:
    #     """内部：选手指 → 读压力数据"""
    #     # 选择手指 (保持寄存器 36)
    #     self._write_register(36, finger)
    #     time.sleep(_INTERVAL)
    #     # 读取压力数据 (输入寄存器 52-122)
    #     return np.array(self._read_input_registers(52, 71))

    def _pressure(self, finger: int) -> np.ndarray:
        """
        6x12 (72点) 矩阵尺寸。
        Modbus 地址 60/62。
        """
        rows = 12  # 12 行
        cols = 6   # 6 列
        finger_size = rows * cols  # 72 个数据点
        
        # modbus 地址和计数
        write_address = 60  # 写入手指选择
        read_address = 62   # 读取压力数据
        read_count = 96     # 读取 96 个寄存器
        skip_count = 10     # 跳过前 10 个校验点
        
        # 0. 参数校验和手指写入值确定
        if finger < 1 or finger > 5:
            raise ValueError(f"无效的手指编号: {finger}。手指编号应在 1 到 5 之间。")
            
        finger_write_value = finger 
        
        # 1. 写入手指选择寄存器 (地址 60)
        time.sleep(0.008)
        wrsp = self.cli.write_register(address=write_address, value=finger_write_value, slave=self.slave)
        if wrsp.isError():
            raise RuntimeError(f"写入手指选择 {finger} 到地址 {write_address} 失败: {wrsp}")

        # 写入后等待片刻
        time.sleep(0.008) 
        
        # 2. 读取地址 62 的数据
        rrsp = self.cli.read_input_registers(address=read_address, count=read_count, slave=self.slave)
        
        if rrsp.isError():
            raise RuntimeError(f"读取地址 {read_address} 压力数据失败: {rrsp}")
            
        registers_16bit: List[int] = rrsp.registers 
        
        # 3. 核心数据处理
        # a. 提取低 8 位数据 (得到 96 个 8 位数据点)
        final_data_96 = [reg_value & 255 for reg_value in registers_16bit]
        
        # b. 跳过前 10 个校验/头部数据点 (得到 86 个有效数据点)
        effective_data = np.array(final_data_96[skip_count:], dtype=np.uint8)
        # c. 截取当前手指的矩阵数据 (从 86 个有效点中截取 72 个点)
        start_idx = 0 
        end_idx = finger_size  # 72
        
        finger_data_flat = effective_data[start_idx:end_idx]
        
        # d. 验证数据长度
        if finger_data_flat.size != finger_size:
            raise ValueError(
                f"数据提取失败。期望 {finger_size} 点 ({rows}x{cols})，"
                f"但仅截取到 {finger_data_flat.size} 点。请检查协议，确认地址 62 是否一次性返回了所有手指数据。"
            )
            
        # e. 重塑为二维矩阵 (12 行 6 列)
        finger_matrix = finger_data_flat.reshape((rows, cols))
                
        return finger_matrix

    def read_pressure_thumb(self) -> np.ndarray:
        """读取大拇指压力数据"""
        return np.array(self._pressure(1), dtype=np.uint8)

    def read_pressure_index(self) -> np.ndarray:
        """读取食指压力数据"""
        return np.array(self._pressure(2), dtype=np.uint8)

    def read_pressure_middle(self) -> np.ndarray:
        """读取中指压力数据"""
        return np.array(self._pressure(3), dtype=np.uint8)

    def read_pressure_ring(self) -> np.ndarray:
        """读取无名指压力数据"""
        return np.array(self._pressure(4), dtype=np.uint8)

    def read_pressure_little(self) -> np.ndarray:
        """读取小拇指压力数据"""
        return np.array(self._pressure(5), dtype=np.uint8)

    # --------------------------------------------------
    # 版本信息接口
    # --------------------------------------------------
    
    def read_versions(self) -> Dict[str, int]:
        """读取版本信息 (输入寄存器 148-155)"""
        result = self._read_input_registers(148, 8)
        
        return {
            "hand_freedom": result[0],
            "hand_version": result[1],
            "hand_number": result[2],
            "hand_direction": result[3],
            "software_version_major": result[4],
            "software_version_minor": result[5] if len(result) > 5 else 0,
            "software_version_revision": result[6] if len(result) > 6 else 0,
            "hardware_version": result[7] if len(result) > 7 else 0
        }

    # --------------------------------------------------
    # 写入接口
    # --------------------------------------------------
    
    def write_angles(self, vals: List[int]):
        """设置6个关节角度 (保持寄存器 0-5)"""
        vals = [int(x) for x in vals]
        if not self.is_valid_6xuint8(vals):
            raise ValueError("需要6个0-255的整数")
        self._write_registers(0, vals)

    def write_torques(self, vals: List[int]):
        """设置6个关节转矩 (保持寄存器 6-11)"""
        vals = [int(x) for x in vals]
        if not self.is_valid_6xuint8(vals):
            raise ValueError("需要6个0-255的整数")
        self._write_registers(6, vals)

    def write_speeds(self, vals: List[int]):
        """设置6个关节速度 (保持寄存器 12-17)"""
        vals = [int(x) for x in vals]
        if not self.is_valid_6xuint8(vals):
            raise ValueError("需要6个0-255的整数")
        self._write_registers(12, vals)

    # --------------------------------------------------
    # 上下文管理
    # --------------------------------------------------
    
    def close(self):
        """关闭连接"""
        if self.connected:
            self.cli.close()
            self.connected = False
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # --------------------------------------------------
    # API固定接口函数
    # --------------------------------------------------
    
    def is_valid_6xuint8(self, lst) -> bool:
        """验证6个0-255的整数列表"""
        if len(lst) != 6:
            return False
        return all(isinstance(x, int) and 0 <= x <= 255 for x in lst)
    
    def set_joint_positions(self, joint_angles=None):
        """设置关节位置"""
        joint_angles = joint_angles or [0] * 6
        self.write_angles(joint_angles)

    def set_speed(self, speed=None):
        """设置速度"""
        speed = speed or [200] * 6
        self.write_speeds(speed)
    
    def set_torque(self, torque=None):
        """设置扭矩"""
        torque = torque or [200] * 6
        self.write_torques(torque)

    def set_current(self, current=None):
        """设置电流 (L6不支持)"""
        print("当前L6不支持设置电流", flush=True)

    def get_version(self) -> list:
        """获取版本信息"""
        versions = self.read_versions()
        return [
            versions.get("hand_freedom", 0),
            versions.get("hand_version", 0),
            versions.get("hand_number", 0),
            versions.get("hand_direction", 0),
            versions.get("software_version_major", 0),
            versions.get("hardware_version", 0)
        ]

    def get_current(self):
        """获取电流 (L6不支持)"""
        print("当前L6不支持获取电流", flush=True)
        return []

    def get_state(self) -> list:
        """获取关节状态"""
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
    
    def get_touch_type(self) -> int:
        """获取压感类型 (2=矩阵式)"""
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

    def get_thumb_matrix_touch(self,sleep_time=0):
        return self._pressure(1)

    def get_index_matrix_touch(self,sleep_time=0):
        return self._pressure(2)

    def get_middle_matrix_touch(self,sleep_time=0):
        return self._pressure(3)

    def get_ring_matrix_touch(self,sleep_time=0):
        return self._pressure(4)

    def get_little_matrix_touch(self,sleep_time=0):
        return self._pressure(5)
        
    def get_matrix_touch(self) -> list:
        """获取压感数据：矩阵式"""
        return [self._pressure(1), self._pressure(2), self._pressure(3), 
                self._pressure(4), self._pressure(5)]
    
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

    # --------------------------------------------------
    # 便捷方法
    # --------------------------------------------------
    
    def relax(self):
        """所有手指伸直"""
        self.set_joint_positions([255] * 6)
    
    def fist(self):
        """所有手指握拳"""
        self.set_joint_positions([0] * 6)
    
    def dump_status(self):
        """打印状态信息"""
        print("=" * 50)
        print("L6机械手状态信息")
        print("=" * 50)
        
        try:
            # 关节状态
            angles = self.read_angles()
            torques = self.read_torques()
            speeds = self.read_speeds()
            temps = self.read_temperatures()
            errors = self.read_error_codes()
            
            print("关节状态:")
            for i, name in enumerate(self.JOINT_NAMES):
                print(f"  {name:15s}: 角度={angles[i]:3d}, 扭矩={torques[i]:3d}, "
                      f"速度={speeds[i]:3d}, 温度={temps[i]:2d}℃, 错误={errors[i]:2d}")
            
            # 版本信息
            versions = self.read_versions()
            print("\n版本信息:")
            for key, value in versions.items():
                print(f"  {key:20s}: {value}")
            
            # 压力传感器测试
            print("\n压力传感器测试:")
            thumb_pressure = self.read_pressure_thumb()
            print(f"大拇指压力数据长度: {len(thumb_pressure)}")
            
        except Exception as e:
            print(f"读取状态时出错: {e}")
        
        print("=" * 50)


# ------------------- 演示程序 -------------------
if __name__ == "__main__":
    # 使用示例
    try:
        with LinkerHandL6RS485(hand_id=0x27, modbus_port="/dev/ttyUSB0", baudrate=115200) as hand:
            print("连接成功!")
            
            # 打印状态信息
            hand.dump_status()
            
            # 测试基本控制
            print("\n测试控制功能...")
            print("伸直手指...")
            hand.relax()
            time.sleep(2)
            
            print("握拳...")
            hand.fist()
            time.sleep(2)
            
            print("恢复伸直...")
            hand.relax()
            
            # 测试压力传感器
            print("\n测试压力传感器...")
            thumb_matrix = hand.get_thumb_matrix_touch()
            print(f"大拇指压力数据: {len(thumb_matrix)}个点")
            
            # 获取所有手指压力数据
            all_matrices = hand.get_matrix_touch()
            for i, name in enumerate(hand.FINGER_NAMES):
                matrix = all_matrices[i]
                print(f"{name}手指压力数据长度: {len(matrix)}")
            
    except Exception as e:
        print(f"错误: {e}")
