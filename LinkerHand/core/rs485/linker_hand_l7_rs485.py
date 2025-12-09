#!/usr/bin/env python3
import time
from typing import List, Dict, Union
import numpy as np
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException

# --- 协议常量和寄存器地址定义 (根据 O7 协议文件) ---

# RS485 通信设置
DEFAULT_BAUDRATE = 115200

# O7机械手七个可控关节的键名 (根据保持寄存器和输入寄存器地址 0-6)
O7_JOINT_KEYS = [
    "Thumb_Pitch", "Thumb_Yaw", "Index_Pitch", "Middle_Pitch",
    "Ring_Pitch", "Little_Pitch", "Thumb_Roll"
]

# 保持寄存器地址 (写操作 FC 16)
HR_ADDR = {
    "Position_Start": 0,    # 关节目标位置 (7 个寄存器: 0-6)
    "Torque_Start": 7,      # 关节目标转矩 (7 个寄存器: 7-13)
    "Speed_Start": 14,      # 关节目标速度 (7 个寄存器: 14-20)
    "Pressure_Select": 42   # 压力传感器数据选择 (1 个寄存器)
    # 21-41 为堵转保护阈值、时间和扭矩，暂未实现
}

# 输入寄存器地址 (读操作 FC 04)
IR_ADDR = {
    "Current_Position_Start": 0,    # 当前关节位置 (7 个寄存器: 0-6)
    "Current_Torque_Start": 7,      # 当前关节转矩 (7 个寄存器: 7-13)
    "Current_Speed_Start": 14,      # 当前关节速度 (7 个寄存器: 14-20)
    "Current_Temperature_Start": 21, # 当前关节温度 (7 个寄存器: 21-27)
    "Error_Code_Start": 28,         # 当前关节错误码 (7 个寄存器: 28-34)
    "Tip_Force_Start": 35,          # 指尖力数据 (20 个寄存器: 35-54)
    "Pressure_Data_Start": 57,      # 压力传感器数据起始 (96 个寄存器: 57-152)
    "Version_Start": 153            # 版本信息 (6 个寄存器: 153-158)
}

# 辅助常量
_JOINT_COUNT = 7
_VERSION_COUNT = 6
_TIP_FORCE_COUNT = 20
_PRESSURE_REG_COUNT = 96 
_PRESSURE_ROWS = 12 # 从 IR 56 (0xC6) 推断
_PRESSURE_COLS = 6  # 从 IR 56 (0xC6) 推断
_PRESSURE_DATA_SIZE = _PRESSURE_ROWS * _PRESSURE_COLS # 72
_PRESSURE_HEADER_SKIP = 10 # 假设跳过 10 个头部/校验字节

# 通信间隔时间 (使用 L10 参考中的 5ms)
_INTERVAL = 0.005 


class LinkerHandL7RS485:
    """
    O7机械手 Modbus RTU (RS485) 控制类。
    使用 pymodbus 3.5.1 版本和 O7 机械手协议。
    """
    def __init__(self, 
                 hand_id: int = 0x27, 
                 modbus_port: str = "/dev/ttyUSB0", 
                 baudrate: int = DEFAULT_BAUDRATE,
                 timeout: float = 0.05):
        """
        初始化 Modbus 客户端。

        :param hand_id: Modbus 从站地址 (0x27: 右手, 0x28: 左手)
        :param modbus_port: 串口名称
        :param baudrate: 波特率 (默认为 115200)
        :param timeout: 通信超时时间 (秒)
        """
        self.slave = hand_id
        self.cli = ModbusSerialClient(
            port=modbus_port,
            baudrate=baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,           # 确保与 pymodbus 3.x 兼容的写法
            timeout=timeout,
            retries=3,            # 重试次数
            retry_on_empty=True,
            handle_local_echo=False,
            method='rtu'
        )

        # 尝试连接
        self.connected = self.cli.connect()
        if not self.connected:
            raise ConnectionError(f"RS485 connect fail to {modbus_port} with ID {hex(hand_id)}.")
        
        print(f"O7机械手 Modbus ID {hex(hand_id)} 连接成功到 {modbus_port}。")


    # --------------------------------------------------
    # 核心读写函数 (基于 pymodbus 3.5.1)
    # --------------------------------------------------

    def _read_input_registers(self, address: int, count: int) -> List[int]:
        """封装 Modbus 读取输入寄存器 (FC 04) 操作。"""
        time.sleep(_INTERVAL)
        try:
            rsp = self.cli.read_input_registers(
                address=address, 
                count=count, 
                slave=self.slave
            )
            
            # 使用 L10 参考中验证过的 3.x 兼容错误检查
            if rsp.isError():
                raise RuntimeError(f"Modbus FC04 读取失败。地址: {address}, 错误: {rsp}")
                
            return rsp.registers
            
        except ModbusException as e:
            # 捕获通信超时、CRC 错误等 Modbus 异常
            raise RuntimeError(f"Modbus 通信异常。地址: {address}, 错误: {e}")
        except Exception as e:
            raise RuntimeError(f"未知读取异常。地址: {address}, 错误: {e}")

    def _write_holding_registers(self, address: int, values: List[int]):
        """封装 Modbus 写入保持寄存器 (FC 16) 操作。"""
        time.sleep(_INTERVAL)
        
        # 批量写入 (FC 16)
        if len(values) > 1:
            write_func = self.cli.write_registers
        # 单个写入 (FC 06)
        elif len(values) == 1:
            write_func = lambda address, values, slave: self.cli.write_register(address, values[0], slave)
        else:
             raise ValueError("写入值列表不能为空。")

        try:
            rsp = write_func(
                address=address, 
                values=values, 
                slave=self.slave
            )
            
            if rsp.isError():
                raise RuntimeError(f"Modbus FC16 写入失败。地址: {address}, 错误: {rsp}")
                
        except ModbusException as e:
            raise RuntimeError(f"Modbus 通信异常。地址: {address}, 错误: {e}")
        except Exception as e:
            raise RuntimeError(f"未知写入异常。地址: {address}, 错误: {e}")

    # --------------------------------------------------
    # 读操作 (Read API)
    # --------------------------------------------------
    
    def get_joint_positions(self) -> Dict[str, int]:
        """读取当前关节位置 (地址 0-6)。"""
        registers = self._read_input_registers(IR_ADDR["Current_Position_Start"], _JOINT_COUNT)
        #return dict(zip(O7_JOINT_KEYS, registers))
        return registers

    def get_current_torques(self) -> Dict[str, int]:
        """读取当前关节转矩 (地址 7-13)。"""
        registers = self._read_input_registers(IR_ADDR["Current_Torque_Start"], _JOINT_COUNT)
        #return dict(zip(O7_JOINT_KEYS, registers))
        return registers

    def get_current_speeds(self) -> Dict[str, int]:
        """读取当前关节速度 (地址 14-20)。"""
        registers = self._read_input_registers(IR_ADDR["Current_Speed_Start"], _JOINT_COUNT)
        #return dict(zip(O7_JOINT_KEYS, registers))
        return registers

    def get_temperatures(self) -> Dict[str, int]:
        """读取当前关节温度 (地址 21-27)。"""
        registers = self._read_input_registers(IR_ADDR["Current_Temperature_Start"], _JOINT_COUNT)
        #return dict(zip(O7_JOINT_KEYS, registers))
        return registers

    def get_error_codes(self) -> Dict[str, int]:
        """读取当前关节错误码 (地址 28-34)。"""
        registers = self._read_input_registers(IR_ADDR["Error_Code_Start"], _JOINT_COUNT)
        #return dict(zip(O7_JOINT_KEYS, registers))
        return registers

    def get_tip_forces(self) -> List[int]:
        """读取指尖法向力、切向力等数据 (地址 35-54)。"""
        return self._read_input_registers(IR_ADDR["Tip_Force_Start"], _TIP_FORCE_COUNT)
        
    def get_version(self) -> List[int]:
        """读取版本信息 (地址 153-158)。"""
        return self._read_input_registers(IR_ADDR["Version_Start"], _VERSION_COUNT)

    def get_pressure_matrix(self, finger_id: int) -> np.ndarray:
        """
        读取特定手指的压力传感器数据矩阵。
        
        :param finger_id: 手指编号 (1: 大拇指, 2: 食指, 3: 中指, 4: 无名指, 5: 小拇指)
        :return: 12x6 的压力数据矩阵 (np.ndarray)
        """
        if not (1 <= finger_id <= 5):
            raise ValueError(f"无效的手指编号: {finger_id}。应在 1 到 5 之间。")
            
        # 1. 写入手指选择寄存器 (HR 42)
        # 使用单个写入 (FC 06)
        self._write_holding_registers(HR_ADDR["Pressure_Select"], [finger_id])

        # 2. 读取压力传感器数据 (IR 57, 96 个寄存器)
        time.sleep(_INTERVAL) # 等待数据更新
        registers_16bit: List[int] = self._read_input_registers(
            IR_ADDR["Pressure_Data_Start"], 
            _PRESSURE_REG_COUNT
        )
        
        # 3. 数据解析 (假设与 L10 类似的数据格式: 低 8 位有效，有头部数据)
        
        # a. 提取低 8 位数据 (得到 96 个 8 位数据点)
        final_data_96 = [reg_value & 255 for reg_value in registers_16bit]
        
        # b. 跳过头部数据点
        effective_data = np.array(final_data_96[_PRESSURE_HEADER_SKIP:], dtype=np.uint8)
        
        # c. 截取当前手指的矩阵数据 (72 个点)
        finger_data_flat = effective_data[:_PRESSURE_DATA_SIZE]
        
        if finger_data_flat.size != _PRESSURE_DATA_SIZE:
             raise ValueError(
                f"压力数据提取失败。期望 {_PRESSURE_DATA_SIZE} 点，"
                f"但仅截取到 {finger_data_flat.size} 点。请检查协议解析逻辑。"
            )
            
        # d. 重塑为二维矩阵 (12 行 6 列)
        finger_matrix = finger_data_flat.reshape((_PRESSURE_ROWS, _PRESSURE_COLS))
        return finger_matrix


    # --------------------------------------------------
    # 写操作 (Write API)
    # --------------------------------------------------

    def set_joint_positions(self, joint_angles: List[int]):
        """
        设置所有 7 个关节的目标位置 (地址 0-6)。
        :param joint_angles: 7 个 0-255 的整数值列表
        """
        if len(joint_angles) != _JOINT_COUNT:
            raise ValueError(f"需要 {_JOINT_COUNT} 个关节位置值，提供了 {len(joint_angles)} 个。")
        self._write_holding_registers(HR_ADDR["Position_Start"], joint_angles)

    def set_torques(self, torques: List[int]):
        """
        设置所有 7 个关节的目标转矩 (地址 7-13)。
        :param torques: 7 个 0-255 的整数值列表
        """
        if len(torques) != _JOINT_COUNT:
            raise ValueError(f"需要 {_JOINT_COUNT} 个关节转矩值，提供了 {len(torques)} 个。")
        self._write_holding_registers(HR_ADDR["Torque_Start"], torques)


    def set_speeds(self, speeds: List[int]):
        """
        设置所有 7 个关节的目标速度 (地址 14-20)。
        :param speeds: 7 个 0-255 的整数值列表
        """
        if len(speeds) != _JOINT_COUNT:
            raise ValueError(f"需要 {_JOINT_COUNT} 个关节速度值，提供了 {len(speeds)} 个。")
        self._write_holding_registers(HR_ADDR["Speed_Start"], speeds)
    
    def set_speed(self, speed:List[int] = [200] * 7):
        self.set_speeds(speed)

    def set_torque(self, torque: List[int] = [250] * 7):
        self.set_torques(torque)

    def set_current(self, current=None):
        print("当前L7不支持设置电流", flush=True)

    def get_current(self):
        #print("当前L7不支持获取电流", flush=True)
        return [-1] * 7

    def get_state(self) -> List[int]:
        return self.get_joint_positions()
    

    def get_state_for_pub(self) -> List[int]:
        return self.get_joint_positions()

    def get_current_status(self) -> List[int]:
        return self.get_joint_positions()

    def get_speed(self) -> List[int]:
        return self.get_current_speeds()

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

    def get_thumb_matrix_touch(self,sleep_time=0):
        return self.get_pressure_matrix(finger_id=1)

    def get_index_matrix_touch(self,sleep_time=0):
        return self.get_pressure_matrix(finger_id=2)

    def get_middle_matrix_touch(self,sleep_time=0):
        return self.get_pressure_matrix(finger_id=3)

    def get_ring_matrix_touch(self,sleep_time=0):
        return self.get_pressure_matrix(finger_id=4)

    def get_little_matrix_touch(self,sleep_time=0):
        return self.get_pressure_matrix(finger_id=5)

    def get_matrix_touch(self) -> List[List[int]]:
        return self.get_thumb_matrix_touch(),self.get_index_matrix_touch(), self.get_middle_matrix_touch(), self.get_ring_matrix_touch(), self.get_little_matrix_touch()

    def get_matrix_touch_v2(self) -> List[List[int]]:
        return self.get_matrix_touch()

    def get_torque(self) -> List[int]:
        return self.get_current_torques()

    def get_temperature(self) -> List[int]:
        return self.get_temperatures()

    def get_fault(self) -> List[int]:
        return self.get_error_codes()

    def get_serial_number(self):
        return [0] * 6
    
    def get_finger_order(self):
        return ["thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch", "middle_mcp_pitch", "ring_mcp_pitch", "pinky_mcp_pitch", "thumb_cmc_roll"]
    
    def show_fun_table(self):
        pass
    # --------------------------------------------------
    # 上下文管理
    # --------------------------------------------------

    def close(self):
        """断开 Modbus 连接。"""
        if self.connected:
            self.cli.close()
            self.connected = False
            print("Modbus 连接已断开。")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# ------------------- Demo/使用示例 -------------------
if __name__ == "__main__":
    # --- 配置区域 ---
    # 右手 Modbus ID: 0x27 (39)
    # 左手 Modbus ID: 0x28 (40)
    TARGET_HAND_ID = 0x28  # <--- 请根据需要修改为 0x27 或 0x28
    PORT = "/dev/ttyUSB0"   # <--- 请修改为您的实际串口，例如 'COM3'
    
    try:
        # 使用上下文管理器，确保连接自动关闭
        with LinkerHandL7RS485(hand_id=TARGET_HAND_ID, modbus_port=PORT) as hand:
            print("\n--- 1. 读取当前状态 ---")
            
            # 读取当前关节位置、速度、转矩
            angles = hand.get_joint_positions()
            print(f"当前关节位置 (7DOF): {angles}")
            
            speeds = hand.get_current_speeds()
            print(f"当前关节速度: {speeds}")
            
            # 读取传感器和错误信息
            temps = hand.get_temperatures()
            print(f"关节温度: {temps}")
            
            errors = hand.get_error_codes()
            print(f"关节错误码: {errors}")
            
            # 读取版本
            version_info = hand.get_version()
            print(f"版本信息 (Hand_freedom, ..., hardware_version): {version_info}")
            
            # --- 2. 写入指令示例 ---
            print("\n--- 2. 写入指令示例 (设置所有关节到 128) ---")
            
            # 假设要将所有关节位置设置到中间值 128
            target_angles = [128] * _JOINT_COUNT
            hand.set_joint_positions(target_angles)
            print(f"写入目标角度: {target_angles}")
            
            # 假设要设置所有关节的速度到 100
            target_speeds = [100] * _JOINT_COUNT
            hand.set_speeds(target_speeds)
            print(f"写入目标速度: {target_speeds}")

            # --- 3. 压力传感器读取示例 ---
            print("\n--- 3. 压力传感器读取 (大拇指 1) ---")
            thumb_matrix = hand.get_pressure_matrix(finger_id=1)
            print(f"大拇指压力矩阵 (12x6):")
            print(thumb_matrix)
            
    except ConnectionError as e:
        print(f"致命错误: 连接失败。{e}")
    except RuntimeError as e:
        print(f"致命错误: Modbus 操作失败。{e}")
    except Exception as e:
        print(f"捕获到未知异常: {e}")