import can
import time, sys
import threading
import numpy as np
from utils.open_can import OpenCan
from utils.color_msg import ColorMsg


class LinkerHandL7Can:
    def __init__(self, can_id, can_channel='can0', baudrate=1000000,yaml=""):
        self.can_id = can_id
        self.can_channel = can_channel
        self.baudrate = baudrate
        self.open_can = OpenCan(load_yaml=yaml)

        self.x01 = [0] * 7
        self.x02 = [-1] * 7
        self.x05 = [0] * 7
        self.x33 = [0] * 7
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
        # Fault codes
        self.x35 = [0] * 7, [0] * 7
        self.joint_angles = [0] * 10
        self.pressures = [200] * 7  # Default torque 200
        self.bus = self.init_can_bus(can_channel, baudrate)
        self.normal_force, self.tangential_force, self.tangential_force_dir, self.approach_inc = [[-1] * 7 for _ in range(4)]
        self.is_lock = False
        self.version = None
        # Start the receiving thread
        self.running = True
        self.receive_thread = threading.Thread(target=self.receive_response)
        self.receive_thread.daemon = True
        self.receive_thread.start()

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

    def send_frame(self, frame_property, data_list,sleep=0.005):
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
        time.sleep(sleep)

    def set_joint_positions(self, joint_angles):
        """Set the positions of 10 joints (joint_angles: list of 10 values)."""
        self.is_lock = True
        if len(joint_angles) > 7:
            self.joint_angles = joint_angles[:7]
        else:
            self.joint_angles = joint_angles
        # Send angle control in frames
        self.send_frame(0x01, self.joint_angles, sleep=0.003)
        self.is_lock = False

    def set_max_torque_limits(self, pressures, type="get"):
        """Set maximum torque limits."""
        if type == "get":
            self.pressures = [0.0]
        else:
            self.pressures = pressures[:7]

    def set_torque(self, torque=[180] * 7):
        """Set L7 maximum torque limits."""
        if len(torque) != 7:
            raise ValueError("Torque list must have 7 elements.")
            return
        self.send_frame(0x02, torque)

    def set_speed(self, speed=[180] * 7):
        """Set L7 speed."""
        if len(speed) != 7:
            raise ValueError("Speed list must have 7 elements.")
            return
        self.x05 = speed
        for i in range(2):
            time.sleep(0.001)
            self.send_frame(0x05, speed)

    ''' -------------------Pressure Sensors---------------------- '''
    def get_normal_force(self):
        self.send_frame(0x20, [],sleep=0.01)

    def get_tangential_force(self):
        self.send_frame(0x21, [],sleep=0.01)

    def get_tangential_force_dir(self):
        self.send_frame(0x22, [],sleep=0.01)

    def get_approach_inc(self):
        self.send_frame(0x23, [],sleep=0.01)

    ''' -------------------Motor Temperature---------------------- '''
    def get_motor_temperature(self):
        self.send_frame(0x33, [])

    # Motor fault codes
    def get_motor_fault_code(self):
        self.send_frame(0x35, [])

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
            frame_type = msg.data[0]
            response_data = msg.data[1:]
            if frame_type == 0x01:   # 0x01
                self.x01 = list(response_data)
            elif frame_type == 0x02:    # 0x02
                self.x02 = list(response_data)
            elif frame_type == 0x05: # Set speed
                self.x05 = list(response_data)
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
            elif frame_type == 0x33: # L7 temperature
                self.x33 = list(response_data)
            elif frame_type == 0x35: # L7 fault codes
                self.x35 = list(response_data)
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
            elif frame_type == 0x64: # L7 version number
                self.version = list(response_data)
            elif frame_type == 0xC2: # O6 version number
                self.version = list(response_data)

    def get_version(self):
        self.send_frame(0x64, [], sleep=0.1)
        time.sleep(0.1)
        if self.version is None:
            self.send_frame(0xC2, [], sleep=0.1)
            time.sleep(0.1)
        return self.version

    def get_current_status(self):
        if self.is_lock:
            return self.x01
        elif self.is_lock == False:
            self.send_frame(0x01, [],sleep=0.003)
            return self.x01
        
    def get_current_pub_status(self):
        return self.x01

    def get_speed(self):
        self.send_frame(0x05, [],sleep=0.003)
        return self.x05

    def get_current(self):
        '''Not supported yet.'''
        self.send_frame(0x2, [],sleep=0.1)
        return self.x02



    def get_torque(self):
        '''Not supported yet.'''
        self.send_frame(0x2, [],sleep=0.01)
        return self.x02

    def get_touch_type(self):
        '''Get touch type'''
        self.send_frame(0xb1,[])
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
        self.send_frame(0xb1,[0xc6],sleep=0.01)
        self.send_frame(0xb2,[0xc6],sleep=0.01)
        self.send_frame(0xb3,[0xc6],sleep=0.01)
        self.send_frame(0xb4,[0xc6],sleep=0.01)
        self.send_frame(0xb5,[0xc6],sleep=0.01)

        return self.thumb_matrix , self.index_matrix , self.middle_matrix , self.ring_matrix , self.little_matrix
    
    def get_matrix_touch_v2(self):
        self.send_frame(0xb1,[0xc6],sleep=0.005)
        self.send_frame(0xb2,[0xc6],sleep=0.005)
        self.send_frame(0xb3,[0xc6],sleep=0.005)
        self.send_frame(0xb4,[0xc6],sleep=0.005)
        self.send_frame(0xb5,[0xc6],sleep=0.005)
        return self.thumb_matrix , self.index_matrix , self.middle_matrix , self.ring_matrix , self.little_matrix


    def get_thumb_matrix_touch(self):
        self.send_frame(0xb1,[0xc6],sleep=0.005)
        return self.thumb_matrix
    
    def get_index_matrix_touch(self):
        self.send_frame(0xb2,[0xc6],sleep=0.005)
        return self.index_matrix
    
    def get_middle_matrix_touch(self):
        self.send_frame(0xb3,[0xc6],sleep=0.005)
        return self.middle_matrix
    
    def get_ring_matrix_touch(self):
        self.send_frame(0xb4,[0xc6],sleep=0.005)
        return self.ring_matrix
    
    def get_little_matrix_touch(self):
        self.send_frame(0xb5,[0xc6],sleep=0.005)
        return self.little_matrix

    def get_force(self):
        '''Get pressure.'''
        return [self.normal_force, self.tangential_force, self.tangential_force_dir, self.approach_inc]

    def get_temperature(self):
        '''Get temperature.'''
        self.get_motor_temperature()
        return self.x33

    def get_fault(self):
        '''Get faults.'''
        self.get_motor_fault_code()
        return self.x35
    
    def show_fun_table(self):
        pass

    def close_can_interface(self):
        """Stop the CAN communication."""
        self.running = False
        if self.receive_thread.is_alive():
            self.receive_thread.join()
        if self.bus:
            self.bus.shutdown()
