from PyQt5.QtWidgets import QMainWindow, QSplitter, QApplication,QMessageBox,QPushButton
from PyQt5.QtCore import Qt, QTimer
import yaml, os, sys,time,json
from views.left_view import LeftView
from views.right_view import RightView
from views.wave_form_plot import WaveformPlot
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, "../.."))
sys.path.append(target_dir)
from LinkerHand.linker_hand_api import LinkerHandApi
from LinkerHand.utils.load_write_yaml import LoadWriteYaml
from LinkerHand.utils.color_msg import ColorMsg
'''
LinkerHand图形控制
'''

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self._init_hand_joint()
        self.api = LinkerHandApi(hand_joint=self.hand_joint,hand_type=self.hand_type)
        self.touch_type = -1
        self._init_gui_view()
        if self.hand_joint == "L7":
            self.add_button_position = [255] * 7
            self.set_speed = [180,250,250,250,250,250,250]
            self.touch_type = self.api.get_touch_type()
            # if self.touch_type == 2:
            #     self._init_normal_force_plot(num_lines=6) # 法向压力波形图
            # else:
            #     self._init_normal_force_plot() # 法向压力波形图
            #     self._init_approach_inc_plot() # 接近感应波形图
        elif self.hand_joint == "L10":
            self.add_button_position = [255] * 10 # 记录添加按钮的位置
            self.set_speed(speed=[180,250,250,250,250])
            self.touch_type = self.api.get_touch_type()
            # if self.touch_type == 2:
            #     self._init_normal_force_plot(num_lines=6) # 法向压力波形图
            # else:
            #     self._init_normal_force_plot() # 法向压力波形图
            #     self._init_approach_inc_plot() # 接近感应波形图
        elif self.hand_joint == "L20":
            self.add_button_position = [255] * 20 # 记录添加按钮的位置
            self.set_speed(speed=[120,180,180,180,180])
            self._init_normal_force_plot() # 法向压力波形图
            self.touch_type = self.api.get_touch_type()
            if self.touch_type == 2:
                self._init_normal_force_plot(num_lines=6) # 法向压力波形图
            else:
                self._init_normal_force_plot() # 法向压力波形图
                self._init_approach_inc_plot() # 接近感应波形图
        elif self.hand_joint == "L21":
            self.add_button_position = [255] * 25
            self.set_speed(speed=[60,220,220,220,220])
            self._init_normal_force_plot() # 法向压力波形图
            self.touch_type = self.api.get_touch_type()
            if self.touch_type == 2:
                self._init_normal_force_plot(num_lines=6) # 法向压力波形图
            else:
                self._init_normal_force_plot() # 法向压力波形图
                self._init_approach_inc_plot() # 接近感应波形图
        elif self.hand_joint == "L25":
            self.add_button_position = [255] * 30 # 记录添加按钮的位置
            self.set_speed(speed=[60,250,250,250,250])
        
        

    def _init_hand_joint(self):
        self.yaml = LoadWriteYaml() # 初始化配置文件
        # 读取配置文件
        self.setting = self.yaml.load_setting_yaml()
        # 判断左手是否配置
        self.left_hand = False
        self.right_hand = False
        if self.setting['LINKER_HAND']['LEFT_HAND']['EXISTS'] == True:
            self.left_hand = True
        elif self.setting['LINKER_HAND']['RIGHT_HAND']['EXISTS'] == True:
            self.right_hand = True
        # gui控制只支持单手，这里进行左右手互斥
        if self.left_hand == True and self.right_hand == True:
            self.left_hand = True
            self.right_hand = False
        if self.left_hand == True:
            print("左手")
            self.hand_exists = True
            self.hand_joint = self.setting['LINKER_HAND']['LEFT_HAND']['JOINT']
            self.hand_type = "left"
        if self.right_hand == True:
            print("右手")
            self.hand_exists = True
            self.hand_joint = self.setting['LINKER_HAND']['RIGHT_HAND']['JOINT']
            self.hand_type = "right"
        
        self.init_pos = [255] * 10
        if self.hand_joint == "L25":
            # L25
            self.init_pos = [96, 255, 255, 255, 255, 150, 114, 151, 189, 255, 180, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
            self.joint_name = ["拇指根部", "食指根部", "中指根部", "无名指根部","小指根部","拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小指侧摆","拇指横摆","预留","预留","预留","预留","拇指中部","食指中部","中指中部","无名指中部","小指中部","拇指指尖","食指指尖","中指指尖","无名指指尖","小指指尖"]
        elif self.hand_joint == "L21":
            # L21
            self.init_pos = [96, 255, 255, 255, 255, 150, 114, 151, 189, 255, 180, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
            self.joint_name = ["拇指根部", "食指根部", "中指根部", "无名指根部","小指根部","拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小指侧摆","拇指横摆","预留","预留","预留","预留","拇指中部","预留","预留","预留","预留","拇指指尖","食指指尖","中指指尖","无名指指尖","小指指尖"]
        elif self.hand_joint == "L20":
            self.init_pos = [255,255,255,255,255,255,10,100,180,240,245,255,255,255,255,255,255,255,255,255]
            # L20
            self.joint_name = ["拇指根部", "食指根部", "中指根部", "无名指根部","小指根部","拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小指侧摆","拇指横摆","预留","预留","预留","预留","拇指尖部","食指末端","中指末端","无名指末端","小指末端"]
        elif self.hand_joint == "L10":
            # L10
            self.init_pos = [255] * 10
            self.joint_name = ["拇指根部", "拇指侧摆","食指根部", "中指根部", "无名指根部","小指根部","食指侧摆","无名指侧摆","小指侧摆","拇指旋转"]
        elif self.hand_joint == "L7":
            # L7
            self.init_pos = [250] * 7
            self.joint_name = ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲","小拇指弯曲","拇指旋转"]
        
    
    # 初始化窗口界面
    def _init_gui_view(self):
        if self.hand_type == "left":
            self.setWindowTitle(f"Linker_Hand:左手- {self.hand_joint} Control - Qt5 with ROS")
        else:
            self.setWindowTitle(f"Linker_Hand:右手- {self.hand_joint} Control - Qt5 with ROS")
        self.setGeometry(100, 100, 600, 800)
        # 创建分割线
        splitter = QSplitter(Qt.Horizontal)
        splitter.setStyleSheet("""
            QSplitter::handle {
                width:1px;
                background-color: lightgray;
                margin: 15px 20px;
            }
        """)
        # 左侧滑动条界面
        self.left_view = LeftView(joint_name=self.joint_name, init_pos=self.init_pos)
        splitter.addWidget(self.left_view)
        self.left_view.slider_value_changed.connect(self.handle_slider_value_changed)
        # 右侧记录动作界面
        self.right_view = RightView(hand_joint=self.hand_joint, hand_type=self.hand_type)
        splitter.addWidget(self.right_view)
        # 接收到信号槽事件，这里用于记录动作序列更新滑动条数据
        self.right_view.handle_button_click.connect(self.handle_button_click)
        self.right_view.add_button_handle.connect(self.add_button_handle)
        splitter.setSizes([600, 450])
        self.setCentralWidget(splitter)
    # 初始化波形图
    def _init_normal_force_plot(self,num_lines=5):
        return
        # 初始化波形图
        self.normal_force_plot = WaveformPlot(num_lines=num_lines, labels=None,title="法向压力波形图")
        # 设置波形图位置
        self.normal_force_plot.setGeometry(700, 100, 800, 400)
        self.normal_force_plot.show()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_normal_force_plot)
        self.timer.start(50)
    def _init_approach_inc_plot(self):
        return
        # 初始化波形图
        self.approach_inc_plot = WaveformPlot(num_lines=5, labels=None,title="接近感应波形图")
        # 设置波形图位置
        self.approach_inc_plot.setGeometry(700, 600, 800, 400)
        self.approach_inc_plot.show()
        self.timer2 = QTimer()
        self.timer2.timeout.connect(self.update_approach_inc_plot)
        self.timer2.start(50)
    # 点击按钮后将动作数值写入yaml文件
    def handle_button_click(self,text):
        all_action = self.yaml.load_action_yaml(hand_type=self.hand_type,hand_joint=self.hand_joint)
        for index,pos in enumerate(all_action):
            if pos['ACTION_NAME'] == text:
                position = pos['POSITION']
                print(type(position))
        #print(f"动作名称:{text}, 动作数值:{action_pos}")
        # if text == self.la:
        #     self.left_view.set_slider_values(values=self.add_button_position)
        ColorMsg(msg=f"动作名称:{text}, 动作数值:{position}", color="green")
        self.api.finger_move(pose=position)
        self.left_view.set_slider_values(values=position)

    #点击添加按钮后将动作数值写入yaml文件
    def add_button_handle(self,text):
        self.add_button_position = self.left_view.get_slider_values()
        self.add_button_text = text
        self.yaml.write_to_yaml(action_name=text, action_pos=self.left_view.get_slider_values(),hand_joint=self.hand_joint,hand_type=self.hand_type)


    # 通过信号机制实时获取滑动条的当前值
    def handle_slider_value_changed(self, slider_values):
        #print("实时获取滑动条的当前值:", slider_values)
        slider_values_list = []
        for key in slider_values:
            slider_values_list.append(slider_values[key])
        self.api.finger_move(pose=slider_values_list)
    # 更新滑动条状态
    def update_label(self, index, value):
        self.left_view.labels[index].setText(f"{self.joint_name[index]}: {value}")

    # 更新法向压力波形图
    def update_normal_force_plot(self):
        import random
        #touch_type = self.api.get_touch_type()
        if self.touch_type == 2:
            values = self.api.get_touch()
        else:
            f = self.api.get_force()
            values = f[0]
        if values == None:
            pass
        else:
            self.normal_force_plot.update_data(values)
    # 更新接近感应波形图
    def update_approach_inc_plot(self):
        import random
        #touch_type = self.api.get_touch_type()
        if self.touch_type == 2:
            values = [0] * 5
        else:
            f = self.api.get_force()
            values = f[3]
        self.approach_inc_plot.update_data(values)

    def set_speed(self,speed=[180,250,250,250,250]):
        ColorMsg(msg=f"设置速度:{speed}", color="green")
        self.api.set_speed(speed)
    # 关闭窗口结束程序
    def closeEvent(self, event):
        """关闭窗口时停止线程并释放资源"""
        # 关闭波形图窗口
        self.normal_force_plot.close()
        self.approach_inc_plot.close()
        self.close()
        event.accept()

    

    

# 主程序运行
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec_())