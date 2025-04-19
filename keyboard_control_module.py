import keyboard
import time
import json
import os  
from Robot_Arm import Can_transfer
from Func import Json_Updata

class KeyboardControl():
    def load_config(self,json_path='config/argument.json'):
        # 检查文件是否存在
        if os.path.exists(json_path):
            with open(json_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        else:
            print(f"配置文件 {json_path} 不存在")
        with open(json_path, 'r', encoding='utf-8') as f:
            try:
                self.argument = json.load(f)
            except json.JSONDecodeError as e:
                raise ValueError(f"JSON 解码错误: {e}")
        self.idVendor = int(self.argument["idVendor"], 16)
        self.idProduct = int(self.argument["idProduct"], 16)

    def __init__(self,can_transfer):
        # with open('config/argument.json', 'r', encoding='utf-8') as f:
        #     self.argument = json.load(f)
        # self.idVendor = int(self.argument["idVendor"],16)
        # self.idProduct = int(self.argument["idProduct"],16)
        try:
            self.load_config()  # 初始化 self.argument
        except (FileNotFoundError, ValueError) as e:
            print(e)
            print("请确保配置文件存在且内容正确。")
            raise
        self.can_ = can_transfer
        self.can_.Update()
        if self.can_.open_deviceflag:
            print("USB设备打开成功")
        else:
            print("USB设备打开失败")
            return
        print(f"传入的 can_transfer 类型: {type(self.can_)}")  # 调试代码
        self.joint_1_key_data = 100
        self.joint_2_key_data = 100
        self.joint_3_key_data = 100
        self.joint_4_key_data = 100
        self.joint_5_key_data = 100
        self.joint_6_key_data = 100

    def update_joint_data(self):
        # 从机械臂读取当前电机状态并更新
        # print("调用 Update 方法...")
        self.can_.Update()
        self.joint_1_key_data = self.can_._1_link_angle
        self.joint_2_key_data = self.can_._2_link_angle
        self.joint_3_key_data = self.can_._3_link_angle
        self.joint_4_key_data = self.can_._4_link_angle
        self.joint_5_key_data = self.can_._5_link_angle
        self.joint_6_key_data = self.can_._6_link_angle
        print("电机状态:", self.joint_1_key_data, self.joint_2_key_data, self.joint_3_key_data, self.joint_4_key_data, self.joint_5_key_data, self.joint_6_key_data)

    def control_joint(self, joint_id, direction):
        # 根据方向调整对应关节的角度
        self.step=50000 #每次调整的步长
        if joint_id == 1:
            self.joint_1_key_data += self.step * direction
            self.can_.send_command([1, 0, self.joint_1_key_data])
            #print(f"控制1号电机，当前角度: {self.joint_1_key_data}")
        elif joint_id == 2:
            self.joint_2_key_data += self.step * direction
            self.can_.send_command([2, 0, self.joint_2_key_data])
            #print(f"控制2号电机，当前角度: {self.joint_2_key_data}")
        elif joint_id == 3:
            self.joint_3_key_data += self.step * direction
            self.can_.send_command([3, 0, self.joint_3_key_data])
        elif joint_id == 4:
            self.joint_4_key_data += self.step * direction
            self.can_.send_command([4, 0, self.joint_4_key_data])
        elif joint_id == 5:
            self.step=5000
            self.joint_5_key_data += self.step * direction
            self.can_.send_command([5, 0, self.joint_5_key_data])
        elif joint_id == 6:
            self.joint_6_key_data += self.step * direction
            self.can_.send_command([6, 0, self.joint_6_key_data])

    def run(self):
        print("键盘控制已启动，按下 'q' 退出")
        self.update_joint_data()  # 初始化关节数据
        while True:
            try:
                if keyboard.is_pressed('a'):  # 控制1号电机增加
                    self.control_joint(1, 1)
                    time.sleep(0.2)
                elif keyboard.is_pressed('d'):  # 控制1号电机减少
                    self.control_joint(1, -1)
                    time.sleep(0.2)
                elif keyboard.is_pressed('w'):  # 控制2号电机增加
                    self.control_joint(2, 1)
                    time.sleep(0.2)
                elif keyboard.is_pressed('s'):  # 控制2号电机减少
                    self.control_joint(2, -1)
                    time.sleep(0.2)
                elif keyboard.is_pressed('e'):  # 控制3号电机增加
                    self.control_joint(3, 1)
                    time.sleep(0.2)
                elif keyboard.is_pressed('q'):  # 控制3号电机减少
                    self.control_joint(3, -1)
                    time.sleep(0.2)
                elif keyboard.is_pressed('l'):  # 控制4号电机增加
                    self.control_joint(4, 1)
                    time.sleep(0.2)
                elif keyboard.is_pressed('j'):  # 控制4号电机减少
                    self.control_joint(4, -1)
                    time.sleep(0.2)
                elif keyboard.is_pressed('k'):  # 控制5号电机增加
                    self.control_joint(5, 1)
                    time.sleep(0.2)
                elif keyboard.is_pressed('i'):  # 控制5号电机减少
                    self.control_joint(5, -1)
                    time.sleep(0.2)
                elif keyboard.is_pressed('t'):  # 控制6号电机增加
                    self.control_joint(6, 1)
                    time.sleep(0.2)
                elif keyboard.is_pressed('r'):  # 控制6号电机减少
                    self.control_joint(6, -1)
                    time.sleep(0.2)
                elif keyboard.is_pressed('esc'):  # 退出
                    print("退出键盘控制")
                    break
            except Exception as e:
                print(f"发生错误: {e}")
                break