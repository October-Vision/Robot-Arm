import keyboard
import json
import os  # 添加 os 模块用于路径检查

class KeyboardControl:
    def load_config(self, json_path='config/keyboard.json'):  # 修正路径
        # 检查文件是否存在
        if not os.path.exists(json_path):
            raise FileNotFoundError(f"配置文件未找到: {json_path}")
        # 检查文件是否为空
        if os.path.getsize(json_path) == 0:
            raise ValueError(f"配置文件为空: {json_path}")
        # 读取配置文件
        with open(json_path, 'r', encoding='utf-8') as f:  # 显式指定编码为 utf-8
            try:
                self.argument = json.load(f)
            except json.JSONDecodeError as e:
                raise ValueError(f"配置文件格式错误: {json_path}. 错误信息: {e}")
        # 获取 idVendor 和 idProduct
        self.idVendor = int(self.argument["idVendor"], 16)
        self.idProduct = int(self.argument["idProduct"], 16)
        self.keyboard_config = self.argument["keyboard"]

    def __init__(self, can_transfer_instance):
        try:
            self.load_config()  # 初始化 self.argument
        except (FileNotFoundError, ValueError) as e:
            print(e)
            print("请确保配置文件存在且内容正确。")
            raise
        self.can_ = can_transfer_instance
        self.can_.Update()
        self.bind_keys()
        if self.can_.open_deviceflag:
            print("设备已连接")
        else:
            print("设备未找到")

    def bind_keys(self):
        # 绑定键盘事件
        keyboard.add_hotkey('w', lambda: self.control_joint_1_and_2_and_3())
        keyboard.add_hotkey('s', lambda: self.can_.send_command(self.keyboard_config["backward"]["data"]))
        keyboard.add_hotkey('a', lambda: self.can_.send_command(self.keyboard_config["left"]["data"]))
        keyboard.add_hotkey('d', lambda: self.can_.send_command(self.keyboard_config["right"]["data"]))
        keyboard.add_hotkey('q', lambda: self.can_.send_command(self.keyboard_config["up"]["data"]))
        keyboard.add_hotkey('e', lambda: self.can_.send_command(self.keyboard_config["down"]["data"]))
        print("键盘事件已绑定: 'w' -> 前进, 's' -> 后退, 'a' -> 左转, 'd' -> 右转, 'q' -> 上升, 'e' -> 下降")

    def control_joint_1_and_2_and_3(self):
        # 控制关节 1 和关节 2
        joint_1_command = self.keyboard_config["forward"]["joint_1_data"]
        joint_2_command = self.keyboard_config["forward"]["joint_2_data"]
        joint_3_command = self.keyboard_config["forward"]["joint_3_data"]
        self.can_.send_command(joint_1_command)
        self.can_.send_command(joint_2_command)
        self.can_.send_command(joint_3_command)

    def run(self):
        # 运行键盘监听
        print("按下 'esc' 键退出程序")
        keyboard.wait('esc')  # 等待 'esc' 键被按下
        # 释放键盘事件
        keyboard.unhook_all()
        print("程序已退出")
