from PyQt5.QtWidgets import QMainWindow
from PyQt5 import QtCore
from PyQt5.QtGui import QImage, QPixmap

from GUI.Record_UI import Ui_Record
from util.Func import Json_Updata

class RecordFunc(QMainWindow, Ui_Record):
    def __init__(self, AC):
        QMainWindow.__init__(self)
        Ui_Record.__init__(self)
        self.setupUi(self)
        
        # 全局变量
        self.AC = AC
        self.targets = []
        self.action_name = None
        self.index = 1
        
        self.pushButton.clicked.connect(self.Start_Record) # 开启录制
        self.pushButton_2.clicked.connect(self.Record_Pose) # 记录当前位姿
        self.pushButton_3.clicked.connect(self.Save_Action) # 保存动作
        
    # 修改enable状态
    def Change_Enable(self, state):
        self.pushButton_2.setEnabled(state)
        self.pushButton_3.setEnabled(state)
        
    # 开启动作录制
    def Start_Record(self):
        # 机械臂的电机使能标志位
        self.AC.can_.motor_enable_state = False
        # 机械臂的电机失能标志位
        self.AC.can_.motor_disable_state = True
        
        self.Change_Enable(True)
        
        self.textEdit.append("开启成功")
        
        
    # 记录当前机械臂的位姿
    def Record_Pose(self):
        pose = self.AC.Get_Pose()

        self.textEdit.append( "--------" + "动作" + str(self.index) + "的位姿" + "--------")
        self.textEdit.append("X: " + str(pose[6]) + " \nY: " + str(pose[7]) + " \nZ: " + str(pose[8]) + " \nRZ: " + str(pose[9]) + " \nRY: " + str(pose[10]) + " \nRX: " + str(pose[11]))
        
        # 写入文件
        new_target = [
            pose[6],
            pose[7],
            pose[8],
            pose[9],
            pose[10],
            pose[11]
        ]
        self.targets.append(new_target)
        self.index += 1
        
    # 保存动作
    def Save_Action(self):
        self.action_name = self.textEdit_2.toPlainText()
        if self.action_name != "":
            Json_Updata("config/motion_config.json", self.action_name, self.targets)
            
            # 机械臂的电机使能标志位
            self.AC.can_.motor_enable_state = True
            # 机械臂的电机失能标志位
            self.AC.can_.motor_disable_state = False
            
            self.textEdit.append("保存完成")
            self.index = 1
        else:
            self.textEdit.append("动作名称不能为空")
        
        