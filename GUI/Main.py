from datetime import datetime
import json
import threading

from PyQt5.QtWidgets import QMainWindow
from PyQt5 import QtCore
from PyQt5.QtGui import QImage, QPixmap
from langchain_core.output_parsers import JsonOutputParser
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.runnables import RunnablePassthrough
from GUI.Argument import ArgumentFunc
from config.prompt_template import System_Arm_Use_Tool

from Control_Arm import ArmControl
from GUI.Main_UI import Ui_MainWindow
from GUI.Record import RecordFunc
from GUI.Calibration import CalibrationFunc
from Location_Capture import CameraParameter
from Location_Capture import *
from Agent_Run import ArmAgent, Invoke_Tool
from config.tools import Execute_Action, Find_Object
from util.Flask_Connect import Multi_Model_Audio_to_Text
from util.Qwen_Connect import *


class MainFunc(QMainWindow, Ui_MainWindow):
    def __init__(self):
        Ui_MainWindow.__init__(self)
        QMainWindow.__init__(self)
        self.setupUi(self)

        # 全局变量
        self.AC = None #机械臂控制实例对象
        self.CP = None # 相机参数实例对象
        self.AA = None # 机械臂Agent实例对象
        self.box = None
        self.cap = None
        self.isPressed = False  # 用于跟踪按钮是否被按下
        self.threading_update = True # 线程里的Update是否继续
        self.isThreading = True # 是否开启线程
        self.isDepth = False # 是否开启深度相机
        self.camera_isOpened = False # 相机是否打开
        self.mouse_Event = False # 鼠标事件是否开启
        self.Record_isOpen = False # 录音是否开启
        
        with open('config/argument.json', 'r', encoding='utf-8') as f:
            self.argument = json.load(f)
        self.com = self.argument["com"]
        self.camera_index = self.argument["camera_index"]  #相机的索引

        
        self.pushButton_5.clicked.connect(self.Connect_Arm) # 连接机械臂
        self.pushButton_10.clicked.connect(self.Adjust) # 机械臂校准
        self.pushButton_12.clicked.connect(self.Reset) # 机械臂复位
        self.pushButton.clicked.connect(self.Send_Command) # 发送动作指令
        self.pushButton_6.clicked.connect(self.Send_Axis) # 发送轴指令
        self.pushButton_7.clicked.connect(self.Arm_Enable) # 机械臂使能
        self.pushButton_9.clicked.connect(self.Arm_Disable) # 机械臂失能
        self.pushButton_3.clicked.connect(self.Start_Vision) # 启动视觉模式
        self.pushButton_4.clicked.connect(self.Exit_Vision) # 退出视觉模式
        self.pushButton_11.clicked.connect(self.Open_Camera) # 打开相机
        self.pushButton_2.clicked.connect(self.Start_Depth_Estimation) # 启动深度估计
        self.pushButton_8.clicked.connect(self.Grab_Object) # 抓取物体
        self.pushButton_13.clicked.connect(self.Open_Agent) # 开启Agent
        self.pushButton_14.clicked.connect(self.Exit_Agent) # 关闭Agent
        self.pushButton_15.clicked.connect(self.Record_Dialog) # 点击开始对话
        
        #菜单栏事件
        self.actionrecord.triggered.connect(self.Action_Record)
        self.actioncalibration.triggered.connect(self.Calibration)
        self.actionArgument.triggered.connect(self.Argument)

        
        # 重写鼠标点击事件
        self.label.mousePressEvent = self.mousePressEvent 
        
        # 搜索所有动作添加到下拉框
        self.Search_Action()


########################################################################################################################################################
#                                                                     定义工具                                                                         #
########################################################################################################################################################
    # 重写closeEvent（窗口关闭信号）
    def closeEvent(self, event):
        # 关闭线程
        self.isThreading = False 
        if self.AC.claw_state is not None:
            self.AC.claw_state.value = -1
        if self.cap is not None:
            if self.cap.isOpened():
                self.cap.release()
                cv2.destroyAllWindows()
        
    # 鼠标点击事件
    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:  # 检查左键点击
            # 如果点击的组件是label_19
            if self.label.underMouse() and self.camera_isOpened and self.mouse_Event:
                x, y = event.x(), event.y()
                now_pose = self.AC.Get_Pose()
                if (x > self.CP.limit_rectangle[0] and x < self.CP.limit_rectangle[0] + self.CP.limit_rectangle[2] and y > self.CP.limit_rectangle[1] and y < self.CP.limit_rectangle[1] + self.CP.limit_rectangle[3]):
                    if self.CP.resize:
                        x, y = x * 2, y * 2
                    # 图像坐标转换为相机坐标
                    X = (x - self.CP.cx) * self.CP.Z / self.CP.fx
                    Y = (y - self.CP.cy) * self.CP.Z / self.CP.fy
                    # 齐次坐标
                    P_target2camera = np.array([X, Y, self.CP.Z, 1]).reshape(4, 1)
                    RT_end2base = self.CP.EIH.pose_to_homogeneous_matrix(now_pose[6:])
                    x, y, z = compute_pose(P_target2camera, self.CP.RT_camera2end, RT_end2base)
                    new_pose = [x + self.CP.baseX , y + self.CP.baseY, z + self.CP.gripper_lenght, 0, 0, -3.14]
                    
                    self.AC.Read_Track("list", new_pose, True)
                    # 驱动机械臂
                    thread = threading.Thread(target=self.Action_Arm, args=(self.AC.Run_Arm, None, None, True, "机械臂控制模式", "vision"))
                    thread.start()
                    
                    self.Update_TextEdit("点击抓取发送成功")

    # 实时更新6轴的角度
    def Updata_Angle_data(self):
        while self.isThreading:
            if self.threading_update:
                self.AC.can_.Update()
            self.label_13.setText(str(self.AC.can_._1_link_angle))
            self.label_14.setText(str(self.AC.can_._2_link_angle))
            self.label_15.setText(str(self.AC.can_._3_link_angle))
            self.label_16.setText(str(self.AC.can_._4_link_angle))
            self.label_17.setText(str(self.AC.can_._5_link_angle))
            self.label_18.setText(str(self.AC.can_._6_link_angle))
            self.label_20.setText(str(format(self.AC.can_.c_angle.px_out, '.5f')))
            self.label_22.setText(str(format(self.AC.can_.c_angle.py_out, '.5f')))
            self.label_24.setText(str(format(self.AC.can_.c_angle.pz_out, '.5f')))
            self.label_26.setText(str(format(self.AC.can_.c_angle.alpha_out, '.5f')))
            self.label_28.setText(str(format(self.AC.can_.c_angle.beta_out, '.5f')))
            self.label_32.setText(str(format(self.AC.can_.c_angle.gama_out, '.5f')))
            time.sleep(0.3)
    
    # 运行机械臂
    def Action_Arm(self, function, axis = None, angle = None, start_claw = False, show_text = "None", enable_pattern = "control"):
        self.threading_update = False
        time.sleep(0.3)
        self.Change_Enable(False, enable_pattern)
        self.label_5.setText("运行中")
        
        #接收参数
        if axis is not None and angle is not None:
            function(axis, angle)
        elif start_claw:
            function(start_claw)
        else:
            function()
        
        self.threading_update = True
        time.sleep(0.3)
        self.Change_Enable(True, enable_pattern)
        self.label_5.setText(show_text)
        
        
    
    # 修改enable状态
    def Change_Enable(self, state, pattern = "control"):
        if pattern == 'control':
            self.pushButton_5.setEnabled(state)
            self.pushButton_10.setEnabled(state)
            self.pushButton_12.setEnabled(state)
            self.pushButton.setEnabled(state)
            self.pushButton_6.setEnabled(state)
            self.pushButton_7.setEnabled(state)
            self.pushButton_9.setEnabled(state)
            self.pushButton_3.setEnabled(state) # 视觉开启的按钮
            self.pushButton_13.setEnabled(state) # Agent开启的按钮
        if pattern == "vision":
            self.pushButton_4.setEnabled(state)
            self.pushButton_11.setEnabled(state)
            self.pushButton_2.setEnabled(state)
            self.pushButton_8.setEnabled(state)
        if pattern == "agent":
            self.pushButton_14.setEnabled(state)
            self.pushButton_15.setEnabled(state)
            
        
    # 更新命令行
    def Update_TextEdit(self, text):
        # 获取当前的时间
        now = datetime.now()
        formatted_date_time = now.strftime("%Y-%m-%d %H:%M:%S")
        self.textEdit_2.append( "--------" + formatted_date_time + "--------")
        self.textEdit_2.append(text)
                
    # 搜索动作
    def Search_Action(self):
        with open("config/motion_config.json", 'r', encoding='utf-8') as f:
            config =json.load(f)
        keys = list(config.keys())
        self.comboBox.addItems(keys)
                
########################################################################################################################################################
#                                                                     一般信号槽                                                                        #
########################################################################################################################################################
    
    # 连接机械臂
    def Connect_Arm(self):
        try:
            self.AC = ArmControl()
            self.CP = CameraParameter()
            if self.AC.can_.open_deviceflag:
                self.CP.resize = True
                self.AC.print_targets = False
                self.AC.Set_Arm(self.com, claw_thread = True if self.argument["claw_thread"] == "True" else False)
                #开始实时更新角度
                thread = threading.Thread(target=self.Updata_Angle_data)
                thread.start()
                # 开启enable
                self.Change_Enable(True)
                # 更新状态
                self.label_5.setText("机械臂控制模式")
                self.Update_TextEdit("连接机械臂成功")
            else:
                self.Update_TextEdit("连接机械臂失败：Failed to open USB device.")
        except Exception as e:
            print(e)
            self.Update_TextEdit("连接机械臂失败："+str(e))
    
    # 机械臂校准
    def Adjust(self):
        try:
            if not self.isPressed:
                # 机械臂的电机使能标志位
                self.AC.can_.motor_enable_state = False
                # 机械臂的电机失能标志位
                self.AC.can_.motor_disable_state = True
                
                self.label_5.setText("校准中")
                self.Update_TextEdit("机械臂处于校准状态，请手动调整机械臂到安全位置。")
            else:
                # 机械臂的电机使能标志位
                self.AC.can_.motor_enable_state = True
                # 机械臂的电机失能标志位
                self.AC.can_.motor_disable_state = False
                self.AC.Calibration_Pose()
                
                self.label_5.setText("机械臂控制模式")
                self.Update_TextEdit("机械臂校准完成，校准文件保存在：config/pose_config.json。")
                
            self.isPressed = not self.isPressed  # 切换状态
            
        except Exception as e:
            self.Update_TextEdit("机械臂校准错误："+str(e))
            
    # 机械臂复位
    def Reset(self):
        try:
            self.AC.Update_Pose_Data()
            self.AC.Read_Track("list", self.AC.pose_config["zero_pose"], False)
            # 驱动机械臂
            thread = threading.Thread(target=self.Action_Arm, args=(self.AC.Run_Arm, None, None, False, "机械臂控制模式", "control"))
            thread.start()
            
            self.Update_TextEdit("机械臂复位成功")
        except Exception as e:
            self.Update_TextEdit("机械臂复位错误："+str(e))
            
    # 发送动作指令
    def Send_Command(self):
        try:
            command = self.comboBox.currentText()
            reset = True
            if command == "立正" or command == "右" or command == "左" or command == "抓取模式" or command == "换电":
                reset = False
            self.AC.Read_Track("json", 'config/motion_config.json', reset, command=command)
            
            # 驱动机械臂
            thread = threading.Thread(target=self.Action_Arm, args=(self.AC.Run_Arm, None, None, False, "机械臂控制模式", "control"))
            thread.start()
            
            self.Update_TextEdit(command + "发送成功")
        except Exception as e:
            self.Update_TextEdit("发送动作指令错误："+str(e))
            
    # 发送指定轴的运动
    def Send_Axis(self):
        try:
            axis = int(self.comboBox_2.currentText()[0])
            angle = int(self.textEdit_3.toPlainText())
            
            # 驱动机械臂
            thread = threading.Thread(target=self.Action_Arm, args=(self.AC.Single_Axis, axis, angle, False, "机械臂控制模式", "control"))
            thread.start()
            
            self.Update_TextEdit(str(axis) + "轴发送成功")
        except Exception as e:
            self.Update_TextEdit("发送指定轴错误："+str(e))
    
    # 使能
    def Arm_Enable(self):
        # 机械臂的电机使能标志位
        self.AC.can_.motor_enable_state = True
        # 机械臂的电机失能标志位
        self.AC.can_.motor_disable_state = False
        self.label_5.setText("使能状态")
        
    # 失能
    def Arm_Disable(self):
        # 机械臂的电机使能标志位
        self.AC.can_.motor_enable_state = False
        # 机械臂的电机失能标志位
        self.AC.can_.motor_disable_state = True
        self.label_5.setText("失能状态")
        
    # 开始视觉抓取模式
    def Start_Vision(self):
        try:
            if not self.camera_isOpened:
                self.Change_Enable(True, "vision")
                self.Change_Enable(False, "control")
                self.Change_Enable(False, "agent")
                self.mouse_Event = True
                # 失能机械臂进行手动调整
                self.Arm_Disable()
                
                self.box = None
                
                self.Update_TextEdit("视觉抓取模式已启动, 请手动调整机械臂至拍照位置\n完成后打开摄像头")
            else:
                self.Update_TextEdit("摄像头以被占用")
        except Exception as e:
            self.Update_TextEdit("视觉抓取模式启动失败："+str(e))
            
    # 退出视觉抓取模式
    def Exit_Vision(self):
        try:
            self.Change_Enable(False, "vision")
            self.Change_Enable(True, "control")
            self.Change_Enable(False, "agent")
            self.mouse_Event = False
            # 使能机械臂
            self.Arm_Enable()
            
            self.label.clear()
            self.label_3.clear()
            if self.cap.isOpened():
                self.cap.release()
                cv2.destroyAllWindows()
                
            self.camera_isOpened = False
            
            self.label_5.setText("机械臂控制模式")
            self.Update_TextEdit("视觉抓取模式已退出")
        except Exception as e:
            self.Update_TextEdit("视觉抓取模式退出失败："+str(e))
            
    # 开启摄像头
    def Open_Camera(self):
        try:
            self.label.clear()
            self.label_3.clear()
            if self.CP.resize:
                self.CP.limit_rectangle = [25, 25, 255, 110]
                
            # 更新机械臂的拍照位置
            Json_Updata("config/pose_config.json", "photo_pose", [self.AC.can_.c_angle.px_out, self.AC.can_.c_angle.py_out, self.AC.can_.c_angle.pz_out, self.AC.can_.c_angle.alpha_out, self.AC.can_.c_angle.beta_out, self.AC.can_.c_angle.gama_out]) 
            self.AC.Update_Pose_Data()
            
            self.cap = cv2.VideoCapture(self.camera_index)  # 打开摄像头
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            # 检查摄像头是否成功打开
            if not self.cap.isOpened():
                self.Update_TextEdit("无法打开摄像头")
                return self.cap
            
            # 使能机械臂
            self.Arm_Enable()
            self.label_5.setText("视觉抓取模式\n" + "当前使用的为默认深度："+str(self.CP.Z) + "mm")
            self.Update_TextEdit("打开摄像头成功")
            timer = QtCore.QTimer(self.centralwidget)
            timer.timeout.connect(self.Show_Video)
            timer.start(33)  # 30帧每秒
            self.camera_isOpened = True 
        except Exception as e:
            self.Update_TextEdit("摄像头开启失败："+str(e))
            
    # 显示视频
    def Show_Video(self):
        ret, frame = self.cap.read()
        # 读取摄像头图像
        if ret:
            # 切割为左右两张图片
            frame_L = frame[:, 0:640]
            frame_R = frame[:, 640:1280]
            # 纠正畸变
            u, v = frame_R.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.CP.mtx, self.CP.dist, (u, v), 0, (u, v))
            dst1 = cv2.undistort(frame_R, self.CP.mtx, self.CP.dist, None, newcameramtx)
            # dst1缩小一半
            if self.CP.resize:
                dst1 = cv2.resize(dst1, (0, 0), fx=0.5, fy=0.5)
            #在dst1上面画一个限位矩形框
            cv2.rectangle(dst1, (self.CP.limit_rectangle[0], self.CP.limit_rectangle[1]), (self.CP.limit_rectangle[0] + self.CP.limit_rectangle[2], self.CP.limit_rectangle[1] + self.CP.limit_rectangle[3]), (0, 255, 0), 2)
            
            # 检测矩形
            self.box, erosion = Detect_Rectangle(dst1, self.CP.limit_rectangle, self.CP.detect_threshold)
            if self.box is not None:
                rx, ry, rw, rh = self.box
                cv2.rectangle(dst1, (rx,ry), (rx + rw, ry + rh), (0, 0, 255), 2)
            
            # 颜色反转(改变通道数量)
            img_l_rgb = cv2.cvtColor(dst1, cv2.COLOR_BGR2RGB)
            img_r_rgb = cv2.cvtColor(erosion, cv2.COLOR_GRAY2RGB)
            
            # 将图像转换为Qt图像
            height_l, width_l, _ = img_l_rgb.shape
            bytes_per_line_l = 3 * width_l
            l_img = QImage(img_l_rgb.data, width_l, height_l, bytes_per_line_l, QImage.Format_RGB888)
            # 在label中显示原图像
            pixmap_l = QPixmap.fromImage(l_img)
            self.label.setPixmap(pixmap_l)
            self.label.setScaledContents(True)
                
            # 显示深度图
            if self.isDepth and self.box is not None: 
                disparity = Count_Disparity(frame_L, frame_R, self.CP.left_map1, self.CP.right_map2, self.CP.left_map1, self.CP.right_map2)
                self.CP.Z, dis_color = Count_Range_Depth(disparity, self.box, self.CP.Q, False, self.CP.resize)
                if self.CP.resize:
                    dis_color = cv2.resize(dis_color, (0, 0), fx=0.5, fy=0.5)
                    
                self.label_5.setText("视觉抓取模式\n" + "当前为实时计算的深度："+str(self.CP.Z) + "mm")
                
                height_d, width_d, _ = dis_color.shape
                bytes_per_line_d = 3 * width_d
                d_img = QImage(dis_color.data, width_d, height_d, bytes_per_line_d, QImage.Format_RGB888)
                # 在label_3中显示原图像
                pixmap_d = QPixmap.fromImage(d_img)
                self.label_3.setPixmap(pixmap_d)
                
            # 显示腐蚀后的图像
            else:
                self.label_5.setText("视觉抓取模式\n" + "当前使用的为默认深度："+str(self.CP.Z) + "mm")
                height_r, width_r, _ = img_r_rgb.shape
                bytes_per_line_r = 3 * width_r
                r_img = QImage(img_r_rgb.data, width_r, height_r, bytes_per_line_r, QImage.Format_RGB888)
                # 在label_3中显示原图像
                pixmap_r = QPixmap.fromImage(r_img)
                self.label_3.setPixmap(pixmap_r)
                self.label_3.setScaledContents(True)
                
            
    # 开启深度估计
    def Start_Depth_Estimation(self):
        try:
            if not self.isDepth:
                self.CP.limit_rectangle = [25, 25, 215, 110]
                self.isDepth = True
                
                self.label_5.setText("视觉抓取模式\n" + "当前为实时计算的深度："+str(self.CP.Z) + "mm")
                self.Update_TextEdit("正在深度计算中")
            else:
                self.CP.limit_rectangle = [25, 25, 255, 110]
                self.isDepth = False
                with open('config/argument.json', 'r', encoding='utf-8') as f:
                    self.argument = json.load(f)
                self.CP.Z = self.argument["default_Z"]
                
                self.label_5.setText("视觉抓取模式\n" + "当前使用的为默认深度："+str(self.CP.Z) + "mm")
                self.Update_TextEdit("关闭深度计算完成")
            
        except Exception as e:
            self.Update_TextEdit("深度计算出错："+str(e))
            
    # 识别物体抓取
    def Grab_Object(self):
        try:
            if self.box is not None:
                now_pose = self.AC.Get_Pose()
                new_pose = Rectangle_Pose(self.box, self.CP, now_pose[6:])
                self.AC.Read_Track("list", new_pose, True)
                
                # 驱动机械臂
                thread = threading.Thread(target=self.Action_Arm, args=(self.AC.Run_Arm, None, None, True, "视觉抓取模式\n" + "当前使用的为默认深度："+str(self.CP.Z) + "mm", "vision"))
                thread.start()
                self.Update_TextEdit("指令发送成功")
            else:
                self.Update_TextEdit("未检测到物体,无法抓取")
        except Exception as e:
            self.Update_TextEdit("抓取出错："+str(e))
            
            
########################################################################################################################################################
#                                                                     Agnet功能区                                                                       #
########################################################################################################################################################   
    # 开启摄像头
    def Agent_Open_Cameras(self):
        try:
            if not self.camera_isOpened:
                self.label.clear()
                self.label_3.clear()
                if self.CP.resize:
                    self.CP.limit_rectangle = [25, 25, 255, 110]
                    
                # 更新机械臂的拍照位置
                Json_Updata("config/pose_config.json", "photo_pose", [self.AC.can_.c_angle.px_out, self.AC.can_.c_angle.py_out, self.AC.can_.c_angle.pz_out, self.AC.can_.c_angle.alpha_out, self.AC.can_.c_angle.beta_out, self.AC.can_.c_angle.gama_out]) 
                self.AC.Update_Pose_Data()
                
                self.cap = cv2.VideoCapture(self.camera_index)  # 打开摄像头
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                # 检查摄像头是否成功打开
                if not self.cap.isOpened():
                    self.Update_TextEdit("无法打开摄像头")
                    return self.cap
                
                self.label_5.setText("Agent模式")
                self.Update_TextEdit("打开摄像头成功，处于Agent模式")
                timer = QtCore.QTimer(self.centralwidget)
                timer.timeout.connect(self.Agent_Show_Video)
                timer.start(33)  # 30帧每秒
                self.camera_isOpened = True 
            else:
                self.Update_TextEdit("摄像头已被占用")
        except Exception as e:
            self.Update_TextEdit("摄像头开启失败："+str(e))
    
    # Agent显示摄像头
    def Agent_Show_Video(self):
        ret, frame = self.cap.read()
        # 读取摄像头图像
        if ret:
            # 切割为左右两张图片
            frame_L = frame[:, 0:640]
            frame_R = frame[:, 640:1280]
            # 纠正畸变
            u, v = frame_R.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.CP.mtx, self.CP.dist, (u, v), 0, (u, v))
            dst1 = cv2.undistort(frame_R, self.CP.mtx, self.CP.dist, None, newcameramtx)
            # dst1缩小一半
            if self.CP.resize:
                dst1 = cv2.resize(dst1, (0, 0), fx=0.5, fy=0.5)
            #在dst1上面画一个限位矩形框
            cv2.rectangle(dst1, (self.CP.limit_rectangle[0], self.CP.limit_rectangle[1]), (self.CP.limit_rectangle[0] + self.CP.limit_rectangle[2], self.CP.limit_rectangle[1] + self.CP.limit_rectangle[3]), (0, 255, 0), 2)
            
            # 开启yolo检测
            if self.AA.open_yolo:
                cv2.imwrite('Temp/current_frame.jpg', frame_R)
                self.AA.YO.Set_Label(self.AA.label)
                self.AA.YO.Set_Source('Temp/current_frame.jpg')
                box_list_xyxy = self.AA.YO.Perform_Inference(**vars(self.AA.YO.opt))
                if box_list_xyxy != []:
                    self.AA.box_list.clear()
                    for xyxy in box_list_xyxy:
                        x1,y1,x2,y2 = int(xyxy[0] / 2), int(xyxy[1] / 2), int(xyxy[2] / 2), int(xyxy[3] / 2)
                        x, y, w, h = self.CP.limit_rectangle
                        rx, ry, rw, rh = x1, y1, x2 - x1, y2 - y1
                        if x1 > x and x1 < x + w and y1 > y and y1 < y + h and rx + rw <= x + w and ry + rh <= y + h:
                            cv2.rectangle(dst1, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            self.AA.box_list.append([rx, ry, rw, rh])
                        
            # 颜色反转(改变通道数量)
            img_l_rgb = cv2.cvtColor(dst1, cv2.COLOR_BGR2RGB)
            
            # 将图像转换为Qt图像
            height_l, width_l, _ = img_l_rgb.shape
            bytes_per_line_l = 3 * width_l
            l_img = QImage(img_l_rgb.data, width_l, height_l, bytes_per_line_l, QImage.Format_RGB888)
            # 在label中显示原图像
            pixmap_l = QPixmap.fromImage(l_img)
            self.label.setPixmap(pixmap_l)
            self.label.setScaledContents(True)
            

    # 打开Agent
    def Open_Agent(self):
        try:
            self.Change_Enable(False, "vision")
            self.Change_Enable(False, "control")
            self.Change_Enable(True, "agent")
            if self.AA == None:
                tools = [Execute_Action, Find_Object]
                self.label_5.setText("开启Agnet中...")
                model_invoke_pattern = self.argument["model_invoke_pattern"]
                self.AA = ArmAgent(self.AC, self.CP, tools, model_invoke_pattern)
                self.AA.mindb = False
            self.Agent_Open_Cameras()
            
        except Exception as e:
            self.Update_TextEdit("打开Agent出错: "+str(e))
            
    # 关闭Agent
    def Exit_Agent(self):
        try:
            self.Change_Enable(False, "vision")
            self.Change_Enable(True, "control")
            self.Change_Enable(False, "agent")
            
            self.label.clear()
            if self.cap.isOpened():
                self.cap.release()
                cv2.destroyAllWindows()
                
            self.camera_isOpened = False
            
            self.label_5.setText("机械臂控制模式")
            self.Update_TextEdit("Agent模式已退出")
        except Exception as e:
            self.Update_TextEdit("Agent模式退出失败："+str(e))
            
    # 录音对话
    def Record_Dialog(self):
        try:
            if not self.Record_isOpen:
                self.Record_isOpen = True
                self.Update_TextEdit("录制对话开始，请开始说话......\n再次点击结束录制。")
                thread = threading.Thread(target=self.AA.AF.Transcribe_Audio, args=("Temp/transcribe_audio.wav", self.Record_isOpen))
                thread.start()
                
            else:
                self.Record_isOpen = False
                self.AA.AF.stop = True
                time.sleep(1)
                if self.AA.CM.model_invoke_pattern == "Local":
                    question = Multi_Model_Audio_to_Text("Temp/transcribe_audio.wav").strip()
                elif self.AA.CM.model_invoke_pattern == "Qwen":
                    question = Qwen_Audio_to_Text("Temp/transcribe_audio.wav").strip()
                self.Update_TextEdit("录制对话已关闭。\n你的问题: " + question)
                prompt = ChatPromptTemplate.from_messages([("system", System_Arm_Use_Tool(self.AA.rendered_tools)), ("human", "{input}")])
                use_tool = (prompt | self.AA.CM | JsonOutputParser()).invoke({"input": question})
                self.Update_TextEdit("机械臂回答: " + str(use_tool))
                # 调用工具
                result = [RunnablePassthrough.assign(output=lambda req: Invoke_Tool(req, tools=self.AA.tools, objects=self, box_list=self.AA.box_list)).invoke(i) for i in use_tool]
                if result == None:
                    self.Update_TextEdit("机械臂回答: 不认识的物体或者没有学过的动作")
                
                
                
                
        except Exception as e:
            self.Update_TextEdit("录音对话出错: "+str(e))
            
            
            
########################################################################################################################################################
#                                                                         菜单栏                                                                       #
########################################################################################################################################################            
            
            
    #弹出动作录制窗口
    def Action_Record(self):
        if self.AC is not None:
            self.actionWindow = RecordFunc(self.AC)
            self.actionWindow.show()
        else:
            self.Update_TextEdit("请先连接机械臂")
            
    #弹出校准窗口
    def Calibration(self):
        if self.AC is not None:
            self.calibrationWindow = CalibrationFunc(self.AC)
            self.calibrationWindow.show()
        else:
            self.Update_TextEdit("请先连接机械臂")

    def Argument(self):
        self.ArgumentWindow = ArgumentFunc()
        self.ArgumentWindow.show()