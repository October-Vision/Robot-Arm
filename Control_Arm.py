import json
import time
import os
import math
import keyboard

from threading import Thread
import multiprocessing
#import cv2
#import keyboard
from keyboard_control_module import KeyboardControl
from Robot_Arm import Can_transfer, claw_control
from Func import Json_Updata

class ArmControl():
    def __init__(self):
        with open('config/argument.json', 'r', encoding='utf-8') as f:
            self.argument = json.load(f)
        self.idVendor = int(self.argument["idVendor"],16)
        self.idProduct = int(self.argument["idProduct"],16)
        self.can_ = Can_transfer(idVendor=self.idVendor, idProduct=self.idProduct) # 连接USBCan通讯（不同数据线的id可能不一样的，这个需要改！！！）
        with open('config/pose_config.json', 'r', encoding='utf-8') as f:
            self.pose_config = json.load(f)
        self.res = self.pose_config["res"] # 机械臂的睡眠点位置
        self.zero = self.pose_config["zero"] # 机械臂的安全点位置
        self.can_.read_angle_flag = True # 读取机械臂的角度标志位
        self.can_.Start(res=self.res, zero=self.zero) # 初始化及连接机械臂
        self.targets = [] # 安全点轨迹列表
        self.sleep_targets = [] # 睡眠点轨迹列表
        self.claw_state = None # 机械爪状态
        self.print_targets = True # 是否打印轨迹列表

    # 设置机械臂
    def Set_Arm(self, claw_com, res = False, zero = False, enable = False, disable = False, claw_thread = True):
        # 机械臂回到睡眠点标志位
        self.can_.write_res_angle_flag = res
        if self.can_.write_res_angle_flag:
            self.can_._1_edit_angle, self.can_._2_edit_angle, self.can_._3_edit_angle, self.can_._4_edit_angle, self.can_._5_edit_angle, self.can_._6_edit_angle = self.res
        # read_angle_flag
        self.can_.write_zero_angle_flag = zero
        if self.can_.write_zero_angle_flag:
            self.can_._1_edit_angle, self.can_._2_edit_angle, self.can_._3_edit_angle, self.can_._4_edit_angle, self.can_._5_edit_angle, self.can_._6_edit_angle = self.zero
        # 机械臂的电机使能标志位
        self.can_.motor_enable_state = enable
        # 机械臂的电机失能标志位
        self.can_.motor_disable_state = disable
        self.can_.Update()
        time.sleep(0.1)

        #开启夹爪线程
        self.claw_state = multiprocessing.Value('d', 0.0)
        if claw_thread:
            self.clawThread = Thread(target=claw_control, args=(self.claw_state, claw_com)) # 开启机械爪线程
            self.clawThread.start()
    
    # 更新获取pose_config文件数据
    def Update_Pose_Data(self):
        with open('config/pose_config.json', 'r', encoding='utf-8') as f:
            self.pose_config = json.load(f)
        self.res = self.pose_config["res"] # 机械臂的睡眠点位置
        self.zero = self.pose_config["zero"] # 机械臂的安全点位置
        
        
    # 获取当前位姿
    def Get_Pose(self):
        self.can_.Update()  # 调用 Update 方法更新电机状态
        pose = [self.can_._1_link_angle,
                self.can_._2_link_angle,
                self.can_._3_link_angle,
                self.can_._4_link_angle,
                self.can_._5_link_angle,
                self.can_._6_link_angle,
                self.can_.c_angle.px_out,
                self.can_.c_angle.py_out,
                self.can_.c_angle.pz_out,
                self.can_.c_angle.alpha_out,
                self.can_.c_angle.beta_out,
                self.can_.c_angle.gama_out]
        return pose 
    
    # 失能校准机械臂
    def Arm_Adjust(self, enable = False, disable = True):
        # 机械臂的电机使能标志位
        self.can_.motor_enable_state = enable
        # 机械臂的电机失能标志位
        self.can_.motor_disable_state = disable
        while True:
            self.can_.Update()
            if not self.can_.write_traj_flag:
                if self.print_targets:
                    os.system('cls' if os.name == 'nt' else 'clear')
                    print('1轴', self.can_._1_link_angle)
                    print('2轴', self.can_._2_link_angle)
                    print('3轴', self.can_._3_link_angle)
                    print('4轴', self.can_._4_link_angle)
                    print('5轴', self.can_._5_link_angle)
                    print('6轴', self.can_._6_link_angle)
                    print('末端x', self.can_.c_angle.px_out)
                    print('末端y', self.can_.c_angle.py_out)
                    print('末端z', self.can_.c_angle.pz_out)
                    print('末端z角度',self. can_.c_angle.alpha_out)
                    print('末端y角度', self.can_.c_angle.beta_out)
                    print('末端x角度', self.can_.c_angle.gama_out)
            if keyboard.is_pressed('q'):
                # 机械臂的电机使能标志位
                self.can_.motor_enable_state = True
                # 机械臂的电机失能标志位
                self.can_.motor_disable_state = False
                self.can_.Update()
                break
            time.sleep(0.1)  # Delay to reduce CPU usage
    
    # 录制轨迹  没有摄像头，此线程没有作用  
    # def Transcribe_Track(self, enable = False, disable = True, show_video = False, save_image = False):
    #     targets = []
    #     # 机械臂的电机使能标志位
    #     self.can_.motor_enable_state = enable
    #     # 机械臂的电机失能标志位
    #     self.can_.motor_disable_state = disable
        
    #     # 视频捕获设置
    #     if show_video:
    #         index = 1
    #         cap = cv2.VideoCapture(0)  # 0是默认摄像头\
    #         cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    #         # 检查摄像头是否成功打开
    #         if not cap.isOpened():
    #             print("Cannot open camera")
    #             return
            
    #     while True:
    #         #显示摄像头
    #         if show_video:
    #             ret, frame = cap.read()
    #             if not ret:
    #                 print("Can't receive frame (stream end?). Exiting ...")
    #                 break
    #             cv2.imshow("R", frame[:,int(640):])
    #             cv2.waitKey(1)
                
    #         self.can_.Update()
    #         if not self.can_.write_traj_flag:
    #             if self.print_targets:
    #                 os.system('cls' if os.name == 'nt' else 'clear')
    #                 print('1轴', self.can_._1_link_angle)
    #                 print('2轴', self.can_._2_link_angle)
    #                 print('3轴', self.can_._3_link_angle)
    #                 print('4轴', self.can_._4_link_angle)
    #                 print('5轴', self.can_._5_link_angle)
    #                 print('6轴', self.can_._6_link_angle)
    #                 print('末端x', self.can_.c_angle.px_out)
    #                 print('末端y', self.can_.c_angle.py_out)
    #                 print('末端z', self.can_.c_angle.pz_out)
    #                 print('末端z角度',self. can_.c_angle.alpha_out)
    #                 print('末端y角度', self.can_.c_angle.beta_out)
    #                 print('末端x角度', self.can_.c_angle.gama_out)
            
    #         # Detect 'w' key press
    #         if keyboard.is_pressed('w'):
    #             new_target = [
    #                 self.can_.c_angle.px_out,
    #                 self.can_.c_angle.py_out,
    #                 self.can_.c_angle.pz_out,
    #                 self.can_.c_angle.alpha_out,
    #                 self.can_.c_angle.beta_out,
    #                 self.can_.c_angle.gama_out
    #             ]
    #             targets.append(new_target)
    #             print("更新目标点为:", targets)
    #             if show_video and save_image:
    #                 cv2.imwrite(f'data/eye_hand_calibration_image/{index}.jpg', frame[:,int(640):])
    #                 index += 1
    #                 print("保存图片到：", f'data/eye_hand_calibration_image/{index}.jpg')
                
    #             time.sleep(1)  # 延迟显示
    #         time.sleep(0.1)  # Delay to reduce CPU usage
    #         if keyboard.is_pressed('q'):
    #             self.claw_state.value = -1.0
    #             break
    #     # 更新文件进行写入姿态
    #     with open('Temp/targets.txt', 'w') as file:
    #         # 遍历二维列表的每一行
    #         for row in targets:
    #             # 将当前行的元素转换为字符串，并用逗号分隔
    #             row_string = ','.join(map(str, row))
    #             # 写入当前行到文件，并添加换行符
    #             file.write(row_string + '\n')
                
    #     print("运动轨迹保存到：", 'Temp/targets.txt')
    #     file.close()    
        
    #     # 机械臂的电机使能标志位
    #     self.can_.motor_enable_state = True
    #     # 机械臂的电机失能标志位
    #     self.can_.motor_disable_state = False
    #     self.can_.Update()  
        
    #     if show_video:
    #         cap.release()
    #         cv2.destroyAllWindows()

    # 校准Pose数据
    def Calibration_Pose(self):
        self.can_.Update()
        # 更新机械臂的初始位置
        Json_Updata("config/pose_config.json", "zero", [self.can_._1_link_angle, self.can_._2_link_angle, self.can_._3_link_angle, self.can_._4_link_angle, self.can_._5_link_angle, self.can_._6_link_angle])
        # 更新机械臂的初始的末端位置
        Json_Updata("config/pose_config.json", "zero_pose", [self.can_.c_angle.px_out, self.can_.c_angle.py_out, self.can_.c_angle.pz_out, self.can_.c_angle.alpha_out, self.can_.c_angle.beta_out, self.can_.c_angle.gama_out])
        print("初始位置、末端位置 ：更新完成")
              
    # 读取轨迹     method参数只能有txt和list   
    def Read_Track(self, method, file_path, reset = True, command = None):
        self.targets.clear()
        
        # 该模式用于测试轨迹任务
        if method == "txt":
            # 打开文件进行读取
            with open(file_path, 'r') as file:
                # 逐行读取文件
                for line in file:
                    # 移除行尾的换行符并按逗号分隔字符串
                    row = line.strip().split(',')
                    # 将分割后的字符串转换为整数，并添加到二维列表中
                    self.targets.append([float(item) for item in row])
                if reset:
                    self.targets.append(self.pose_config["zero_pose"])
                    
        # 该模式用于轨迹任务         
        elif method == "json":
            with open(file_path, 'r',  encoding='utf-8') as f:
                json_data = json.load(f)
            self.targets.extend(json_data[command])
            if reset:
                self.targets.append(self.pose_config["zero_pose"])
                
        # # 该模式用于抓取任务
        # elif method == "list":
        #     self.targets.append(file_path)
        #     if reset:
        #         # 回到拍照位置
        #         self.targets.append(self.pose_config["photo_pose"])
        #         # 放物品
        #         self.targets.append(self.pose_config["place_pose"])
        #         # 回到拍照位置
        #         self.targets.append(self.pose_config["photo_pose"])
                
    # 单轴运动
    def Single_Axis(self, axis, angle):
        self.can_.Update()
        self.can_._1_edit_angle = self.can_._1_link_angle
        self.can_._2_edit_angle = self.can_._2_link_angle
        self.can_._3_edit_angle = self.can_._3_link_angle
        self.can_._4_edit_angle = self.can_._4_link_angle
        self.can_._5_edit_angle = self.can_._5_link_angle
        self.can_._6_edit_angle = self.can_._6_link_angle
        if (0 < axis and axis <= 6):
            self.can_.write_angle_flag = True
            if axis == 1:
                self.can_._1_edit_angle = angle
            elif axis == 2:
                self.can_._2_edit_angle = angle
            elif axis == 3:
                self.can_._3_edit_angle = angle
            elif axis == 4:
                self.can_._4_edit_angle = angle
            elif axis == 5:
                self.can_._5_edit_angle = angle
            elif axis == 6:
                self.can_._6_edit_angle = angle
            self.can_.Update()
        else:
            print("输入轴号错误,请输入1~6轴")
            
                        
    # 执行轨迹
    def Run_Arm(self, start_claw = False):
        # 遍历所有目标位置
        for i in range(len(self.targets)):
            # 设置标志位，表示正在写入轨迹
            self.can_.write_traj_flag = True
            # 输出当前目标位置
            self.can_.out_traj_button(self.targets[i])
            # 初始化六个标志位，用于判断每个轴是否到达目标位置
            t1, t2, t3, t4, t5, t6 = False, False, False, False, False, False
            # 初始化末端执行器的位置和角度
            px_out=212.6780147757499,
            py_out=-0.008595470952314645,
            pz_out=174.7365548995827,
            alpha_out=-0.05488580558178437,
            beta_out=1.5641305671108854,
            gama_out=3.132217683652705
            # 记录当前时间
            ts = time.time()
            # 进入循环，直到所有轴到达目标位置或超时
            while True:
                # 更新CAN总线状态
                self.can_.Update()
                # 如果不再写入轨迹
                if not self.can_.write_traj_flag:
                    # 如果需要打印目标位置
                    if self.print_targets:
                        # 清屏
                        os.system('cls' if os.name == 'nt' else 'clear')
                        # 打印每个轴的角度
                        print('1轴', self.can_._1_link_angle)
                        print('2轴', self.can_._2_link_angle)
                        print('3轴', self.can_._3_link_angle)
                        print('4轴', self.can_._4_link_angle)
                        print('5轴', self.can_._5_link_angle)
                        print('6轴', self.can_._6_link_angle)
                        print('末端x', self.can_.c_angle.px_out)#self.can_.c_angle.px_out
                        print('末端y', self.can_.c_angle.py_out)#self.can_.c_angle.py_out
                        print('末端z', self.can_.c_angle.pz_out)#self.can_.c_angle.pz_out
                        print('末端z角度', self.can_.c_angle.alpha_out)#self.can_.c_angle.alpha_out
                        print('末端y角度', self.can_.c_angle.beta_out)#self.can_.c_angle.beta_out
                        print('末端x角度', self.can_.c_angle.gama_out)#self.can_.c_angle.gama_out
                    if abs(self.can_.c_angle.px_out - self.targets[i][0]) < 0.01:
                        t1 = True
                    if abs(self.can_.c_angle.py_out - self.targets[i][1]) < 0.01:
                        t2 = True
                    if abs(self.can_.c_angle.pz_out - self.targets[i][2]) < 0.01:
                        t3 = True
                    if abs(self.can_.c_angle.alpha_out - self.targets[i][3]) < 0.01:
                        t4 = True
                    if abs(self.can_.c_angle.beta_out - self.targets[i][4]) < 0.01:
                        t5 = True
                    if abs(self.can_.c_angle.gama_out - self.targets[i][5]) < 0.01:
                        t6 = True
                if (t1 and t2 and t3 and t4 and t5 and t6) or time.time() - ts > 2:
                    break
            # if(start_claw):        
            #     if i == 0:
            #         time.sleep(2)
            #         self.claw_state.value = 1.0
            #         time.sleep(2)
            #     if i == 2:
            #         time.sleep(2)
            #         self.claw_state.value = 0.0
            #         time.sleep(2)
    def Move_To_Position(self, target_pose, reset=True):
        '''
        target_pose: 目标位姿[x,y,z,rx,ry,rz]
        reset: 是否回到初始位置
        '''   
        # 清空历史轨迹
        self.targets.clear()
        # 调用逆运动学求解关节角度
        self.can_.c_angle.out_edit_xyz_abg = target_pose
        self.can_.c_angle.IK_PUBLIC_F()  # 计算逆解
        # 检查是否有解
        if self.can_.c_angle.out_result_index == 0:
            print("无解: 无法到达目标点", target_pose)
            return
        # 提取第一个解
        joint_angles = [
            self.can_.c_angle.out_thtaValue[0][0],
            self.can_.c_angle.out_thtaValue[1][0],
            self.can_.c_angle.out_thtaValue[2][0],
            self.can_.c_angle.out_thtaValue[3][0],
            self.can_.c_angle.out_thtaValue[4][0],
            self.can_.c_angle.out_thtaValue[5][0],
        ] 
        target_joint_pose = [
            joint_angles[0] * 180 / math.pi,  # 弧度转角度
            joint_angles[1] * 180 / math.pi,
            joint_angles[2] * 180 / math.pi,
            joint_angles[3] * 180 / math.pi,
            joint_angles[4] * 180 / math.pi,
            joint_angles[5] * 180 / math.pi
        ]
        self.targets.append(target_joint_pose)
        if reset:
            self.targets.append(self.pose_config["zero_pose"])
        self.Run_Arm()

if __name__ == "__main__":
    AC = ArmControl()
    # AC.Set_Arm("COM13", claw_thread=True)端口用于夹爪线程。如果不需要夹爪，则不需要开启线程
    
    
########################################################################################################################################################
#                                                                    机械臂校准                                                                        #
########################################################################################################################################################
    # AC.Arm_Adjust() # 按q退出，保存校准数据
    # AC.Calibration_Pose()
    
########################################################################################################################################################
#                                                                    基础动作                                                                          #
########################################################################################################################################################
    # while True:
    #     reset = True
    #     x = input("输入动作: ")
    #     try:
    #         if(x == "q"):
    #             AC.claw_state.value = -1.0
    #             break
    #         if x == "立正" or x == "右" or x == "左":
    #             reset = False
    #         AC.Read_Track("json", 'config/motion_config.json', reset, command=x)
    #         AC.Run_Arm()
    #     except:
    #         print("动作有误，请重新输入")
        

########################################################################################################################################################
#                                                                     录制轨迹                                                                         #
########################################################################################################################################################
    # AC.Transcribe_Track(show_video=True)
    
    
########################################################################################################################################################
#                                                                     单轴控制                                                                         #
########################################################################################################################################################
    #AC.Single_Axis(1, 50000)

########################################################################################################################################################
#                                                                    到达目标点                                                                          #
########################################################################################################################################################
    # AC.Set_Arm("COM13", claw_thread=True)#夹爪线程，没有夹爪不需要开启
    # 输入目标坐标
    # target_pose = [  212.6780147757499,
    #     -0.008595470952314645,
    #     174.7365548995827,
    #     -0.05488580558178437,
    #     1.5641305671108854,
    #     3.132217683652705]
    # # #移动到目标点
    # AC.Move_To_Position(target_pose, reset=True)
    
########################################################################################################################################################
#                                                                    键盘控制                                                                         #
########################################################################################################################################################
    
    AC.keyboardControl = KeyboardControl(AC.can_)  # 传入AC.can_ can_transfer 实例
    AC.keyboardControl.run()  # 运行键盘控制