from datetime import datetime
import json
import threading
import time
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QMainWindow
from PyQt5 import QtCore
from PyQt5.QtGui import QImage, QPixmap
import cv2
import numpy as np

from GUI.Calibration_UI import Ui_Calibration
from util.One_Camera_Calibration import CameraCalibrator
from util.Hand_Eye_Calibration import EyeInHand
from util.Two_Camera_Calibration import Two_Camera_Clibration

class CalibrationFunc(QMainWindow,Ui_Calibration):
    def __init__(self,AC):
        QMainWindow.__init__(self)
        Ui_Calibration.__init__(self)
        self.setupUi(self)

        self.CC = None
        self.AC = AC
        self.w = None
        self.h = None
        self.square_size = None
        self.camera = None
        self.images_path = './data/one_calibration_image/*.jpg'
        self.camera_isOpened = False
        self.i = 0
        self.cap = None
        self.selection = None
        self.targets = []

        with open('config/argument.json', 'r', encoding='utf-8') as f:
            self.argument = json.load(f)
        self.camera_index = self.argument["camera_index"]  #相机的索引


        self.pushButton.clicked.connect(self.Open_Camera)
        self.pushButton_2.clicked.connect(self.Take_Photo)
        self.pushButton_4.clicked.connect(self.Get_Arguments)
        self.pushButton_3.clicked.connect(self.Select_Type)
        self.pushButton_5.clicked.connect(self.Calibration)
        self.pushButton_6.clicked.connect(self.save_Arguments)

    def closeEvent(self, event):
        if self.cap is not None:
            if self.cap.isOpened():
                self.cap.release()
                cv2.destroyAllWindows()

    def Change_Enable(self, state, pattern = "select"):
        if pattern == 'select':
            self.pushButton_4.setEnabled(state)
            self.pushButton_5.setEnabled(state)
            self.pushButton_6.setEnabled(state)
            self.pushButton.setEnabled(state)
            self.pushButton_2.setEnabled(state)
        if pattern == "camera":
            self.pushButton_2.setEnabled(state)

    def Open_Camera(self):
        self.i = 0
        self.Change_Enable(False, pattern = "select")
        self.Change_Enable(True, pattern = "camera")
        if self.selection == "手眼标定":
            # 机械臂的电机使能标志位
            self.AC.can_.motor_enable_state = False
            # 机械臂的电机失能标志位
            self.AC.can_.motor_disable_state = True
            with open('data/targets.txt', "w") as file:
                pass
        try:
            self.label.clear()
            self.label_2.clear()
            self.camera_isOpened = True
            self.cap = cv2.VideoCapture(self.camera_index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            if not self.cap.isOpened():
                self.Update_TextEdit("无法打开摄像头")
                return self.cap
            timer = QtCore.QTimer(self.centralwidget)
            timer.timeout.connect(self.Show_Video)
            timer.start(33)  # 30帧每秒
        except Exception as e:
            print("摄像头开启失败："+str(e))
            self.Update_TextEdit("摄像头开启失败："+str(e))
    
    def Show_Video(self):
        ret,frame = self.cap.read()
        self.frame = frame
        if ret:
            frame_l = frame[:,:640]
            frame_R = frame[:, 640:]
            img_r_rgb = cv2.cvtColor(frame_R, cv2.COLOR_BGR2RGB)
            img_l_rgb = cv2.cvtColor(frame_l, cv2.COLOR_BGR2RGB)
            height_R, width_R, _ = img_r_rgb.shape
            height_l, width_l, _ = img_l_rgb.shape
            bytes_per_line_R = 3 * width_R
            bytes_per_line_l = 3 * width_l
            R_img = QImage(img_r_rgb.data.tobytes(), width_R, height_R, bytes_per_line_R, QImage.Format_RGB888)
            l_img = QImage(img_l_rgb.data.tobytes(), width_l, height_l, bytes_per_line_l, QImage.Format_RGB888)
            pixmap_R = QPixmap.fromImage(R_img)
            pixmap_l = QPixmap.fromImage(l_img)
            if self.selection == "单目标定" or self.selection == "手眼标定":
                self.label_2.setPixmap(pixmap_R)
                self.label_2.setScaledContents(True)
            elif self.selection == "双目标定":
                self.label_2.setPixmap(pixmap_l)
                self.label_2.setScaledContents(True)
                self.label.setPixmap(pixmap_R)
                self.label.setScaledContents(True)


    def Take_Photo(self):
        self.i += 1
        if self.selection == "双目标定":
            save_left_path = f'data/two_calibration_image/left/{self.i}.jpg'
            save_right_path = f'data/two_calibration_image/right/{self.i}.jpg'
            cv2.imwrite(save_left_path, self.frame[:,:int(640)])
            cv2.imwrite(save_right_path, self.frame[:,int(640):])
            self.Update_TextEdit(f'第{self.i}张')
        else:
            if self.selection == "单目标定":
                save_path = f'data/one_calibration_image/{self.i}.jpg'
            elif self.selection == "手眼标定":
                self.AC.can_.Update()
                new_target = [
                        self.AC.can_.c_angle.px_out,
                        self.AC.can_.c_angle.py_out,
                        self.AC.can_.c_angle.pz_out,
                        self.AC.can_.c_angle.alpha_out,
                        self.AC.can_.c_angle.beta_out,
                        self.AC.can_.c_angle.gama_out
                    ]
                save_path = f'data/eye_hand_calibration_image/{self.i}.jpg'
                with open('data/targets.txt', 'a') as file:
                    row_string = ','.join(map(str, new_target))
                    # 写入当前行到文件，并添加换行符
                    file.write(row_string + '\n')
            cv2.imwrite(save_path, self.frame[:,int(640):])
            pixmap = QPixmap(save_path)
            self.label.setPixmap(pixmap)
            self.label.setScaledContents(True)
            self.Update_TextEdit(f'第{self.i}张')
            

    def Get_Arguments(self):
        self.w = int(self.textEdit_2.toPlainText()) - 1 # 实际棋盘格的宽 - 1
        self.h = int(self.textEdit_3.toPlainText()) - 1 # 实际棋盘格的高 - 1
        self.square_size = float(self.textEdit_4.toPlainText())
        self.Update_TextEdit(f'参数\n列：{self.w}\n行：{self.h}\n宽度{self.square_size}mm')


    def Select_Type(self):
        # 机械臂的电机使能标志位
        self.AC.can_.motor_enable_state = True
        # 机械臂的电机失能标志位
        self.AC.can_.motor_disable_state = True
        if self.cap:
            if self.cap.isOpened():
                self.cap.release()
                cv2.destroyAllWindows()
        self.Change_Enable(True, pattern = "select")
        self.label.clear()
        self.label_2.clear()
        self.selection = self.comboBox.currentText()
        if self.selection == "单目标定":
            self.CC = CameraCalibrator()
            self.Update_TextEdit(f'单目标定，可选择拍照或进行标定')
        elif self.selection == "双目标定":
            self.TCC = Two_Camera_Clibration()
            self.Update_TextEdit(f'双目标定，可选择拍照或进行标定')
        elif self.selection == "手眼标定":
            self.EIH = EyeInHand()
            self.Update_TextEdit(f'手眼标定，可选择拍照或进行标定')
            

    def Calibration(self):
        if self.selection == "单目标定":
            self.One_Eye_Calibrator()
        elif self.selection == "双目标定":
            self.Two_Eye_Calibrator()
        elif self.selection == "手眼标定":
            self.Hand_Eye_Calibrator()

    def One_Eye_Calibrator(self):
        ret, mtx, dist, u, v, processed_images, rvecs, tvecs, newcameramtx = self.CC.run_calibration(self.images_path, self.w, self.h,self.square_size,pattern='GUI')
        self.mtx,self.dist,self.u,self.v = mtx,dist,u,v
        self.Update_TextEdit(f'ret:{ret}\nmtx:\n{mtx}\ndist畸变值:\n{dist}\nrvecs旋转（向量）外参:\n{rvecs}\ntvecs平移（向量）外参:\n{tvecs}\nnewcameramtx外参:{newcameramtx}')
        
    def Two_Eye_Calibrator(self):
        self.cameraMatrix1,self.dist1,self.cameraMatrix2,self.dist2,self.R,self.T = self.TCC.calibration_run(self.w,self.h,self.square_size)
        self.Update_TextEdit(f"stereoCalibrate : \nCamera matrix left : \n{self.cameraMatrix1}\ndistCoeffs left  : \n{self.dist1}\ncameraMatrix left : \n{self.cameraMatrix2}\ndistCoeffs left : \n{self.dist2}\nR : \n{self.R}\nT : \n{self.T}")


    def Hand_Eye_Calibrator(self):
        try:
            images_path = "data/eye_hand_calibration_image" #手眼标定采集的标定版图片所在路径
            file_path = "data/targets.txt" #采集标定板图片时对应的机械臂末端的位姿 从 第一行到最后一行 需要和采集的标定板的图片顺序进行对应
            pose = [180.03314599646225,0.0,140.81726470277647,-0.03337942194439155,0.007238034275528547,-3.141592653589793]
            self.Update_TextEdit(f"手眼标定采集的标定版图片所在路径:{images_path}\n采集标定板图片时对应的机械臂末端的位姿:{file_path}")

            
            self.EIH.poses_to_matrix_save_csv(file_path)
            rotation_matrix ,translation_vector = self.EIH.compute_T(images_path,self.w,self.h,(self.square_size/1000))
            print('默认返回tsai方法计算结果,可根据设计情况自行选择合适的矩阵和平移向量 ')
            self.Update_TextEdit(f"rotation_matrix:{rotation_matrix}\ntranslation_vector:{translation_vector}")
            
            
            RT_camera2end = np.eye(4)
            RT_camera2end[0:3,0:3] = rotation_matrix
            RT_camera2end[0:3,3] = translation_vector.reshape(3)
            self.RT_camera2end = RT_camera2end
            self.Update_TextEdit(f'RT_camera2end:{RT_camera2end}')
            
            u, v = 280, 264
            Z = 0.24  # 假设已知的深度值，单位为米
            fx, fy, cx, cy = 334.74708104,447.86175651, 308.83371984,  230.12967209
            # 图像坐标转换为相机坐标
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            # 齐次坐标
            P_camera_homogeneous = np.array([X, Y, Z, 1]).reshape(4, 1)
            self.Update_TextEdit(f'相机坐标系中的坐标:{P_camera_homogeneous}')  
            
            RT_end2base = self.EIH.pose_to_homogeneous_matrix(pose)
            self.Update_TextEdit(f'机械臂末端坐标系到机械臂基坐标系的变换矩阵:{RT_end2base}')  
            # 从相机坐标系转换到机械臂末端坐标系
            P_end_homogeneous = RT_camera2end @ P_camera_homogeneous
            self.Update_TextEdit(f'机械臂末端坐标系中的坐标:{P_end_homogeneous}')  


            # 从机械臂末端坐标系转换到机械臂基底坐标系
            P_base_homogeneous = RT_end2base @ P_end_homogeneous
            x,y,z = P_base_homogeneous[0,0],-P_base_homogeneous[1,0],P_base_homogeneous[2,0]
            self.Update_TextEdit(f'机械臂基底坐标系中的坐标:{x*1000,y*1000,z*1000}')  
        except:
            self.Update_TextEdit(f'没有数据')


    def Show_Camera(self):
        (grabbed, frame) = self.camera.read()
        if not grabbed:
            return

        h1, w1 = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (self.u, self.v), 0, (self.u, self.v))
        # 纠正畸变
        dst1 = cv2.undistort(frame, self.mtx, self.dist, None, newcameramtx)
        mapx, mapy = cv2.initUndistortRectifyMap(self.mtx, self.dist, None, newcameramtx, (w1, h1), cv2.CV_32FC1)
        dst2 = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
        # 裁剪图像，输出纠正畸变以后的图片
        x, y, w1, h1 = roi
        dst1 = dst1[y:y + h1, x:x + w1]

        # 获取label_2的大小
        label_size = self.label_2.size()

        # 调整图像尺寸以适应label_2的大小
        resized_dst2 = cv2.resize(dst2, (label_size.width(), label_size.height()))

        # 将OpenCV图像转换为QImage
        q_image = QImage(resized_dst2.data, resized_dst2.shape[1], resized_dst2.shape[0], QImage.Format_RGB888).rgbSwapped()

        # 将QImage转换为QPixmap，并设置到label_2上
        pixmap = QPixmap.fromImage(q_image)
        self.label_2.setPixmap(pixmap)
        self.label_2.setScaledContents(True)

    def save_Arguments(self):
        config_path = 'config/calibration_parameter.json'
        with open(config_path,'r') as file:
            config_data = json.load(file)
        if self.selection == "单目标定":
            config_data["one"]["camera_matrix"] = self.mtx.tolist()
            config_data["one"]["camera_distortion"] = self.dist.tolist()
        elif self.selection == "双目标定":
            # self.cameraMatrix1,self.dist1,self.cameraMatrix2,self.dist2,self.R,self.T
            config_data["two"]["left_camera_matrix"] = self.cameraMatrix1.tolist()
            config_data["two"]["right_camera_matrix"] = self.cameraMatrix2.tolist()
            config_data["two"]["left_distortion"] = self.dist1.tolist()
            config_data["two"]["right_distortion"] = self.dist2.tolist()
            config_data["two"]["R"] = self.R.tolist()
            config_data["two"]["T"] = self.T.tolist()
        elif self.selection == "手眼标定":
            config_data["two"]["RT_camera2end"] = self.RT_camera2end.tolist()


        with open(config_path, 'w') as file:
            json.dump(config_data, file, indent=4)
        self.Update_TextEdit(f'保存完毕')


    def Update_TextEdit(self, text):
        # 获取当前的时间
        now = datetime.now()
        formatted_date_time = now.strftime("%Y-%m-%d %H:%M:%S")
        self.textEdit.append( "--------" + formatted_date_time + "--------")
        self.textEdit.append(text)