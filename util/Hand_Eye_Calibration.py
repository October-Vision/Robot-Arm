"""
眼在手上 
用采集到的图片信息和机械臂位姿信息计算相机坐标系相对于机械臂末端坐标系的旋转矩阵和平移向量

所有单位为米

"""

import os.path
os.environ['KMP_DUPLICATE_LIB_OK']='True'
import cv2
import numpy as np
import csv
np.set_printoptions(precision=8,suppress=True)

class EyeInHand():
    def __init__(self,):
        self.csv_path = './data/robotToolPose.csv'
        self.image_count = 25 # 照片数量

    def euler_angles_to_rotation_matrix(self,rx, ry, rz):
        # 计算旋转矩阵
        Rx = np.array([[1, 0, 0],
                    [0, np.cos(rx), -np.sin(rx)],
                    [0, np.sin(rx), np.cos(rx)]])
        Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                    [0, 1, 0],
                    [-np.sin(ry), 0, np.cos(ry)]])
        Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                    [np.sin(rz), np.cos(rz), 0],
                    [0, 0, 1]])
        # R = Rz@Ry@Rx   #xyz
        R = Rx@Ry@Rz      #zyx

        return R

    def pose_to_homogeneous_matrix(self,pose):
        x, y, z, rz, ry, rx = pose
        if x > 1: # 把毫米转为米
            x = x / 1000
            y = y / 1000
            z = z / 1000
        R = self.euler_angles_to_rotation_matrix(rx, ry, rz)
        t = np.array([x, y, z]).reshape(3, 1)
        H = np.eye(4)
        H[:3, :3] = R
        H[:3, 3] = t[:, 0]
        return H

    def save_matrices_to_csv(self, matrices, file_name):
        rows, cols = matrices[0].shape
        num_matrices = len(matrices)
        combined_matrix = np.zeros((rows, cols * num_matrices))

        for i, matrix in enumerate(matrices):
            combined_matrix[:, i * cols: (i + 1) * cols] = matrix

        with open(file_name, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            for row in combined_matrix:
                csv_writer.writerow(row)

    def poses_to_matrix_save_csv(self, filepath):
        # 打开文本文件
        with open(filepath, 'r') as file:
            data = file.readlines()
        # 处理每一行，将字符串转换为整数列表
        pose_vectors = []
        for line in data:
            # 分割每一行的字符串，用逗号作为分隔符，并将每个值转换为整数
            int_values = [float(value) for value in line.strip().split(',')]
            pose_vectors.append(int_values)

        matrices = []
        for i in range(0,len(pose_vectors)):
            matrices.append(self.pose_to_homogeneous_matrix(pose_vectors[i]))
        # 将齐次变换矩阵列表存储到 CSV 文件中
        self.save_matrices_to_csv(matrices, self.csv_path)

    def compute_T(self, images_path,corner_point_long,corner_point_short,corner_point_size):
        # 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
        criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
        # 获取标定板角点的位置
        objp = np.zeros((corner_point_long * corner_point_short, 3), np.float32)
        objp[:, :2] = np.mgrid[0:corner_point_long, 0:corner_point_short].T.reshape(-1, 2)     
        # 将世界坐标系建在标定板上，所有点的Z坐标全部为0，所以只需要赋值x和y
        objp = corner_point_size*objp

        obj_points = []     # 存储3D点
        img_points = []     # 存储2D点
        # 遍历文件夹中的图片
        for i in range(self.image_count):   #标定好的图片在images_path路径下，从0.jpg到x.jpg   一次采集的图片最多不超过30张，遍历从0.jpg到30.jpg ，选择能够读取的到的图片
            image = f"{images_path}/{i+1}.jpg"   
            if os.path.exists(image):
                img = cv2.imread(image)
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                size = gray.shape[::-1]
                ret, corners = cv2.findChessboardCorners(gray, (corner_point_long, corner_point_short), None)
                if ret:
                    obj_points.append(objp)
                    corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 在原角点的基础上寻找亚像素角点
                    if [corners2]:
                        img_points.append(corners2)
                    else:
                        img_points.append(corners)
            cv2.destroyAllWindows()
        N = len(img_points)
        
        # 标定,得到图案在相机坐标系下的位姿
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)

        # 机器人末端在基坐标系下的位姿
        tool_pose = np.loadtxt(self.csv_path, delimiter=',')  #与poses_save_csv保存的名字对应上
        R_tool = []
        t_tool = []
        for i in range(int(N)):
            R_tool.append(tool_pose[0:3,4*i:4*i+3])
            t_tool.append(tool_pose[0:3,4*i+3])

        # 调用 cv2.calibrateHandEye 进行手眼标定 (方法: TSAI)
        method_tsai = cv2.CALIB_HAND_EYE_TSAI
        R_cam2gripper_tsai, t_cam2gripper_tsai = cv2.calibrateHandEye(
            R_tool, t_tool, 
            rvecs, tvecs, 
            method=method_tsai
        )
        return R_cam2gripper_tsai, t_cam2gripper_tsai

if __name__ == '__main__':

    EIH = EyeInHand()
    images_path = "data/eye_hand_calibration_image" #手眼标定采集的标定版图片所在路径
    file_path = "data/targets.txt" #采集标定板图片时对应的机械臂末端的位姿 从 第一行到最后一行 需要和采集的标定板的图片顺序进行对应
    corner_point_long=9      #标定板角点数量  长边
    corner_point_short=6
    corner_point_size=0.022        #标定板方格真实尺寸  m
    pose = [180.03314599646225,0.0,140.81726470277647,-0.03337942194439155,0.007238034275528547,-3.141592653589793]

    print("手眼标定采集的标定版图片所在路径", images_path)
    print("采集标定板图片时对应的机械臂末端的位姿", file_path)
    
    EIH.poses_to_matrix_save_csv(file_path)
    rotation_matrix ,translation_vector = EIH.compute_T(images_path,corner_point_long,corner_point_short,corner_point_size)
    print('默认返回tsai方法计算结果,可根据设计情况自行选择合适的矩阵和平移向量 ')
    
    print("////////////////////////////////////////////////////////////////////////////////////////////////")
    print('rotation_matrix:')
    print(rotation_matrix)
    print('translation_vector:')
    print(translation_vector)
    
    print("////////////////////////////////////////////////////////////////////////////////////////////////")
    RT_camera2end = np.eye(4)
    RT_camera2end[0:3,0:3] = rotation_matrix
    RT_camera2end[0:3,3] = translation_vector.reshape(3)
    print('RT_camera2end:')
    print(RT_camera2end)
    print("////////////////////////////////////////////////////////////////////////////////////////////////")
    
    u, v = 280, 264
    Z = 0.24  # 假设已知的深度值，单位为米
    fx, fy, cx, cy = 334.74708104,447.86175651, 308.83371984,  230.12967209
    # 图像坐标转换为相机坐标
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    # 齐次坐标
    P_camera_homogeneous = np.array([X, Y, Z, 1]).reshape(4, 1)
    print("相机坐标系中的坐标:", P_camera_homogeneous)    
    print("////////////////////////////////////////////////////////////////////////////////////////////////")
    
    RT_end2base = EIH.pose_to_homogeneous_matrix(pose)
    print("机械臂末端坐标系到机械臂基坐标系的变换矩阵:")
    print(RT_end2base)
    print("////////////////////////////////////////////////////////////////////////////////////////////////")
    # 从相机坐标系转换到机械臂末端坐标系
    P_end_homogeneous = RT_camera2end @ P_camera_homogeneous
    print("机械臂末端坐标系中的坐标:", P_end_homogeneous)
    print("////////////////////////////////////////////////////////////////////////////////////////////////")

    # 从机械臂末端坐标系转换到机械臂基底坐标系
    P_base_homogeneous = RT_end2base @ P_end_homogeneous
    x,y,z = P_base_homogeneous[0,0],-P_base_homogeneous[1,0],P_base_homogeneous[2,0]
    print("机械臂基底坐标系中的坐标:", x*1000,y*1000,z*1000)
    print("////////////////////////////////////////////////////////////////////////////////////////////////")

    
    
