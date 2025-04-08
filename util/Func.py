import base64
import json
import numpy as np

def Decode_File(base64_data, filename):
    # 解码Base64数据
    data = base64.b64decode(base64_data)
    
    # 保存解码后的音频数据到文件
    with open(filename, 'wb') as output_file:
        output_file.write(data)
    
# json值更新
def Json_Updata(file_path, key, value):
    # 保存机械臂当前位置
    with open(file_path, 'r', encoding='utf-8') as f:
        config =json.load(f)
    # 替换值
    config[key] = value
    # 将更新后的配置写回文件
    with open(file_path, 'w', encoding='utf-8') as f:
        json.dump(config, f, ensure_ascii=False, indent=4)
        
# 获取手眼标定参数(eye_in_hand)
def Get_EyeInHand_Parameter(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)
    data = data["eyeinhand"]
    RT_camera2end = np.array(data['RT_camera2end'])
    return RT_camera2end
        
    
# 获取双目相机参数
def Get_Two_Camera_Parameter(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)
    data = data["two"]
    # -----------------------------------双目相机的基本参数---------------------------------------------------------
    #   left_camera_matrix          左相机的内参矩阵
    #   right_camera_matrix         右相机的内参矩阵
    #
    #   left_distortion             左相机的畸变系数    格式(K1,K2,P1,P2,0)
    #   right_distortion            右相机的畸变系数
    # -------------------------------------------------------------------------------------------------------------
    left_camera_matrix = np.array(data['left_camera_matrix'])
    right_camera_matrix = np.array(data['right_camera_matrix'])
    left_distortion = np.array(data['left_distortion'])
    right_distortion = np.array(data['right_distortion'])
    size = (640, 480)
    R = np.array(data['R'])
    T = np.array(data['T'])
    parameter = [left_camera_matrix, left_distortion, right_camera_matrix, right_distortion, size, R, T]
    return parameter

# 获取单目相机参数
def Get_One_Camera_Parameter(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)
    data = data["one"]
    
    camera_matrix = np.array(data['camera_matrix'])
    camera_distortion = np.array(data['camera_distortion'])
    
    return camera_matrix, camera_distortion
