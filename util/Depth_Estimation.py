from util.Func import Get_Two_Camera_Parameter 
import cv2
import numpy as np
import math


# 立体校正检验----画线
def draw_line(image1, image2):
    # 建立输出图像
    height = max(image1.shape[0], image2.shape[0])
    width = image1.shape[1] + image2.shape[1]
 
    output = np.zeros((height, width, 3), dtype=np.uint8)
    output[0:image1.shape[0], 0:image1.shape[1]] = image1
    output[0:image2.shape[0], image1.shape[1]:] = image2
 
    # 绘制等间距平行线
    line_interval = 50  # 直线间隔：50
    for k in range(height // line_interval):
        cv2.line(output, (0, line_interval * (k + 1)), (2 * width, line_interval * (k + 1)), (0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
 
    return output

# 相机矫正
def Rectify(parameter):
    left_camera_matrix, left_distortion, right_camera_matrix, right_distortion, size, R, T = parameter
    
    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                    right_camera_matrix, right_distortion, size, R,
                                                                    T)

    # 校正查找映射表,将原始图像和校正后的图像上的点一一对应起来
    left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
    right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)
    
    return left_map1, left_map2, right_map1, right_map2, Q

# 计算视差
def Count_Disparity(left_image, right_image, left_map1, left_map2, right_map1, right_map2, blockSize = 9, num = 9, minDisparity =20):
    # 将BGR格式转换成灰度图片，用于畸变矫正
    imgL = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
    imgR = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

    # 重映射，就是把一幅图像中某位置的像素放置到另一个图片指定位置的过程。
    # 依据MATLAB测量数据重建无畸变图片,输入图片要求为灰度图
    img1_rectified = cv2.remap(imgL, left_map1, left_map2, cv2.INTER_LINEAR)
    img2_rectified = cv2.remap(imgR, right_map1, right_map2, cv2.INTER_LINEAR)

    # ------------------------------------SGBM算法----------------------------------------------------------
    #   blockSize                   深度图成块，blocksize越低，其深度图就越零碎，0<blockSize<10
    #   img_channels                BGR图像的颜色通道，img_channels=3，不可更改
    #   numDisparities              SGBM感知的范围，越大生成的精度越好，速度越慢，需要被16整除，如numDisparities
    #                               取16、32、48、64等
    #   mode                        sgbm算法选择模式，以速度由快到慢为：STEREO_SGBM_MODE_SGBM_3WAY、
    #                               STEREO_SGBM_MODE_HH4、STEREO_SGBM_MODE_SGBM、STEREO_SGBM_MODE_HH。精度反之
    # ------------------------------------------------------------------------------------------------------
    img_channels = 3
    # blockSize = 9
    # num = 9
    # minDisparity = 20

    stereo = cv2.StereoSGBM_create(minDisparity = minDisparity,
                                    numDisparities=16 * num,
                                    blockSize=blockSize,
                                    P1=8 * img_channels * blockSize * blockSize,
                                    P2=32 * img_channels * blockSize * blockSize,
                                    disp12MaxDiff=-1,
                                    preFilterCap=1,
                                    uniquenessRatio=10,
                                    speckleWindowSize=100,
                                    speckleRange=100,
                                    mode=cv2.STEREO_SGBM_MODE_HH)
    # 计算视差
    disparity = stereo.compute(img1_rectified, img2_rectified)
    
    return disparity

# 计算范围深度图距离(yolo)
def Count_Range_Depth(disparity, rectangle, Q, show = False, resize = False):
    x, y, w, h = rectangle
    if resize:
        x, y, w, h = x * 2, y * 2, w * 2, h * 2
    x += 125 # 视差图偏移
    
    # 计算中心点
    X, Y = x + w // 2, y + h // 2
    # 生成深度图（颜色图）
    dis_color = disparity
    dis_color = cv2.normalize(dis_color, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    # 空洞填充
    dis_color = cv2.medianBlur(dis_color, 5)
    dis_color = cv2.GaussianBlur(dis_color, (5, 5), 0)
    dis_color = cv2.applyColorMap(dis_color, 2)
    if show:
        cv2.imshow("disparity", dis_color)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
    # # 获取ROI
    # roi = dis_color[y:y+h, x:x+w]
    # # 找到ROI中出现最多的颜色
    # colors, counts = np.unique(roi.reshape(-1, 3), axis=0, return_counts=True)
    # most_common_color = colors[np.argmax(counts)]
    # # 将修改后的ROI放回原图
    # dis_color[y:y+h, x:x+w] = most_common_color

    if show:
        cv2.imshow("Modified Disparity", dis_color)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    # 计算三维坐标数据值
    threeD = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=True)
    # 计算出的threeD，需要乘以16，才等于现实中的距离
    threeD = threeD * 16
    
    distance = math.sqrt(threeD[Y][X][0] ** 2 + threeD[Y][X][1] ** 2 + threeD[Y][X][2] ** 2)
    distance = distance / 1000.0  # mm -> m
    # print("距离是：", distance, "m")
    return distance, dis_color
    

# 计算点深度图距离
def Count_Point_Depth(disparity, point, Q, show = False):
    X, Y = point
    # 生成深度图（颜色图）
    dis_color = disparity
    dis_color = cv2.normalize(dis_color, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    # 空洞填充
    dis_color = cv2.medianBlur(dis_color, 5)
    dis_color = cv2.GaussianBlur(dis_color, (5, 5), 0)
    dis_color = cv2.applyColorMap(dis_color, 2)
    if show:
        cv2.imshow("disparity", dis_color)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        # 计算三维坐标数据值
    threeD = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=True)
    # 计算出的threeD，需要乘以16，才等于现实中的距离
    threeD = threeD * 16
    
    distance = math.sqrt(threeD[Y][X][0] ** 2 + threeD[Y][X][1] ** 2 + threeD[Y][X][2] ** 2)
    distance = distance / 1000.0  # mm -> m
    print("距离是：", distance, "m")
        
if __name__ == '__main__':
    left = "left.jpg"
    right = "right.jpg"
    
    left = cv2.imread(left)
    right = cv2.imread(right)
    
    parameter = Get_Two_Camera_Parameter("config/calibration_parameter.json")
    
    left_map1, left_map2, right_map1, right_map2, Q = Rectify(parameter)
    disparity = Count_Disparity(left, right, left_map1, right_map2, left_map1, right_map2)
    Count_Range_Depth(disparity, [255,208,107,87], Q, True)

