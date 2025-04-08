import cv2
import os
import numpy as np

class Two_Camera_Clibration():
    def __init__(self):
        self.leftpath = r'data\two_calibration_image\left'
        self.rightpath = r'data\two_calibration_image\right'
        self.w = 9
        self.h = 6
        self.square_size = 22.0
        self.imgpoints_l = []    #存放左图像坐标系下角点位置
        self.imgpoints_r = []    #存放左图像坐标系下角点位置
        self.objpoints = []   #存放世界坐标系下角点位置

    def calibration_run(self,w,h,square_size):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((1, w*h, 3), np.float32)
        objp[0,:,:2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2)
        objp[0,:,0] *= square_size
        objp[0,:,1] *= square_size

        for ii in os.listdir(self.leftpath):
            img_l = cv2.imread(os.path.join(self.leftpath,ii))
            gray_l = cv2.cvtColor(img_l,cv2.COLOR_BGR2GRAY)
            img_r = cv2.imread(os.path.join(self.rightpath,ii))
            gray_r = cv2.cvtColor(img_r,cv2.COLOR_BGR2GRAY)
            ret_l, corners_l = cv2.findChessboardCorners(gray_l, (w,h),None)   #检测棋盘格内角点
            ret_r, corners_r = cv2.findChessboardCorners(gray_r, (w,h),None)
            if ret_l and ret_r:
                self.objpoints.append(objp)
                corners2_l = cv2.cornerSubPix(gray_l,corners_l,(11,11),(-1,-1),criteria) 
                self.imgpoints_l.append(corners2_l)
                corners2_r = cv2.cornerSubPix(gray_r,corners_r,(11,11),(-1,-1),criteria)
                self.imgpoints_r.append(corners2_r)
                #img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2,ret)
                #cv2.imwrite('./ChessboardCornersimg.jpg', img)
        ret, mtx_l, dist_l, rvecs_l, tvecs_l = cv2.calibrateCamera(self.objpoints, self.imgpoints_l, gray_l.shape[::-1],None,None)  #先分别做单目标定
        ret, mtx_r, dist_r, rvecs_r, tvecs_r = cv2.calibrateCamera(self.objpoints, self.imgpoints_r, gray_r.shape[::-1],None,None)

        retval, cameraMatrix1, dist1, cameraMatrix2, dist2, R, T, E, F = \
            cv2.stereoCalibrate(self.objpoints, self.imgpoints_l, self.imgpoints_r, mtx_l, dist_l, mtx_r, dist_r, gray_l.shape[::-1])   #再做双目标定

        print("stereoCalibrate : \n")
        print("Camera matrix left : \n")
        print(cameraMatrix1)
        print("distCoeffs left  : \n")
        print(dist1)
        print("cameraMatrix left : \n")
        print(cameraMatrix2)
        print("distCoeffs left : \n")
        print(dist2)
        print("R : \n")
        print(R)
        print("T : \n")
        print(T)
        print("E : \n")
        print(E)
        print("F : \n")
        print(F)
        return cameraMatrix1,dist1,cameraMatrix2,dist2,R,T

if __name__ == "__main__":
    TCC = Two_Camera_Clibration()
    TCC.calibration_run(TCC.w,TCC.h,TCC.square_size)