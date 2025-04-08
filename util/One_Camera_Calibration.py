import cv2
import numpy as np
import glob

class CameraCalibrator():
    def __init__(self):
        self.w = 9  # 实际棋盘格的宽 - 1
        self.h = 6  # 实际棋盘格的高 - 1
        self.square_size = 22.0  # 棋盘格每个方格的尺寸，单位mm
        self.objpoints = []
        self.imgpoints = []
        self.images_path = './data/one_calibration_image/*.jpg'


    def run_calibration(self,images_path,w,h,square_size,pattern='CMD'):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((w*h,3), np.float32)
        objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2)
        objp = objp*square_size  # 18.1 mm

        images = glob.glob(images_path)
        i = 0
        processed_images = []
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            u,v = img.shape[:2]
            ret, corners = cv2.findChessboardCorners(gray, (w,h),None)
            if ret == True:
                # print("i:", i)
                i = i+1
                # 在原角点的基础上寻找亚像素角点
                cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                #追加进入世界三维点和平面二维点中
                self.objpoints.append(objp)
                self.imgpoints.append(corners)
                # 将角点在图像上显示
                cv2.drawChessboardCorners(img, (w,h), corners, ret)
                if pattern == 'GUI':
                    processed_images.append(img)
                elif pattern == 'CMD':
                    cv2.namedWindow('findCorners', cv2.WINDOW_NORMAL)
                    cv2.resizeWindow('findCorners', 640, 480)
                    cv2.imshow('findCorners',img)
                    cv2.waitKey(200)
        cv2.destroyAllWindows()

        print('正在计算')
        #标定
        ret, mtx, dist, rvecs, tvecs = \
            cv2.calibrateCamera(self.objpoints, self.imgpoints, gray.shape[::-1], None, None)
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (u, v), 0, (u, v))
        print('计算完毕')
        if pattern == 'CMD':
            print("ret:",ret  )
            print("mtx:\n",mtx)      # 内参数矩阵
            print("dist畸变值:\n",dist   )   # 畸变系数   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)
            print("rvecs旋转（向量）外参:\n",rvecs)   # 旋转向量  # 外参数
            print("tvecs平移（向量）外参:\n",tvecs  )  # 平移向量  # 外参数
            print('newcameramtx外参',newcameramtx)
        return ret,mtx,dist,u,v,processed_images,rvecs,tvecs,newcameramtx
    
    def start_capture(self,mtx,dist,u,v):
        camera = cv2.VideoCapture(0)
        while True:
            (grabbed,frame)=camera.read()
            h1, w1 = frame.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (u, v), 0, (u, v))
            # 纠正畸变
            dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
            #dst2 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
            mapx,mapy=cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w1,h1),5)
            dst2=cv2.remap(frame,mapx,mapy,cv2.INTER_LINEAR)
            # 裁剪图像，输出纠正畸变以后的图片
            x, y, w1, h1 = roi
            dst1 = dst1[y:y + h1, x:x + w1]

            #cv2.imshow('frame',dst2)
            #cv2.imshow('dst1',dst1)
            cv2.imshow('dst2', dst2)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # 按q
                break
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    CC = CameraCalibrator()
    ret, mtx, dist, u, v, processed_images, rvecs, tvecs, newcameramtx = CC.run_calibration(CC.images_path, CC.w, CC.h, CC.square_size)
    CC.start_capture(mtx, dist, u, v)

    