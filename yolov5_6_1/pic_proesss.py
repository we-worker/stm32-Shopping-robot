# import cv2 as cv
# import numpy as np

# src = cv.imread("1.jpg")
# gray_img = cv.cvtColor(src, cv.COLOR_BGR2GRAY)


# dst = cv.equalizeHist(gray_img)
# # 高斯滤波降噪
# gaussian = cv.GaussianBlur(dst, (9, 9), 0)
# # cv.imshow("gaussian", gaussian)

# # 边缘检测
# edges = cv.Canny(gaussian, 0, 255)
# cv.imshow("edges", edges)
# # Hough 直线检测
# # 重点注意第四个参数 阈值，只有累加后的值高于阈值时才被认为是一条直线，也可以把它看成能检测到的直线的最短长度（以像素点为单位）
# # 在霍夫空间理解为：至少有多少条正弦曲线交于一点才被认为是直线
# lines = cv.HoughLines(edges, 1.0, np.pi/180, 150)

# # 将检测到的直线通过极坐标的方式画出来
# print(lines.ndim)
# print(lines.shape)
# for line in lines:
#     # line[0]存储的是点到直线的极径和极角，其中极角是弧度表示的，theta是弧度
#     rho, theta = line[0]

#     # 下述代码为获取 (x0,y0) 具体值
#     a = np.cos(theta)
#     b = np.sin(theta)

#     x0 = a*rho
#     y0 = b*rho

#     # 下图 1000 的目的是为了将线段延长
#     # 以 (x0,y0) 为基础，进行延长
#     x1 = int(x0+1000*(-b))
#     y1 = int(y0+1000*a)
#     x2 = int(x0-1000*(-b))
#     y2 = int(y0-1000*a)

#     cv.line(src, (x1, y1), (x2, y2), (0, 255, 0), 2)
# cv.imshow("src", src)
# cv.waitKey()
# cv.destroyAllWindows()

import cv2 as cv
import numpy as np
import time
# src = cv.imread("1.jpg")
cap = cv.VideoCapture(0)

def nothing():
	pass
	# cv.namedWindow("bar")
	# cv.createTrackbar("num1", "bar", 0, 255, nothing)
	# cv.createTrackbar("num2", "bar", 0, 255, nothing)
frame_count=0
while True:
	
	frame_count = frame_count + 1	# 已读取的帧数计算
	# time.sleep(0.01)
	if frame_count % int(10) == 0:  # 当累计帧数为你想要的帧率的倍数时，再进行图片处理操作！
		ret, frame = cap.read()

		gray_img = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

		dst = cv.equalizeHist(gray_img)
		# 高斯滤波降噪
		gaussian = cv.GaussianBlur(dst, (9, 9), 0)
		# cv.imshow("gaussian", gaussian)



		edges = cv.Canny(gaussian, 0, 255)
		cv.imshow("edges", edges)

		lines = cv.HoughLinesP(edges, 1.0, np.pi/180, 100 ,
							minLineLength=0, maxLineGap=180)
		# print(type(lines))
		k=[]
		Lmin=639
		Rmax=0
		try:
			for line in lines:
				x1, y1, x2, y2 = line[0]
				kt=(y2-y1)/(x2-x1)
				k.append(kt)
				if(-0.5<kt<0.5 or kt>10 or kt<-10):
					cv.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
			
			for i in range(len(k)):
				if(k[i]<-10 or k[i]>10):
					if(lines[i][0][0]<Lmin):
						Lmin=lines[i][0][0]
					if(lines[i][0][0]>Rmax):
						Rmax=lines[i][0][0]
			if((Rmax-Lmin)<500 or (Rmax-Lmin)>800):
				continue
			cv.imshow("src", frame[:,Lmin:Rmax])
		except:
			continue
			
		
		if cv.waitKey(1) & 0xFF == ord('q'):
			break

cap.release()
cv.destroyAllWindows()
