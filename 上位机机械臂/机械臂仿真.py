from cProfile import label
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import Qt

from math import *

from 机械臂解算 import ArmSolution,Armsolution2


import serial # pyserial
import time

def sendxy(self,com,x,height,dir="O"):
	portx = com
	bps = 9600
	timex = 1

	# 打开串口，并得到串口对象
	try:
		ser = serial.Serial(portx, bps, timeout=timex)
	except:
		self.bt2.setEnabled(True)
		return 

	x=str(x)[1:]+' '
	height=str(height)
	send_str=bytes([0xA5])+x.encode("utf-8")+height.encode("utf-8")
	for i in range(9-len(height)-len(x)-1):
		send_str+=bytes([0x5A])

	print(send_str)
	ser.write(send_str)

	ser.close() # 关闭串口


def send(self,com,height,dir="O"):
	portx = com
	bps = 9600
	timex = 1

	# 打开串口，并得到串口对象
	try:
		ser = serial.Serial(portx, bps, timeout=timex)
	except:
		self.bt2.setEnabled(True)
		return 

	dir=dir.encode("utf-8")
	height=str(height)
	send_str=bytes([0xA5])+dir+height.encode("utf-8")
	for i in range(9-len(height)-2):
		send_str+=bytes([0x5A])

	print(send_str)
	ser.write(send_str)

	ser.close() # 关闭串口



WIDTH=600
HEIGHT=600

o1=1.10
o2=-0.47
o3=2.52
o4=-0.62

L1=120
L2=35
L3=118
L4=35
L5=137+L4
L6=22

class Drawing(QWidget):
	def __init__(self):
		super().__init__()
		self.initUI()	


		self.slider_1.setValue(-100)
		self.slider_2.setValue(100)
		
	def initUI(self):   
		self.setGeometry(300, 300, WIDTH, HEIGHT)
		self.setWindowTitle('机械臂仿真')      
		self.slider_1 = QSlider(Qt.Horizontal, self)                                       # 1
		self.slider_1.setRange(-200, 100)                                                     # 2
		self.slider_1.valueChanged.connect(lambda: self.on_change_func(self.slider_1))     # 3
 
		self.slider_2 = QSlider(Qt.Horizontal, self)
		self.slider_2.setMinimum(-50)                                                        # 4
		self.slider_2.setMaximum(300)                                                      # 5
		self.slider_2.valueChanged.connect(lambda: self.on_change_func(self.slider_2))

		self.slider_3 = QSlider(Qt.Horizontal, self)
		self.slider_3.setMinimum(0)                                                        # 4
		self.slider_3.setMaximum(180)                                                      # 5
		self.slider_3.valueChanged.connect(lambda: self.on_change_func(self.slider_3))


		self.labelx = QLineEdit('0', self)                                                     # 6
		self.labelx.setFont(QFont('Arial Black', 20))
		self.labely = QLineEdit('0', self)                                                     # 6
		self.labely.setFont(QFont('Arial Black', 20))
		self.labelT = QLabel('0', self)                                                     # 6
		self.labelT.setFont(QFont('Arial Black', 15))
		self.bt1 = QPushButton('确认输入',self)
		self.bt1.clicked.connect(lambda: self.btAction(self.bt1))

		self.COM= QLineEdit('例如COM8', self)                                                     # 6
		self.COM.setFont(QFont('Arial Black', 10))
		self.bt2 = QPushButton('连接通信',self)
		self.bt2.clicked.connect(lambda: self.btAction(self.bt2))

		self.labelT.setText("x,y坐标:")
		self.labelT.move(150,60)
		self.labelx.move(250,60)
		self.labely.move(self.labelx.x()+100,self.labelx.y())
		self.bt1.move(self.labely.x()+100,self.labely.y())
		self.COM.move(self.labelx.x(),self.labelx.y()+60)
		self.bt2.move(self.labely.x(),self.labely.y()+60)
		
		self.labeldir = QLineEdit('0', self)                                                     # 6
		self.labeldir.setFont(QFont('Arial Black', 20))
		self.labeldir.move(250,500)

		self.h_layout = QHBoxLayout()
		self.v_layout = QVBoxLayout()
 
		self.v_layout.addStretch(1)
		self.v_layout.addWidget(self.slider_1)
		# self.v_layout.addWidget(self.labelx)
		# self.v_layout.addWidget(self.labely)
		self.v_layout.addStretch(1)
		self.v_layout.addWidget(self.slider_2)
		self.v_layout.addStretch(20)

		# self.v_layout.addStretch(1)
		self.v_layout.addWidget(self.slider_3)
		# self.v_layout.addStretch(20)

		# self.h_layout.addStretch(2)
		# self.h_layout.addWidget(self.label)

		
		self.setLayout(self.v_layout)
 
	def on_change_func(self, slider):                                                       # 7
		if slider == self.slider_1:
			# self.slider_2.setValue(self.slider_1.value())
			self.labelx.setText(str(self.slider_1.value()))
		elif slider == self.slider_2:
			# self.slider_1.setValue(self.slider_2.value())
			self.labely.setText(str(self.slider_2.value()))
		else:
			self.labeldir.setText(str(self.slider_3.value()))
			if(self.bt2.isEnabled()==False):
				send(self,self.COM.text(),int(self.labeldir.text()),'D')
		global o1,o2,o3,o4
		o1,o2,o3,o4=Armsolution2(int(self.labelx.text()),int(self.labely.text()))
		if(self.bt2.isEnabled()==False):
			sendxy(self,self.COM.text(),int(self.labelx.text()),int(self.labely.text()))
		self.update()

	def btAction(self,bt):
		if bt==self.bt1:
			textxValue = int(self.labelx.text())
			textyValue = int(self.labely.text())
			self.slider_1.setValue(textxValue)
			self.slider_2.setValue(textyValue)
		if bt==self.bt2:
			self.bt2.setEnabled(False)
			sendxy(self,self.COM.text(),int(self.labelx.text()),int(self.labely.text()))


	def paintEvent(self, e): 
		qp = QPainter()
		qp.begin(self)
		self.drawLines(qp)
		qp.end()

	def drawLines(self, qp):

		P1=[300,400]
		P2=[P1[0]+L1*cos(o1), P1[1]-L1*sin(o1)]
		P3=[P1[0]+L2*cos(o2), P1[1]-L2*sin(o2)]
		P4=[P2[0]-L4*cos(o3), P2[1]+L4*sin(o3)]
		P5=[P4[0]+L6*cos(o3-pi/2), P4[1]-L6*sin(o3-pi/2)]
		P6=[P5[0]+L5*cos(o3), P5[1]-L5*sin(o3)]


		pen = QPen(Qt.black, 2, Qt.SolidLine)
		qp.setPen(pen)
		qp.drawLine(P1[0],P1[1],P2[0],P2[1])
		qp.drawLine(P1[0],P1[1],P3[0],P3[1])
		qp.drawLine(P3[0],P3[1],P4[0],P4[1])
		qp.drawLine(P2[0],P2[1],P4[0],P4[1])
		qp.drawLine(P4[0],P4[1],P5[0],P5[1])
		qp.drawLine(P5[0],P5[1],P6[0],P6[1])
		


def main():
	app = QApplication(sys.argv)
	demo = Drawing()
	demo.show()
	sys.exit(app.exec_())
main()