from cProfile import label
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import Qt

from math import *


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
HEIGHT=300


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
		self.slider_1.setRange(-230, 50)                                                     # 2
		self.slider_1.valueChanged.connect(lambda: self.on_change_func(self.slider_1))     # 3
 
		self.slider_2 = QSlider(Qt.Horizontal, self)
		self.slider_2.setMinimum(-100)                                                        # 4
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
		self.labeldirT = QLabel('0', self)                                                     # 6
		self.labeldirT.setFont(QFont('Arial Black', 15))
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
		self.labeldir.move(100,120)
		self.labeldirT.setText("方向:")
		self.labeldirT.move(0,120)

		self.h_layout = QHBoxLayout()
		self.v_layout = QVBoxLayout()
 
		# self.v_layout.addStretch(1)
		self.v_layout.addWidget(self.slider_1)
		# self.v_layout.addWidget(self.labelx)
		# self.v_layout.addWidget(self.labely)
		# self.v_layout.addStretch(1)
		self.v_layout.addWidget(self.slider_2)
		# self.v_layout.addStretch(20)

		self.v_layout.addStretch(2)
		self.v_layout.addWidget(self.slider_3)
		# self.v_layout.addStretch(20)

		self.v_layout.addStretch(12)
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
		if(self.bt2.isEnabled()==False):
			sendxy(self,self.COM.text(),int(self.labelx.text()),int(self.labely.text()))
		self.update()

	def btAction(self,bt):
		if bt==self.bt1:
			textxValue = int(self.labelx.text())
			textyValue = int(self.labely.text())
			textdValue = int(self.labeldir.text())
			self.slider_1.setValue(textxValue)
			self.slider_2.setValue(textyValue)
			self.slider_3.setValue(textdValue)
		if bt==self.bt2:
			self.bt2.setEnabled(False)
			sendxy(self,self.COM.text(),int(self.labelx.text()),int(self.labely.text()))


	def paintEvent(self, e): 
		qp = QPainter()
		qp.begin(self)
		# self.drawLines(qp)
		qp.end()




def main():
	app = QApplication(sys.argv)
	demo = Drawing()
	demo.show()
	sys.exit(app.exec_())
main()