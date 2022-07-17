import re
import serial as ser
import time


last_time=0
box=[]
# se = ser.Serial("/dev/ttyTHS1", 9600, timeout=1)


portx = "COM5"
bps = 9600
timex = 1
se = ser.Serial(portx, bps, timeout=timex)

need_remove=[]#box在一定范围内才发送，因为每次只能抓一格的东西,所以要剔除一些	
def Get_True_box(boxin):
	global last_time
	global box

	# if(time.time()-last_time>=60):
	# 	last_time=time.time()

	box=boxin

	

	# print("左上点的坐标为：(" + str(box[0+num*4]) + "," + str(box[1+num*4]) + ")，右下点的坐标为(" + str(box[2+num*4]) + "," + str(box[3+num*4]) + ")")
	# print("点{}:({},{})".format(num,box[0+num*2],box[1+num*2]))
	Commuication()
	
def send(dir,height):
	dir=(str(640-dir)+' ').encode("utf-8")
	h_send=''
	height=int(480-(height))
	h_send=str(height).encode("utf-8")
	send_str=bytes([0xA5])+dir+h_send
	for i in range(9-len(h_send)-len(dir)-1):
		send_str+=bytes([0x5A])
	# print(send_str)
	se.write(send_str)
	print("发送",send_str)

def sendStr(str):

	h_send=str.encode("utf-8")
	send_str=bytes([0xA5])+h_send

	for i in range(9-len(h_send)-1):
		send_str+=bytes([0x5A])

	se.write(send_str)
	print("发送",send_str)

def Commuication():
	global box

	try:
		confirm_box=[]
		#筛选一遍box
		for i in range(len(box)):
		#box在一定范围内才发送，因为每次只能抓一格的东西	
			if(box[i][0]<520 and box[i][0]>90):
				confirm_box.append(box[i])
		
		if(len(confirm_box)==0):
			sendStr("None")
			return
		else:
			sendStr("start") 
			time.sleep(0.01)
			for i in range(len(confirm_box)):
			#box在一定范围内才发送，因为每次只能抓一格的东西	
				send(confirm_box[i][0],confirm_box[i][1])

			sendStr("over")
		
	except:
		return 


# Get_True_box([ [366, 161],[263, 350],[479, 350]])
#[160, 300], [320, 300],[320, 170], 
#[366, 162],[263, 161]
# sendStr("None") 