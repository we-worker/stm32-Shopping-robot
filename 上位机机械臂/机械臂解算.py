from math import *

L1=120
L2=36
L3=118
L4=33
L5=170
L6=22


def ArmSolution(x,y):

	#求解角3
	t1=2*L5*y+2*L6*x;
	t2=2*L5*x-2*L6*y;

	tsin=1.0*(x*x+y*y+L5*L5+L6*L6-L1*L1)/(sqrt(t1*t1+t2*t2));

	if(tsin>1 or tsin <-1):
		return [pi/2,0,0,0]
	if(y==0):
		return [pi/2,0,0,0]
	o3=3.1415926-asin(tsin)-atan(t2/t1);
	if(o3<1.57):
		o3+=3.1415926
	#求解角1
	tsin=1.0*(x*x+y*y+L1*L1-L5*L5)/(2*L1*sqrt(x*x+y*y));
	if(tsin>1 or tsin <-1):
		return [pi/2,0,0,0]
	o1=asin(tsin)-atan(x/y);
	#求解角2
	t1=L1*cos(o1)-L4*cos(o3);
	t2=L1*sin(o1)-L4*sin(o3);

	if(t1*t1+t2*t2<=0):
		print("除数为零")
		return [pi/2,0,0,0]

	tsin=1.0*(t1*t1+t2*t2+L2*L2-L3*L3)/(2*L2*sqrt(t1*t1+t2*t2));

	if(tsin>1 or tsin <-1):
		return [pi/2,0,0,0]

	o2=asin(tsin)-atan(t1/t2);
	# if(o2>o1):
	# 	o2=3.1415926-asin(tsin)-atan(t1/t2)

	o4=o3-3.14159;
	print("角1：{:.2f}	角2：{:.2f}	角3：{:.2f}	角4：{:.2f}".format(o1/6.28*360,o2/6.28*360,o3/6.28*360,o4/6.28*360));
	# print("角1：{:.2f}	角2：{:.2f}	角3：{:.2f}	角4：{:.2f}".format(o1,o2,o3,o4));
	return o1,o2,o3,o4


def Armsolution2(x,y):
	x1=0 
	y1=0

	A=sqrt((L5-L4)*(L5-L4)+L6*L6);
	B=sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
	try:
		o4=acos((B*B+L1*L1-A*A)/2/B/L1);
	except:
		print("error")

	if(x-x1==0):
		o5=-pi/2
	else:
		o5=atan((y-y1)/(x-x1));

	o1=pi-o4+o5;
	print("o1=",o1,o1*360/2/pi)
	o4=acos((B*B-L1*L1+A*A)/2/B/A);
	o6=acos(((L5-L4)*(L5-L4)+A*A-L6*L6)/2/A/(L5-L4))

	o5=pi/2+o5;
	o3=pi-o4-o5-o6;

	print("o3=",o3,o3*360/2/pi)



	a=x+L5*sin(o3)+L6*cos(o3)-x1;
	b=y+L5*cos(o3)-L6*sin(o3)-y1;
	t=1.0*(a*a+b*b-L3*L3+L2*L2)/2/L2/sqrt(a*a+b*b);


	fi=atan(-1.0*b/a);
	o2=asin(t)-fi;
	if(o2<0):
		o2+=pi;


	print("o2=",o2,o2*360/2/3.14159);

	o4=pi/2-o3;
	print("角1：{:.2f}	角2：{:.2f}	角3：{:.2f}	角4：{:.2f}".format(o1/6.28*360,(o2-3.14159/2)/6.28*360,(3/2*3.14159-o3)/6.28*360,o4/6.28*360));
	return o1,o2-3.14159/2,2/3*3.14159-o3,0


# ArmSolution(-100,100)
