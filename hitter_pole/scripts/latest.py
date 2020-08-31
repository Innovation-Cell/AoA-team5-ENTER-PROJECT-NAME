#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math


offset=0.0
x=0.0

def callback(data):
	global x
	if data.position[0]!=0:
		x=data.position[0]
	print(x)


def rotate(angle):
	global offset
	speed=Float64()
	sub = rospy.Subscriber("/catapult/joint_states", JointState, callback)
	pub = rospy.Publisher("/catapult/base_rotation_controller/command", Float64, queue_size=1)




	while True:
		inc_theta=angle-x+offset
		#print(inc_theta)
		if abs(inc_theta)>0.01:
			speed.data=0.3*inc_theta
			pub.publish(speed)

		if abs(inc_theta)<0.01:
			speed.data=0
			pub.publish(speed)
			break


def angle_plan(px,py):
	inc_x=px+4.247
	inc_y=py

	angle=math.atan2(-inc_y,-inc_x)

	rotate(angle)

def distance(x1,y1,x2,y2):
	d=math.sqrt(((x2-x1)**2)+((y2-y1)**2))

	return d

def throwing_plan(px,py):
	v=Float64()
	pub2 = rospy.Publisher("/catapult/throwing_controller/command", Float64, queue_size=1)
	d=distance(-4.247,0,px,py)
	ang_vel=(d+9)/3.4

	v.data=ang_vel

	pub2.publish(v)

	rospy.sleep(1)

	v.data=-1

	pub2.publish(v)

	rospy.sleep(2)

	v.data=0

	pub2.publish(v)



def publisher():
	rospy.init_node('umic_bot1',anonymous=True)
	pub = rospy.Publisher("/catapult/base_rotation_controller/command", Float64, queue_size=1)
	pub2 = rospy.Publisher("/catapult/throwing_controller/command", Float64, queue_size=1)
	sub = rospy.Subscriber("/catapult/joint_states", JointState, callback)
	r=rospy.Rate(100)


	while not rospy.is_shutdown():
		#v.data=-1
		#pub2.publish(v)
		#v.data=0
		#pub2.publish(v)
		offset=0
		angle_plan(-7,7)
		throwing_plan(-7,7)
		offset=offset+0.080155
		angle_plan(-5,0)

		angle_plan(-5,-5)
		throwing_plan(-5,-5)
		offset=offset+0.080155

		angle_plan(-5,0)


		break

	

	
	



if __name__=='__main__':
	try:
		publisher()

		
	except rospy.ROSInterruptException:
		pass 

