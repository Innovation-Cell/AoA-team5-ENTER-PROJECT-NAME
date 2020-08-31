#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import time
import math
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry



c=Float64()
out=Twist()




def publisher():
	rospy.init_node('umic_bot',anonymous=True)
	pub=rospy.Publisher('/mybot/mobile_base_controller/cmd_vel',Twist,queue_size=10)
	pub2=rospy.Publisher('/mybot/gripper_extension_controller/command',Float64,queue_size=10)


	c.data=0



	out.linear.x = 0
	out.linear.y = 0
	out.linear.z = 0
	out.angular.x = 0
	out.angular.y = 0
	out.angular.z = 0
	t0=0
	t1=0


	while not rospy.is_shutdown():



		t0=time.time()
		t1=time.time()
		out.linear.x = 0.3
		while (t1-t0)*0.3<=0.05:
			print((t1-t0)*0.3)
			pub2.publish(c)
			pub.publish(out)
			print('publishing1')
			t1=time.time()
		out.linear.x=0
		pub.publish(out)


		out.angular.z=-0.05
		t0=time.time()
		t1=time.time()
		while (t1-t0)*0.05<=1.59079632697:
			print((t1-t0)*0.3)
			pub2.publish(c)
			pub.publish(out)
			print('publishing2')
			t1=time.time()
		out.angular.z=0
		pub.publish(out)

		t0=time.time()
		t1=time.time()
		out.linear.x = 0.1
		while (t1-t0)*0.1<=0.1:
			print((t1-t0)*0.1)
			pub2.publish(c)
			pub.publish(out)
			print('publishing1')
			t1=time.time()
		out.linear.x=0
		pub.publish(out)

		
		rospy.sleep(2)
		t0=time.time()
		t1=time.time()
		c.data = 5
		while t1-t0<=5:
			pub2.publish(c)
			print('publishing3')
			t1=time.time()
		c.data=-5
		pub2.publish(c)

		t0=time.time()
		t1=time.time()
		out.linear.x = 0.3
		while (t1-t0)*0.3<=0.5:
			print((t1-t0)*0.3)
			pub2.publish(c)
			pub.publish(out)
			print('publishing1')
			t1=time.time()
		out.linear.x=0
		pub.publish(out)

		rospy.sleep(2)
		t0=time.time()
		t1=time.time()
		c.data = 5
		while t1-t0<=5:
			pub2.publish(c)
			print('publishing3')
			t1=time.time()
		c.data=-5
		pub2.publish(c)

		t0=time.time()
		t1=time.time()
		out.linear.x = -0.3
		while (t1-t0)*0.3<=1:
			print((t1-t0)*0.3)
			pub2.publish(c)
			pub.publish(out)
			print('publishing1')
			t1=time.time()
		out.linear.x=0
		pub.publish(out)

		rospy.sleep(2)
		t0=time.time()
		t1=time.time()
		c.data = 5
		while t1-t0<=5:
			pub2.publish(c)
			print('publishing3')
			t1=time.time()
		c.data=-5
		pub2.publish(c)



		print('done')
		break



if __name__=='__main__':
	try:
		publisher()

		
	except rospy.ROSInterruptException:
		pass