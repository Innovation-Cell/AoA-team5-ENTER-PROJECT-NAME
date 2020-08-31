#!/usr/bin/env python
from pylab import *
import numpy as np
from matplotlib import pyplot as plt
import rospy
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from math import atan2
import math
import time
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

pixel =64
#global alphabets 
alphabets = ['P', 'S', 'L', 'U', 'B', 'O', 'G', 'N', 'C', 'Y', 'K', 'C', 'M', 'F', 'I', 'H', 'A', 'T']

global c
global out

rospy.init_node('umic_bot',anonymous=True)
#pub=rospy.Publisher('/mybot/mobile_base_controller/cmd_vel',Twist,queue_size=10)
#pub2=rospy.Publisher('/mybot/gripper_extension_controller/command',Float64,queue_size=10)


c=Float64()
out=Twist()

global PI
PI = 3.1415926535897

x = 0.0
y = 0.0
#z = 0.0
theta = 0.0
global kp
kp = 0.5
t=0
v=0


offset=0.0
x=0.0

def callback(data):
    global x
    if data.position[0]!=0:
        x=data.position[0]
    #print(x)


def newOdom(msg):
    global x
    global y
    #global z
    global theta
    global v         # v = angular speed (x)
    global t         # t = angular speed(z)

    x = msg.pose.pose.position.x  # pose.position.x
    y = msg.pose.pose.position.y  # pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])


    v = msg .twist.twist.linear.x
    t = msg .twist.twist.angular.z

def rotate(angle):
    global offset
    speed=Float64()
    sub1 = rospy.Subscriber("/catapult/joint_states", JointState, callback)
    pub3 = rospy.Publisher("/catapult/base_rotation_controller/command", Float64, queue_size=1)

    while True:
        inc_theta=angle-x+offset
        #print(inc_theta)
        if abs(inc_theta)>0.005:
            speed.data=0.3*inc_theta
            pub3.publish(speed)

        if abs(inc_theta)<=0.005:
            speed.data=0
            pub3.publish(speed)
            break
    sub1.unregister()

def distance(x1,y1,x2,y2):
	d=math.sqrt(((x2-x1)**2)+((y2-y1)**2))

	return d

def angle_plan(px,py):
    inc_x=px+4.247
    inc_y=py

    angle=math.atan2(-inc_y,-inc_x)

    rotate(angle)

def throwing_plan(px,py):
    v=Float64()
    pub2 = rospy.Publisher("/catapult/throwing_controller/command", Float64, queue_size=1)
    d=distance(-4.247,0,px,py)
    ang_vel=(d+9)/3.4
    v.data=ang_vel


        

    pub2.publish(v)

    rospy.sleep(0.5)

    v.data=-1

    pub2.publish(v)

    rospy.sleep(1)

    v.data=0

    pub2.publish(v)





def path_plan(px, py):

    
    sub = rospy.Subscriber("/mybot/odom", Odometry, newOdom)
    pub = rospy.Publisher("/mybot/mobile_base_controller/cmd_vel", Twist, queue_size=1)
    

    speed = Twist()

    r = rospy.Rate(10)
    
    goal = Point()
    goal.x = px
    goal.y = py

    while not rospy.is_shutdown():

        inc_x = goal.x - x
        inc_y = goal.y - y

        print("goal_x={}  goal_y:{}".format(goal.x,goal.y))
        print("current_x={}  current_y:{}".format(x,y))
        print("inc_x={}  inc_y:{}".format(inc_x,inc_y))

        

        angle_to_goal = atan2(inc_y, inc_x)
        print("Target_angle={}  Current_theta:{}".format(angle_to_goal,theta))
        print(abs(angle_to_goal - theta))

        if abs(angle_to_goal - theta) > 0.1:      #0.01
            
            print("1")
            speed.linear.x = 0.0
            #speed.angular.z = 0.3
            speed.angular.z =((angle_to_goal)- theta)*0.3
            print("linearspeed={}  angularspeed:{}".format(v ,t))
            '''
            if abs(inc_x) <0.05 and abs(inc_y) <0.05 :
                print("2")
                print("linearspeed={}  angularspeed:{}".format(speed.linear.x ,speed.angular.z))
                break

            '''
            if abs(inc_x )<0.05 and abs(inc_y) <0.05 :
                print("2")
                speed.linear.x = 0.0
                speed.angular.z = 0.0 
                print("v={}  w:{}".format(v ,t))
                print("linearspeed1={}  angularspeed1:{}".format(speed.linear.x ,speed.angular.z))
                pub.publish(speed)
                print("v={}  w:{}".format(v ,t))
                print("linearspeed2={}  angularspeed2:{}".format(speed.linear.x ,speed.angular.z))
                r.sleep()
                print("v={}  w:{}".format(v ,t))
                print("linearspeed3={}  angularspeed3:{}".format(speed.linear.x ,speed.angular.z))
                break
            
              
        else:
            
            
            
            '''
            if inc_x >0.05 and inc_y >0.05 :

                print("3")
                speed.linear.x = 0.2
                speed.angular.z = 0.0
                pub.publish(speed)
                r.sleep()
                

            else :
                print("4")
                speed.linear.x = 0.0
                speed.angular.z = 0.0 
                pub.publish(speed)
                r.sleep()
                break
            
            '''

            print("3")
            speed.linear.x = 0.2
            speed.angular.z = 0.0


            print("v={}  w:{}".format(v ,t))
            print("linearspeed4={}  angularspeed4:{}".format(speed.linear.x ,speed.angular.z))
            '''
            if inc_x <0.01 and inc_y <0.01 :
                print("4")
                print("linearspeed={}  angularspeed:{}".format(speed.linear.x ,speed.angular.z))
                break
             '''   
            
            if abs(inc_x) <0.05 and abs(inc_y) <0.05 :
                print("4")
                speed.linear.x = 0.0
                speed.angular.z = 0.0 

                print("v={}  w:{}".format(v ,t))
                print("linearspeed5={}  angularspeed5:{}".format(speed.linear.x ,speed.angular.z))
                pub.publish(speed)
                r.sleep()
                break
                

        pub.publish(speed)
        
        print("confirm")
        r.sleep()
        print("not stopping") 

def reverse_path_plan(px, py):

    
    sub = rospy.Subscriber("/mybot/odom", Odometry, newOdom)
    pub = rospy.Publisher("/mybot/mobile_base_controller/cmd_vel", Twist, queue_size=1)


    

    speed = Twist()

    r = rospy.Rate(10)
    
    goal = Point()
    goal.x = px
    goal.y = py

    while not rospy.is_shutdown():

        inc_x = goal.x - x
        inc_y = goal.y - y

        print("goal_x={}  goal_y:{}".format(goal.x,goal.y))
        print("current_x={}  current_y:{}".format(x,y))
        print("inc_x={}  inc_y:{}".format(inc_x,inc_y))

        

        angle_to_goal = atan2(-inc_y, -inc_x)
        print("Target_angle={}  Current_theta:{}".format(angle_to_goal,theta))
        print(abs(angle_to_goal - theta))

        if abs(angle_to_goal - theta) > 0.1:      #0.01
            
            print("1")
            speed.linear.x = 0.0
            #speed.angular.z = 0.3
            speed.angular.z =((angle_to_goal)- theta)*0.3
            print("linearspeed={}  angularspeed:{}".format(v ,t))
            '''
            if abs(inc_x) <0.05 and abs(inc_y) <0.05 :
                print("2")
                print("linearspeed={}  angularspeed:{}".format(speed.linear.x ,speed.angular.z))
                break

            '''
            if abs(inc_x )<0.05 and abs(inc_y) <0.05 :
                print("2")
                speed.linear.x = 0.0
                speed.angular.z = 0.0 
                print("v={}  w:{}".format(v ,t))
                print("linearspeed1={}  angularspeed1:{}".format(speed.linear.x ,speed.angular.z))
                pub.publish(speed)
                print("v={}  w:{}".format(v ,t))
                print("linearspeed2={}  angularspeed2:{}".format(speed.linear.x ,speed.angular.z))
                r.sleep()
                print("v={}  w:{}".format(v ,t))
                print("linearspeed3={}  angularspeed3:{}".format(speed.linear.x ,speed.angular.z))
                break
            
              
        else:
            
            
            
            '''
            if inc_x >0.05 and inc_y >0.05 :

                print("3")
                speed.linear.x = 0.2
                speed.angular.z = 0.0
                pub.publish(speed)
                r.sleep()
                

            else :
                print("4")
                speed.linear.x = 0.0
                speed.angular.z = 0.0 
                pub.publish(speed)
                r.sleep()
                break
            
            '''

            print("3")
            speed.linear.x = -0.3
            speed.angular.z = 0.0
            pub.publish(speed)


            print("v={}  w:{}".format(v ,t))
            print("linearspeed4={}  angularspeed4:{}".format(speed.linear.x ,speed.angular.z))
            '''
            if inc_x <0.01 and inc_y <0.01 :
                print("4")
                print("linearspeed={}  angularspeed:{}".format(speed.linear.x ,speed.angular.z))
                break
             '''   
            
            if abs(inc_x) <0.05 and abs(inc_y) <0.05 :
                print("4")
                speed.linear.x = 0.0
                speed.angular.z = 0.0 

                print("v={}  w:{}".format(v ,t))
                print("linearspeed5={}  angularspeed5:{}".format(speed.linear.x ,speed.angular.z))
                pub.publish(speed)
                r.sleep()
                break
                

        pub.publish(speed)
        
        print("confirm")
        r.sleep()
        print("not stopping") 

def orient_along(px, py):


    #rospy.init_node("speed_controller")

    #sub = rospy.Subscriber("/mybot/mobile_base_controller/odom", Odometry, newOdom)
    sub = rospy.Subscriber("/mybot/odom", Odometry, newOdom)
    pub = rospy.Publisher("/mybot/mobile_base_controller/cmd_vel", Twist, queue_size=1)

    speed = Twist()

    goal = Point()
    ##r = rospy.Rate(1000)
    goal.x = px
    goal.y = py
    #print(goal.x)
    #print(goal.y)
    while True:

        inc_x = goal.x - x
        inc_y = goal.y - y
        #print(x,y)
        #print(inc_x,inc_y)
        
        
        angle_to_goal = atan2(inc_y, inc_x)
        #angle_to_goal_rad = angle_to_goal * (math.pi/180)
        #print(angle_to_goal)
        #print(theta)
        #print(abs(angle_to_goal - theta))
        if abs(angle_to_goal - theta) > 0.01:
            

            #speed.angular.z =0.9
            speed.angular.z =((angle_to_goal)- theta)*0.3
            
        
        else:
        #    speed.linear.x = 0.0
            speed.angular.z = 0.0
            #pub.publish(speed)
            #r.sleep()
            break
            
                
        pub.publish(speed)
        print("Target={}  Current:{}".format(angle_to_goal,theta))
        r.sleep()
        print("not stopping")   

def point_decide(bx,by,tx,ty):
	angle_to_goal = atan2(ty-by,tx-bx)
	xf=bx-0.28*cos(angle_to_goal)
	yf=by-0.28*sin(angle_to_goal)
	return (xf,yf)

def decide_piston_speed(bx,by,tx,ty):
	d=distance(bx,by,tx,ty)

	v= math.sqrt(55*d/4)

	return v


            

def publisher():
    global r
    r = rospy.Rate(100)
    c.data=0
    sub = rospy.Subscriber("/mybot/odom", Odometry, newOdom)
    pub = rospy.Publisher("/mybot/mobile_base_controller/cmd_vel", Twist, queue_size=1)

    pub3 = rospy.Publisher("/catapult/base_rotation_controller/command", Float64, queue_size=1)
    pub2 = rospy.Publisher("/catapult/throwing_controller/command", Float64, queue_size=1)



    out.linear.x = 0
    out.linear.y = 0
    out.linear.z = 0
    out.angular.x = 0
    out.angular.y = 0
    out.angular.z = 0
    t0=0
    t1=0


    while not rospy.is_shutdown():

        #path_plan(point_decide(0.5,1,2,5)[0],point_decide(0.5,1,2,5)[1])
        #orient_along(2,5)
        #speed=decide_piston_speed(0.5,1,2,5)
        offset=0
        path_plan(0.5,1)
        path_plan(1.5,1)
        path_plan(1.5,0.6)
        orient_along(5,0.5)

        reverse_path_plan(0,0)
        orient_along(5,0)

        t0=time.time()
        t1=time.time()
        while (t1-t0)<9.35:
        	out.linear.x=-0.3
        	pub.publish(out)
        	t1=time.time()
        out.linear.x=0
        pub.publish(out)

        path_plan(0,0)
        orient_along(1,0)

        angle_plan(-7,7)
        rospy.sleep(0.5)
        throwing_plan(-7,7)
        offset=offset+0.080155
        angle_plan(-5,0)




        path_plan(0.5,0.5)
        path_plan(1.5,0.5)
        path_plan(1.5,0)
        orient_along(5,0)

        t0=time.time()
        t1=time.time()
        while (t1-t0)<12:
        	out.linear.x=-0.3
        	pub.publish(out)
        	t1=time.time()
        out.linear.x=0
        pub.publish(out)

        path_plan(0,0)
        orient_along(1,0)

        angle_plan(-5,-5)
        rospy.sleep(0.5)
        throwing_plan(-5,-5)
        offset=offset+0.080155
        angle_plan(-5,0)

        path_plan(0.5,-1)
        path_plan(1.5,-1)
        path_plan(1.5,-0.6)
        orient_along(5,-0.5)

        reverse_path_plan(0,0)
        orient_along(5,0)

        t0=time.time()
        t1=time.time()
        while (t1-t0)<9.35:
        	out.linear.x=-0.3
        	pub.publish(out)
        	t1=time.time()
        out.linear.x=0
        pub.publish(out)

        path_plan(0,0)
        orient_along(1,0)

        angle_plan(3,0)
        rospy.sleep(0.5)
        throwing_plan(3,0)
        offset=offset+0.080155
        angle_plan(-5,0)



        print('done')
        break


if __name__=='__main__':
	try:
		publisher()

		
	except rospy.ROSInterruptException:
		pass      
