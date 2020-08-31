#!/usr/bin/env python
import rospy
from time import time
import time
##from hector_uav_msgs.msg import PoseActionGoal
##from geometry_msgs import PoseStamped

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from math import atan2
from std_msgs.msg import Float64
import math

global PI
PI = 3.1415926535897

x = 0.0
y = 0.0
#z = 0.0
theta = 0.0
global kp
kp = 0.5


def newOdom(msg):
    global x
    global y
    #global z
    global theta

    x = msg.pose.pose.position.x  # pose.position.x
    y = msg.pose.pose.position.y  # pose.position.y
    
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])




def path_plan(px, py):

    #rospy.init_node("speed_controller")

    #pub = rospy.Publisher("/cmd_vel/", Twist, queue_size=1)

    speed = Twist()

    goal = Point()
    ##r = rospy.Rate(1000)
    goal.x = px
    goal.y = py
    #print(goal.x,goal.y)
    print("goal_x={}  goal_y:{}".format(goal.x,goal.y))
    #print()
    while True:

        inc_x = goal.x - x
        inc_y = goal.y - y
        print("current_x={}  current_y:{}".format(x,y))
        #print(x,y)
        #print(inc_x,inc_y)
        

        angle_to_goal = atan2(inc_y, inc_x)

        if abs(angle_to_goal - theta) > 0.01: 
            

            print("1")
            if angle_to_goal - theta >0:
                speed.linear.x = 0.0
                speed.angular.z = 0.1
            else:
                speed.linear.x = 0.0
                speed.angular.z = -0.1     
            pub.publish(speed)            
            #speed.angular.z =((angle_to_goal)- theta)
            
            if abs(inc_x) <0.05 and abs(inc_y) <0.05 :
                print(theta)
                print(angle_to_goal)
                print(x)
                print(y)
                print(inc_x)
                print(inc_y)
                print("2")
                speed.linear.x = 0.0
                speed.angular.z = 0.0 
                pub.publish(speed)
                r.sleep()
                break

              
        else:
            
            
            

            if abs(inc_x) >0.05 and abs(inc_y) >0.05 :

                print("3")
                speed.linear.x = 0.1
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


        pub.publish(speed)
        
        print("Target={}  Current:{}".format(angle_to_goal,theta))
        r.sleep()
        print("not stopping")    

def orient_along(px, py):


    #rospy.init_node("speed_controller")

    #sub = rospy.Subscriber("/mybot/mobile_base_controller/odom", Odometry, newOdom)

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
        
        
        angle_to_goal = atan2(py, px)
        #angle_to_goal_rad = angle_to_goal * (math.pi/180)
        #print(angle_to_goal)
        #print(theta)
        #print(abs(angle_to_goal - theta))
        if abs(angle_to_goal - theta) > 0.01:
            

            #speed.angular.z =0.9
            speed.angular.z =((angle_to_goal)- theta)
            
        
        else:
        #    speed.linear.x = 0.0
            speed.angular.z = 0.0
            pub.publish(speed)
            r.sleep()
            break
            
                
        pub.publish(speed)
        print("Target={}  Current:{}".format(angle_to_goal,theta))
        r.sleep()
        print("not stopping")   



def gripper(c):

    #rospy.init_node("gripper")
    pub = rospy.Publisher("/mybot/gripper_extension_controller/command", Float64, queue_size=1)
    r = rospy.Rate(100000)
    print("gripper")
    grip = Float64()
    grip.data=c


    while not rospy.is_shutdown():
        #ta =rospy.get_rostime()
        #b =rospy.get_rostime()
        #zero_time = rospy.Time()

        if  (tb-ta)*5  <= (1/c) :
        
        #for i in range (1):
        #grip.data = [s , d,f]
        #r = rospy.Rate(100)
        #print("gripper1")
            pub.publish(grip)
	            #b = rospy.get_rostime()
            r.sleep()

#orient_along(0,0.5)

#orient_along(-5,0)
#gripper(0)
#gripper(-0.2)



#gripper(10)

#path_plan(2, 2)

#gripper(10)

#path_plan(0.5, -0.5)
'''
for i in range (1):

    path_plan(0.5, 0.5)
    for j  in range (1):

        orient_along(1,0.5)
'''

#r=rospy.Rate()
def main():

    rospy.init_node("speed_controller")
    global sub
    global pub
    global r

    #sub = rospy.Subscriber("/mybot/mobile_base_controller/odom", Odometry, newOdom)
    pub = rospy.Publisher("/mybot/mobile_base_controller/cmd_vel", Twist, queue_size=1)
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        sub = rospy.Subscriber("/mybot/mobile_base_controller/odom", Odometry, newOdom)
        #orient_along(0,1)
        #path_plan(0,0.5)
        #orient_along(1,1)
        path_plan(0.5,0.5)
        path_plan(0.5,-0.5)

main()

#path_plan(0.5, 0.5)

#r.sleep()
#orient_along(1,0.5)
#path_plan(0.5, 0)
#orient_along(1,0)
#path_plan(0.5, -0.5)
#orient_along(1,-0.5)
