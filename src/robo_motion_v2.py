#!/usr/bin/python3

import rospy
import numpy as np
import math

from std_msgs.msg import Float64, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import sys, select, termios, tty
import time

msg = """
Control Your Rat Bot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
CTRL-C to quit
"""

class Robot:
        # pub_left = rospy.Publisher('/rat_bot/front_left_axle_revolute_controller/command', Float64, queue_size=10)
    
     # Add your topic for move here '' Eg '/my_robot/longitudinal_controller/command'

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    # rate = rospy.Rate(100)
    
    front_man_angle= 3*1.57
    left_side_man_angle = 2*3.14
    right_side_man_angle =  3.14
    base_pose_tray_diffusor = 6
    final_pose_tray_diffusor = 6.2
    initial_end_effector_pose = 0
    final_end_effector_pose = .15
    
    current_man_angle = front_man_angle
    current_end_effector_pose = initial_end_effector_pose
    current_tray_diffusor_pose = base_pose_tray_diffusor

    moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }
    def __init__(self, robo_cmd="/robo9"):
        self.speed = 0.3 # 1m/s
        self.turn = 0.3 # 1rad/sec

        self.pub_manipulator = rospy.Publisher(f'{robo_cmd}/first_link_controller/command', Float64, queue_size=10)
        self.pub_move = rospy.Publisher(f'{robo_cmd}/cmd_vel', Twist, queue_size=10)
        self.pub_tray_diff = rospy.Publisher(f'{robo_cmd}/tray_diffusor_joint_controller/command', Float64, queue_size=10)
        self.pub_end_effector = rospy.Publisher(f'{robo_cmd}/end_effector_controller/command', Float64, queue_size=10)
        
        self.tablePose = self.table2pose()
        self.odom_output = None
        rospy.Subscriber("robo9/odom", Odometry, self.callback)
        
        self.startPose = (0,0)
        self.start=True
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.kp = 0.05
        self.record = {}
        

    def callback(self,odom_data):
        curr_time = odom_data.header.stamp
        self.odom_output  = odom_data.pose.pose
        
    def table2pose(self):
        return {
            "1" : ((-2,0),"right"),
            "2" : ((-5,0),"right"),
            "3" : ((-8,0),"right"),
            "4" : ((-8,0),"left"),
            "5" : ((-5,0),"left"),
            "6" : ((-2,0),"left"),            
        }
    def getKey(self):
        
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def vels(self,speed,turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

    def gen_arm_move_func(self, publisher, initial, final, num=50):
        
        steps = np.linspace(initial, final, num)
        print(steps)
        for i in steps:
            publisher.publish(i)
            time.sleep(0.1)
    
    def get_desired_steering(self, x, y):
        theta = np.arctan2((y),(x))# between -pi to pi
        # -pi/4 means 4rd quadrant and pi/4 1st quadrant
        return theta

    def get_rotation(self,ang):
        orientation_q = ang
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw

    def base_move(self, prev_pose, goal_pose, vel, theta, delta_t):
        prev_x,prev_y = prev_pose
        x_g,y_g = goal_pose
        new_x = prev_x + vel * math.cos(theta) * delta_t
        new_y = prev_y + vel * math.sin(theta) * delta_t
        if (abs(new_x - x_g)<0.1 or abs(new_x - x_g)>0.1) and (abs(new_y - y_g)<0.1 or abs(new_y - y_g)>0.1): 
            return None
        
        return (new_x,new_y)
        
    def move(self,key):
        move_msg = Twist()
        target_speed = self.speed
        target_turn = self.turn 
        count = 0
        try:
            # print (vels(speed,turn))
            # print(key)
            if key in self.tablePose.keys():
                self.start = False
                self.TPose = self.tablePose[key]
                TPose, direction = self.TPose
                TPosex,TPosey = TPose
                
            if self.TPose:
                TPose, direction = self.TPose
                TPosex,TPosey = TPose
                print(f"dist X : {TPosex-self.odom_output.position.x}")
                print(f"dist Y : {TPosey-self.odom_output.position.y}")
                if not(abs(TPosex-self.odom_output.position.x)<0.1 and abs(TPosex-self.odom_output.position.x)>-0.1):
                    # in x axis y = 0 
                    theta = self.get_desired_steering(TPosex-self.odom_output.position.x,0)
                    # print(theta)
                    curr_rotation = self.get_rotation(self.odom_output.orientation)
                    print(theta," | ", curr_rotation)
                    
                    self.rotate=True
                    
                    if abs(theta-curr_rotation)<0.1:
                        move_msg.linear.x = self.speed
                        print(f"X move : not({abs(TPosex-self.odom_output.position.x)}<0.1 or {abs(TPosex-self.odom_output.position.x)}>-0.1)")
                    else:
                        move_msg.angular.z = self.kp*(theta-curr_rotation)
                        print(f"X rotation : {TPosex}-{self.odom_output.position.x}<0 and {curr_rotation}")
                elif not(abs(TPosey-self.odom_output.position.y)>-0.2 and abs(TPosey-self.odom_output.position.y)<0.2):
                    theta = self.get_desired_steering(0,TPosey-self.odom_output.position.y)
                    print(theta)
                    theta_dist = self.translate_angle_to_odom_orient(theta)
                    print(theta_dist," | ", self.odom_output.orientation.z)
                    if not (abs(self.odom_output.orientation.z-theta_dist)<0.01):
                        # Right or left
                        move_msg.angular.z = self.turn
                        self.rotate=True
                        print(f"Y rotation right or left : {TPosey}-{self.odom_output.position.y}<0 and {abs(self.odom_output.orientation.z)}>-0.5")
                    # elif TPosey-self.odom_output.position.y>0 and self.odom_output.orientation.z<0.5:
                    #     # left
                    #     move_msg.angular.z = self.turn
                    #     self.rotate=True
                    #     print(f"Y rotation left : {TPosey}-{self.odom_output.position.y}>0 and {abs(self.odom_output.orientation.z)}<0.5")
                    else:
                        move_msg.linear.x = self.speed
                        print(f"Y move: not({abs(TPosey-self.odom_output.position.y)}>-0.1 or {abs(TPosey-self.odom_output.position.y)}<0.1)")
                else:
                    self.TPose=None
                
                print("TPose : ",self.TPose)


                if direction=="right":
                    man_end_pose = 3.14
                if direction=="left":
                    man_end_pose = 2*3.14

            # if not armBindingCalls:
        except Exception as e:
            print (e)

        finally:
            
            self.pub_move.publish(move_msg)
            print("############")
            
            if self.start:
                self.pub_manipulator.publish(self.front_man_angle)
                self.pub_tray_diff.publish(self.base_pose_tray_diffusor)
                self.pub_end_effector.publish(self.initial_end_effector_pose)
        return 

    def run(self):
        r = rospy.Rate(10)
        self.TPose = None
        print (msg)
        while not rospy.is_shutdown():
            rospy.Subscriber("robo9/odom", Odometry, self.callback)
            key = self.getKey()
            self.move(key)
            print("Subscriber : ",self.odom_output )
            print("\n##########\n")
            if (key == '\x03'):
                break
            # time.sleep(0.1)
            r.sleep()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
if __name__=="__main__":
    
    
    rospy.init_node('my_teleop')

    # pub_right = rospy.Publisher('/rat_bot/front_right_axle_revolute_controller/command', Float64, queue_size=10) # Add your topic here between ''. Eg '/my_robot/steering_controller/command'

    robot  = Robot()    
    robot.run()
    
    
