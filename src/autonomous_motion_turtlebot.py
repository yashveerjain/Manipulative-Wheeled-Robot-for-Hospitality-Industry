#!/usr/bin/python3

"""
Operate the Robot fully autonomously
"""

import rospy
import numpy as np
import math

from std_msgs.msg import Float64, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import cv2

import sys, select, termios, tty
import time
from utils.rrt_star import RRTStarGraph, Map



class Robot():

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

    def __init__(self, robo_cmd="/robo9", start_pos=(0,1,0),clearance=5,map_dim=(200,500) ):
        # self.speed = 0.3 # 1m/s
        # self.turn = 0.3 # 1rad/sec

        # self.pub_manipulator = rospy.Publisher(f'{robo_cmd}/first_link_controller/command', Float64, queue_size=10)
        # self.pub_move = rospy.Publisher(f'{robo_cmd}/cmd_vel', Twist, queue_size=10)
        # self.pub_tray_diff = rospy.Publisher(f'{robo_cmd}/tray_diffusor_joint_controller/command', Float64, queue_size=10)
        # self.pub_end_effector = rospy.Publisher(f'{robo_cmd}/end_effector_controller/command', Float64, queue_size=10)
        self.pub_move = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.odom_callback)

        self.odom_output = None
        # rospy.Subscriber("robo9/odom", Odometry, self.odom_callback)

        self.restraunt_map = Map(start_pos,clearance=clearance, map_dim=map_dim)
        self.start_pos = self.restraunt_map.process_pos2cm(start_pos)
        print("start pos", self.start_pos)
        self.planner = RRTStarGraph(start=self.start_pos, mapDimension=map_dim, map = self.restraunt_map.map)
        self.dist_thresh = 10/100 # for point selection in m

        self.start=True
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.kp = 1
        self.record = {}

        self.vel_x = 0.1 # 0.1 m/s
        

    def getKey(self):
        
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def odom_callback(self,odom_data):
        curr_time = odom_data.header.stamp
        self.odom_output  = odom_data.pose.pose
        # print(self.odom_output)

    def gen_arm_move_func(self, publisher, initial, final, num=50):
        
        steps = np.linspace(initial, final, num)
        print(steps)
        for i in steps:
            publisher.publish(i)
            time.sleep(0.1)
    
    def get_desired_steering(self, x, y):
        theta = np.arctan2((y),(x))# between -pi to pi
        # theta = np.arctan(y/x)# between -pi to pi
        # -pi/4 means 4rd quadrant and pi/4 1st quadrant

        #clip theta to turtle bot max possible angle rotation
        # theta = max(min(theta,2),-2) # limiting to max possible roation velocity 2.84rad/sec
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
    
    def distance(self, pos1, pos2):
        x1, y1 = (pos1[0], pos1[1])
        x2, y2 = (pos2[0], pos2[1])

        dist=  math.sqrt(math.pow(x1-x2,2)+math.pow(y1-y2,2))
        return dist
    def move(self,key):
        # move_msg = Twist()
        # count = 0
        # try:
        rate = rospy.Rate(30)
        if True:
            # while self.odom_output is None:
            #     continue
            if key in self.restraunt_map.tablePoses.keys():
                # self.start = False
                # print(dir(self.restraunt_map))
                TPose = self.restraunt_map.tablePoses[key]
                # T_trans = TPose
                TPosex,TPosey,Tang = TPose
                
                if TPose:
                    print(TPose)
                    # TPose, direction = self.TPose
                    # TPosex,TPosey = TPose
                    self.restraunt_map.draw_map(self.start_pos, TPose)
                    ret = self.planner.search(TPose)
                    print(ret)
                    if ret: 
                        (path, smooth_path, short_path), RW_paths = self.planner.getPathCoords()
                        rw_short_path = RW_paths[1]
                        rw_smooth_path = RW_paths[2]
                        print(short_path)
                        print(smooth_path)
                        self.restraunt_map.draw_path(path, smooth_path)
                        rw_back2start = list(reversed(rw_short_path))
                        # rw_back2start = list(reversed(rw_smooth_path))
                        for path_coord in rw_smooth_path:
                            # convet
                            newx,newy = path_coord
                            (newx,newy,_) = self.restraunt_map.process_pos2m((newx,newy,0))
                            # print(newx,newy)
                            currx, curry = (self.odom_output.position.x,self.odom_output.position.y)
                            print(f"dist X : {newx-self.odom_output.position.x}")
                            print(f"dist Y : {newy-self.odom_output.position.y}")
                            print(newx,newy," | ",currx, curry)
                            # # if not(abs(TPosex-self.odom_output.position.x)<self.dist_thresh and abs(TPosex-self.odom_output.position.x)>-self.dist_thresh):
                            dist = self.distance((newx, newy),(currx, curry))
                            while dist >self.dist_thresh:
                                currx, curry = (self.odom_output.position.x,self.odom_output.position.y)
                                dist = self.distance((newx,newy),(currx,curry))
                                print(f"dist : {dist} | smooth_path : ",path_coord)
                                # print(f"dist Y : {newy-self.odom_output.position.y}")
                            #     # in x axis y = 0 
                                delta_x = newx-currx
                                delta_y = newy-curry
                                print(delta_x, " | " ,delta_y)
                                # minus desired theta because map computation is in reverse order
                                desired_theta = self.get_desired_steering(delta_x,delta_y)
                                # if desired_theta
                            #     print(desired_theta)
                            #     print(theta)
                                curr_rotation = self.get_rotation(self.odom_output.orientation)
                                print(desired_theta," | ", curr_rotation)
                                msg = Twist()
                                msg.linear.x = self.vel_x
                                msg.angular.z = self.kp*(desired_theta-curr_rotation)
                                
                                self.pub_move.publish(msg)
                                rate.sleep()
                        
                        # facing the table  
                        desired_theta = 0
                        curr_rotation = self.get_rotation(self.odom_output.orientation)
                        err = (desired_theta-curr_rotation)
                        while abs(err)>0.01:
                            msg = Twist()
                            # msg.linear.x = self.vel_x
                            msg.angular.z = self.kp*(desired_theta-curr_rotation)
                            curr_rotation = self.get_rotation(self.odom_output.orientation)
                            err = (desired_theta-curr_rotation)
                            self.pub_move.publish(msg)
                            rate.sleep()

                        # turn back from the table towards kitchen 
                        # desired_theta = 3.14
                        # curr_rotation = self.get_rotation(self.odom_output.orientation)
                        # err = (desired_theta-curr_rotation)
                        # while abs(err)>0.01:
                        #     msg = Twist()
                        #     # msg.linear.x = self.vel_x
                        #     ang_vel = self.kp*err
                        #     ang_vel = max(min(ang_vel,2),-2) # limit speed to 2 rad/sec
                        #     msg.angular.z = ang_vel        
                        #     curr_rotation = self.get_rotation(self.odom_output.orientation)
                        #     err = (desired_theta-curr_rotation)
                        #     self.pub_move.publish(msg)
                        #     rate.sleep()


                        for path_coord in rw_back2start[1:]:
                            # convet
                            newx,newy = path_coord
                            (newx,newy,_) = self.restraunt_map.process_pos2m((newx,newy,0))
                            # print(newx,newy)
                            currx, curry = (self.odom_output.position.x,self.odom_output.position.y)
                            print(f"dist X : {newx-self.odom_output.position.x}")
                            print(f"dist Y : {newy-self.odom_output.position.y}")
                            print(newx,newy," | ",currx, curry)
                            # # if not(abs(TPosex-self.odom_output.position.x)<self.dist_thresh and abs(TPosex-self.odom_output.position.x)>-self.dist_thresh):
                            dist = self.distance((newx,newy),(currx,curry))
                            while dist > self.dist_thresh:
                                currx, curry = (self.odom_output.position.x, self.odom_output.position.y)
                                dist = self.distance((newx,newy),(currx,curry))
                                print(f"dist : {dist} | smooth_path : ",path_coord)
                                # print(f"dist Y : {newy-self.odom_output.position.y}")
                            #     # in x axis y = 0 
                                delta_x = newx - currx
                                delta_y = newy - curry
                                # print(delta_x | delta_y)

                                # minus desired theta because map computation is in reverse order
                                desired_theta = self.get_desired_steering(delta_x,delta_y)
                            #     print(desired_theta)
                            #     print(theta)
                                curr_rotation = self.get_rotation(self.odom_output.orientation)
                                print(desired_theta," | ", curr_rotation)
                                msg = Twist()
                                msg.linear.x = self.vel_x
                                ang_vel = self.kp*(desired_theta-curr_rotation)
                                # ang_vel = max(min(ang_vel,1),-1) # limit speed to 2 rad/sec
                                msg.angular.z = ang_vel                                
                                self.pub_move.publish(msg)
                                rate.sleep()
                                
        # except Exception as e:
        #     print("error : ", e)

    def run(self):
    
        while(1):
            key = self.getKey()
            inp = self.move(key)
            if (key == '\x03'):
                break
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__=="__main__":
    rospy.init_node('boy')
    robot  = Robot()    
    robot.run()
    