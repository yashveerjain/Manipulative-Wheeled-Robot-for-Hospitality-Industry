#!/usr/bin/python3

import rospy
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
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

moveBindings = {
        'i':(.3,0),
        'o':(.3,-.3),
        'j':(0,.3),
        'l':(0,-.3),
        'u':(.3,.3),
        ',':(-.3,0),
        '.':(-.3,.3),
        'm':(-.3,-.3),
           }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 1
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def move_plate(publisher, initial, final, num=50):
    # if initial>final:
    #     step-=step
    steps = np.linspace(initial, final, num)
    print(steps)
    for i in steps:
        publisher.publish(i)
        time.sleep(0.1)
def get_desired_steering(initial_pose, goal_pose, time=3):
    x_o,y_o = initial_pose
    x_g,y_g = goal_pose
    theta = np.arctan2((y_g-y_o),(x_g-x_o)) # between -pi to pi

    # meter
    wheel_base = 0.76
    wheel_radius = 0.1016

    omega = (theta*wheel_base/2)/(wheel_radius*time)
    return omega
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('my_teleop')

    # pub_right = rospy.Publisher('/rat_bot/front_right_axle_revolute_controller/command', Float64, queue_size=10) # Add your topic here between ''. Eg '/my_robot/steering_controller/command'
    # pub_left = rospy.Publisher('/rat_bot/front_left_axle_revolute_controller/command', Float64, queue_size=10)
    robo_cmd = "/robo9"
    pub_manipulator = rospy.Publisher(f'{robo_cmd}/first_link_controller/command', Float64, queue_size=10)
    pub_move = rospy.Publisher(f'{robo_cmd}/cmd_vel', Twist, queue_size=10)
    pub_tray_diff = rospy.Publisher(f'{robo_cmd}/tray_diffusor_joint_controller/command', Float64, queue_size=10)
    pub_end_effector = rospy.Publisher(f'{robo_cmd}/end_effector_controller/command', Float64, queue_size=10)
    
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
    move_msg = Twist()
    front_man_angle= 3*1.57
    left_side_man_angle = 2*3.14
    right_side_man_angle =  3.14
    base_pose_tray_diffusor = 4.6
    final_pose_tray_diffusor = 6.2
    initial_end_effector_pose = 0
    final_end_effector_pose = .15
    armBindings={
        'e':(pub_end_effector,initial_end_effector_pose,final_end_effector_pose),
        'E':(pub_end_effector,final_end_effector_pose,initial_end_effector_pose),
        't': (pub_tray_diff,base_pose_tray_diffusor,final_pose_tray_diffusor),
        'T': (pub_tray_diff,final_pose_tray_diffusor,base_pose_tray_diffusor),
        'a' : (pub_manipulator,front_man_angle,left_side_man_angle), # left side manipulator
        'd' : (pub_manipulator,front_man_angle,right_side_man_angle), # right side manipulator
        'A' : (pub_manipulator,left_side_man_angle,front_man_angle), # left side to front post manipulator
        'D' : (pub_manipulator,right_side_man_angle,front_man_angle), # right side to front manipulator
        }

    current_man_angle = front_man_angle
    current_end_effector_pose = initial_end_effector_pose
    current_tray_diffusor_pose = base_pose_tray_diffusor

    armBindingCalls = False
    try:
        print (msg)
        print(move_msg)
        # print (vels(speed,turn))

        while(1):
            key = getKey()
            # print(key)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                print("key",key)
                count = 0
            elif key in armBindings.keys():
                data = armBindings[key]
                move_msg.angular.z = 0
                move_msg.linear.x = 0
                pub_move.publish(move_msg)
                print("############")
                print("key: ",key)
                print("data : ",data)
                print("############")
                move_plate(*data)
                
                armBindingCalls = True
                count = 0
                if (status == 14):
                    print (msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                
                if (key == '\x03'):
                    break
            
            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            # control_turn=0.5
            # control_speed =5
            print("############")
            print("control speed : ",control_speed)
            print("control turn : ",control_turn)
            print("############") 
            move_msg.angular.z = control_turn
            move_msg.linear.x = control_speed
            # pub_right.publish(control_turn) # publish the turn command.
            # pub_left.publish(control_turn) # publish the turn command.
            pub_move.publish(move_msg)
            print("############")
            ini_pose = (0,0)
            final_pose = (1,1)
            print("omega : ",get_desired_steering(ini_pose,final_pose))
            if not armBindingCalls:
                pub_manipulator.publish(front_man_angle)
                pub_tray_diff.publish(base_pose_tray_diffusor)
                pub_end_effector.publish(initial_end_effector_pose)
            # pub_right_wheel_move.publish(control_speed)
            # pub_left_wheel_move.publish(control_speed)
            # rate.sleep()
            
            


    except Exception as e:
        print (e)

    finally:
        # pub_right.publish(control_turn)
        # pub_left.publish(control_turn)
        # pub_move_left_wheel.publish(control_speed) # publish the control speed. 
        # pub_move_right_wheel.publish(control_speed)
        pub_move.publish(move_msg)
        # twist = Twist()
        # twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        # pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
