"""
Python Script for Studying the Robot Manipulator details
"""

import matplotlib.pyplot as plt
import numpy as np
import math
import roboticstoolbox as rtb
from  roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import logging
import matplotlib.pyplot as plt

logging.basicConfig(level=logging.INFO)

class RoboArm():

    def __init__(self):
        ## Assigning DH Parameters for scara Robot
        self.robot = DHRobot([
                    RevoluteDH(a=0,alpha=0,d=0),
                    PrismaticDH(a=0,alpha=-math.pi/2,offset=12),], name="RoboArm")
                
        logging.info("\n######\nDH Parameter for Scara 2DOF Robot :")
        logging.info(f"{self.robot}")

    def plot(self,qs, dt, gif_filename):
        axes = np.array([-14,14,-14,14,-14,14])
        self.robot.plot(qs,dt=dt, backend='pyplot', eeframe=True,jointaxes=True,limits=axes, movie=gif_filename)
arm = RoboArm()
ang = np.linspace(0,2*3.14,num=50).reshape(-1,1)
rng = np.ones_like(ang)
Qs = np.hstack((ang,rng))

###
a2 = 12 # in # first link length
D = 10 # in # max end effector pose
x1 = (a2+D)*np.cos(ang).reshape(-1,)
y1 = (a2+D)*np.sin(ang).reshape(-1,)
plt.title("Robot Arm Workspace")
plt.xlabel("Robot x axis (in)")
plt.ylabel("Robot y axis (in)")
plt.plot(x1,y1,'r',label="max")

# min end effector pose 0 in
x2 = (a2)*np.cos(ang).reshape(-1,)
y2 = (a2)*np.sin(ang).reshape(-1,)
plt.plot(x2,y2,'b',label="min")
plt.fill_between(x1,y1,alpha=0.2,color="green")
plt.fill_between(x2,y2,alpha=0.5,color="white")
plt.legend()
plt.show()
