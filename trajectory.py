from robot import kinematics
from numpy import *
my_robot = kinematics(1,1,3,2,0,0)
START = 6,0,1
TARGET = 5,0,0.5
H0_5 =  [ #Orientation of the end effector
            [1,0,0,0],
            [0,0,-1,0],
            [0,1,0,0],
            [0,0,0,1]
        ]
th1,th2,th3 = my_robot.ikine(6,0,1)
H0_3 = my_robot.pos(th1,th2,th3)
H3_5 = dot(linalg.inv(H0_3),H0_5)
th4 = round(rad2deg(arcsin(H3_5[0,2])),2) #Theta4
th5 = round(rad2deg(arcsin(H3_5[2,1])),2) #Theta5
angles = [th1,th2,th3,th4,th5]
print(angles)