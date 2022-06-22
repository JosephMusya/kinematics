from robot import kinematics
from numpy import *
import threading
import time
import os

class Move(threading.Thread):
    def __init__(self,name,qo,qf,tf,spacing,servo_pin):
        threading.Thread.__init__(self)
        self.name = name
        self.qo = qo
        self.qf = qf
        self.tf = tf
        self.spacing = spacing
        self.servo_pin = servo_pin
        
    def write_angle(self,q):
        pwm = q + 60
        cmd = "echo "+str(self.servo_pin)+"="+str(pwm)+ "> /dev/servoblaster"
        # os.system(cmd)
        
    def run(self):
        qdd_p = 4*((self.qf-self.qo))/pow(self.tf,2)
        x = round((pow(qdd_p,2)*pow(self.tf,2)),3)
        y = round((4*qdd_p*(self.qf-self.qo)),3)
        num = round(sqrt(x-y),3)
        tb = (0.5*self.tf) - (num)/(2*qdd_p)
        qb_p = self.qo + (0.5*qdd_p*pow(tb,2))
        sec = linspace(0,self.tf,self.spacing)
        delay = self.tf/(len(sec))*1
        start = time.time()
        th = []
        tm = []
        
        #calculating each joint's velocity and acceleration at each instance of time
        for t in sec:
            if t < tb and t >= 0:
                q = self.qo + 0.5*qdd_p*pow(t,2)
               
            if t < (self.tf-tb) and t >= (tb):
                q = qb_p + qdd_p*tb*(t-tb)

            if t <= self.tf and t >= (self.tf-tb):
                q = self.qf - 0.5*qdd_p*pow((t-self.tf),2)
                
            q = round(q,2)
            self.write_angle(q)
            
            t = round(t,2)
                   
            th.append(q)
            tm.append(t)

            time.sleep(delay)
            
        stop = time.time()
        exe_t = stop-start
        print("{} took {} seconds".format(self.name,exe_t))
        # print(th)
        return tm,th

def move_joints(qo,qf,tf,spacing):
    process = []
    if abs(qo[0] - qf[0]) != 0:
        joint2 = Move('Joint 1',qo[0],qf[0],tf,spacing,1)
        joint2.start(),process.append(joint2)
    else:
        pass
    
    if abs(qo[1] - qf[1]) != 0:
        joint2 = Move('Joint 2',qo[1],qf[1],tf,spacing,2)
        joint2.start(),process.append(joint2)
    else:
        pass
    
    if abs(qo[2] - qf[2]) != 0:
        joint2 = Move('Joint 3',qo[2],qf[2],tf,spacing,3)
        joint2.start(),process.append(joint2)
    else:
        pass
    
    if abs(qo[3] - qf[3]) != 0:
        joint2 = Move('Joint 4',qo[3],qf[3],tf,spacing,4)
        joint2.start(),process.append(joint2)
    else:
        pass
    
    if abs(qo[4] - qf[4]) != 0:
        joint2 = Move('Joint 5',qo[4],qf[4],tf,spacing,5)
        joint2.start(),process.append(joint2)
    else:
        pass
    
    for p in process:
        p.join()

    print("Finished")
    
my_robot = kinematics(1,1,3,2,0,0)

START = 6,0,1
TARGET = 5,0,0.5
H0_5 =  [ #Orientation of the end effector
            [1,0,0,0],
            [0,0,-1,0],
            [0,1,0,0],
            [0,0,0,1]
        ]
qo = my_robot.get_angles(START,H0_5)
qf = my_robot.get_angles(TARGET,H0_5)
move_joints(qo,qf,tf=3,spacing=10)