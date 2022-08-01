from numpy import * #Import numerical python numpy (Math Library)
class kinematics():
    def __init__(self,a1,a2,a3,a4,a5,a6):
        self.a1 = a1 #Link 1 length
        self.a2 = a2 #Link 2 Length
        self.a3 = a3 #Link 3 Length
        self.a4 = a4 #Link 4 Length
        self.a5 = a5
        self.a6 = a6
    # ikine >>> Inverse Kinematics | Takes the desired cordinates, 
    # and returns the desired first angles (theta1,theta2,theta3)
    def ikine(self,x,y,z):
        r3 = sqrt(x*x + y*y)
        r2 = z - self.a1
        r1 = sqrt(r2*r2 + (r3-self.a2)*(r3-self.a2)) 
        phi2 = arctan2(r2,(r3-self.a2))
        phi1 = arccos((self.a4*self.a4 - self.a3*self.a3 - r1*r1)/(-2*self.a3*r1))
        phi3 = arccos((r1*r1 - self.a3*self.a3 - self.a4*self.a4)/(-2*self.a3*self.a4))
        theta1 = round(rad2deg(arctan2(y,x)),2)
        theta2 = round(rad2deg(phi2 - phi1),2)
        theta3 = round(rad2deg(deg2rad(180) - phi3),2)
        return theta1,theta2,theta3
    
    #DHP >>> Denavit Herternberg Parameters | Takes the DH parameter
    # of the robot links and returns an homogenous transformation matrix (HTM)
    def DHP(self,th,alph,d,r):
        th = deg2rad(th)
        alph = deg2rad(alph)
        HTM = [
            [cos(th), -sin(th)*cos(alph), sin(th)*sin(alph), r*cos(th)],
            [sin(th), cos(th)*cos(alph), -cos(th)*sin(alph), r*sin(th)],
            [0, sin(alph), cos(alph), d],
            [0, 0, 0, 1]
        ]
        return matrix(HTM)
    
    #Takes the first three link angles and returns the homogenous tranformation matrix
    def pos(self,th1,th2,th3):
        H0_1 = self.DHP(th1,90,self.a1,self.a2)
        H1_2 = self.DHP(th2,0,0,self.a3)
        H2_3 = self.DHP(th3+90,90,0,0)
        
        H0_2 = dot(H0_1,H1_2)
        H0_3 = dot(H0_2,H2_3)
        
        return H0_3
    def get_angles(self,POINT,ORI):
        x = POINT[0]
        y = POINT[1]
        z = POINT[2]

        th1,th2,th3 = self.ikine(x,y,z)
        H0_3 = self.pos(th1,th2,th3)
        H3_5 = dot(linalg.inv(H0_3),ORI)
        th4 = round(rad2deg(arcsin(H3_5[0,2])),2) #Theta4
        if y == 0:
            th5 = round(rad2deg(arcsin(H3_5[2,1])),2) #Theta5
        else:
            th5 = round(rad2deg(arccos(H3_5[0,2])),2) #Theta5
        
        return [th1,th2,th3,th4,th5]  
    def path(self,qo,qf,tf,space):
        qdd,qb = [],[]
        th1_a,th2_a,th3_a,th4_a,th5_a = [],[],[],[],[]
        for i in range(len(qo)):
            qdd_p = 4*(qf[i]-qo[i])/pow(tf,2)
            x = pow(qdd_p,2)*pow(tf,2)
            y = 4*qdd_p*(qf[i]-qo[i])
            num = sqrt((x-y))
            tb = (0.5*tf) - num/(2*qdd_p)
            qb_p = qo[i] + (0.5*qdd_p*pow(tb,2))
            qdd.append(qdd_p),qb.append(qb_p)

        sec = linspace(0,tf,space)       
        for t in sec:
            for j in arange(len(qo)):
                if t < tb and t >= 0:
                    q = qo[j] + 0.5*qdd[j]*pow(t,2)
                if t < (tf-tb) and t >= (tb):
                    q = qb[j] + qdd[j]*tb*(t-tb)
                if t <= tf and t >= (tf-tb):
                    q = qf[j] - 0.5*qdd[j]*pow((t-tf),2)
                q = round(q,1)
                if j == 0:
                    th1_a.append(q)
                if j == 1:
                    th2_a.append(q)
                if j == 2:
                    th3_a.append(q)
                if j == 3:
                    th4_a.append(q)
                if j == 4:
                    th5_a.append(q)
        return th1_a,th2_a,th3_a,th4_a,th5_a    
    
if __name__ == '__main__':
    try:
        my_robot = kinematics(1,1,3,2,0,0) #Arguments are link1,link2,link3,link4
        x,y,z = float(input("X: ")),float(input("Y: ")),float(input("Z: "))
        # x,y,z = 6,0,1
        th1,th2,th3 = my_robot.ikine(x,y,z) #Returns theta1,theta2,theta3
        
        H0_5 = [ #Orientation of the end effector
            [1,0,0,0],
            [0,0,-1,0],
            [0,1,0,0],
            [0,0,0,1]
        ]
        
        H0_3 = my_robot.pos(th1,th2,th3) #Gets the Homogenous matrix from 0 to 3
        H3_5 = dot(linalg.inv(H0_3),H0_5) #Gets the Homogenous matrix from 3 to 5
        print(H3_5)
        print('-------------------------------------------------------------------------------')
        th4 = round(rad2deg(arcsin(H3_5[0,2])),2) #Theta4
        th5 = round(rad2deg(arcsin(H3_5[2,1])),2) #Theta5
        # H3_5 = my_robot.ori(th4,th5)
        # print(H3_5)
        print("Th1={} Th2={} Th3={} Th4={} Th5={} ".format(th1,th2,th3,th4,th5))
    except Exception as e: 
        print("ERROR ==> {}".format(e))

    #Send Command to servos no angles are none
    
