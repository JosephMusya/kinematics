import tkinter as tk
import trajectory
import os
import sys
import numpy as np
from tkinter import filedialog as fd,messagebox
from tkinter.filedialog import asksaveasfile as asaf
import time
os.chdir('/home/musia/RobotArm/servoblaster/')
os.system('sudo ./servod --pcm &')

gui = tk.Tk()
gui.geometry('1000x600+50+50')
gui.title('Robot GUI - Pick and Place')

x_m = tk.Label(text='X Target').place(x=10,y=50)
x_p = tk.Entry(gui)
x_p.pack()
x_p.place(x=100,y=50)

y_m = tk.Label(text='Y Target').place(x=10,y=80)
y_p = tk.Entry(gui)
y_p.pack()
y_p.place(x=100,y=80)

z_m = tk.Label(text='Z Target').place(x=10,y=110)
z_p = tk.Entry(gui)
z_p.pack()
z_p.place(x=100,y=110)

th1 = tk.DoubleVar()
th2 = tk.DoubleVar()
th3 = tk.DoubleVar()
th4 = tk.DoubleVar()
th5 = tk.DoubleVar()

def slider1_val(event):
    val = (th1.get())
    pwm = val + 60
    if SETUP:
        cmd = "echo '0={}' | sudo tee /dev/servoblaster > /dev/null".format(pwm)
        os.system(cmd)

def slider2_val(event):
    val = (th2.get())
    pwm = val + 60
    
    if SETUP:
        cmd = "echo '1={}' | sudo tee /dev/servoblaster > /dev/null".format(pwm)
        os.system(cmd)
    
def slider3_val(event):
    val = (th3.get())
    pwm = val + 60
    if SETUP:
        cmd = "echo '2={}' | sudo tee /dev/servoblaster > /dev/null".format(pwm)
        os.system(cmd)
    
def slider4_val(event):
    val = (th4.get())
    pwm = val + 60
    if SETUP:
        cmd = "echo '4={}' | sudo tee /dev/servoblaster > /dev/null".format(pwm)
        os.system(cmd)

def slider5_val(event):
    val = (th5.get()) 
    pwm = val + 60
    if SETUP:
        cmd = "echo '5={}' | sudo tee /dev/servoblaster > /dev/null".format(pwm)
        os.system(cmd)

def release():
    val=90
    pwm = val + 60
    cmd = "echo '7={}' | sudo tee /dev/servoblaster > /dev/null".format(pwm)
    os.system(cmd)
    
def grab():
    val=0
    pwm = val + 60
    cmd = "echo '7={}' | sudo tee /dev/servoblaster > /dev/null".format(pwm)
    os.system(cmd)
    
j1 = tk.Scale(
    gui,
    variable = th1,
    from_ = 0,
    to = 180,
    tickinterval = 0.1,
    orient = 'horizontal',
    command = slider1_val
    )

j2 = tk.Scale(
    gui,
    variable = th2,
    from_ = 0,
    to = 180,
    tickinterval = 0.1,    
    orient = 'horizontal',
    command = slider2_val
    )

j3 = tk.Scale(
    gui,
    variable = th3,
    from_ = 0,
    to = 180,
    tickinterval = 0.1,    
    orient = 'horizontal',
    command = slider3_val
    )

j4 = tk.Scale(
    gui,
    variable = th4,
    from_ = 0,
    to = 180,
    tickinterval = 0.1,    
    orient = 'horizontal',
    command = slider4_val
    )

j5 = tk.Scale(
    gui,
    variable = th5,
    from_ = 0,
    to = 180,
    tickinterval = 0.1,    
    orient = 'horizontal',
    command = slider5_val
    )

j1.set(1)
j1.pack()
j2.set(135)
j2.pack()
j3.set(90)
j3.pack()
j4.set(90)
j4.pack()
j5.set(1)
j5.pack()

j1.place(x=600,y=30)
j2.place(x=600,y=80)
j3.place(x=600,y=130)
j4.place(x=600,y=180)
j5.place(x=600,y=230)

joint1 = tk.Label(text='Base').place(x=530,y=50)
joint2 = tk.Label(text='Shoulder').place(x=530,y=100)
joint3 = tk.Label(text='Elbow').place(x=530,y=150)
joint4 = tk.Label(text='Roll').place(x=530,y=200)
joint5 = tk.Label(text='Pitch').place(x=530,y=250)

def home():
    global START
    global HOMED

    if not start:
        START = [19,0.1,0.1]
    from_ = START
    if not HOMED:
        print("Homing")
        trajectory.start_r(from_,HOME_POS,action='pass')
        HOMED = True    
    
def move():
    print()
    global start
    global START
    global HOMED
    global SETUP
    
    SETUP = True
    
    x_val = float(x_p.get())
    y_val = float(y_p.get())
    z_val = float(z_p.get())
    z_val = -1*(z_val)+16
    
    TARGET = [x_val,y_val,z_val]
    if not start:
        START = [20,0,0]

    start = True
    
    trajectory.start_r(START,TARGET,action='pick',)
    START = TARGET
    HOMED = False

def change_pos(axis):
    global PATH_POS
    global x
    global y
    global z
    
    axis = str(axis)

    if axis=='x+':
        x = PATH_POS[0]+1
        if x > 30:
            x=30
    elif axis=='x-':
        x = PATH_POS[0]-1
        if x < 15:
            x=15
            
    if axis=='y+':
        y = PATH_POS[1]+1
        if y > 30:
            y=30
    elif axis=='y-':
        y = PATH_POS[1]-1
        if y < 0:
            y=0
            
    if axis=='z+':
        z = PATH_POS[2]+1
        if z > 15:
            z=15
    elif axis=='z-':
        z = PATH_POS[2]-1
        if z < 1:
            z=0.5

    print('{} {} {}'.format(x,y,z))
    trajectory.start_r(start=PATH_POS,target=[x,y,z],action='pass',tf=0.0001)
    time.sleep(0.5)
    PATH_POS = [x,y,z]
    
def save_point(x,y,z,**kwargs):
    if kwargs:
        action = kwargs['action']
        if action == 'pick':
            grab()
        elif action == 'place':
            release()
    else:
        action = 'pass'
    point = [[x,y,z],[action]]
    if point != PTS[-1]:        
        PTS.append(point)
        print(PTS)
def save_path():
    os.chdir('/home/musia/RobotArm/kinematics/paths')
    file = asaf(initialfile='Path1',defaultextension='.npy',filetypes=[('All Files',"*.*")])#np.savetxt('/home/musia/RobotArm/kinematics/Path1.txt',PTS)
    #array = np.asanyarray(PTS)
    np.save(file.name,PTS)
    print("File Saved")
    os.chdir('/home/musia/RobotArm/servoblaster/')
def open_file():
    global PATH
    os.chdir('/home/musia/RobotArm/kinematics/paths')
    file = fd.askopenfilename(title='Select File with path',filetypes=[('Text Files',"*.npy")])
    if file:
        PATH = np.load(file,allow_pickle=True)        
    os.chdir('/home/musia/RobotArm/servoblaster/')
def follow_path():
    global path
    size = len(PATH)
    if size == 0:
        messagebox.showerror("Error","Could not read the points")
        return
    for itr in range(0,size-1):
        start = PATH[itr][0]
        target = PATH[itr+1][0]
        action = PATH[itr+1][1]
        itr=+1
        trajectory.start_r(start,target,action=action)
        last_point = target
        #time.sleep(0.01)
    trajectory.start_r(start=last_point,target=PATH[0][0],action=action)
    follow_path()                 
    
#PICK/pick/HOME/
SETUP = False
HOMED = False
HOME_POS = [20,0,0]
PATH_POS = [20,0,0]
PTS = [[[20,0,0],['move']]]
PATH = []
x = PATH_POS[0]
y = PATH_POS[1]
z = PATH_POS[2]

start = False

#home()

move_btn = tk.Button(gui,text='Move',command=move).place(x=10,y=150)
release_btn = tk.Button(gui,text='Release',command=release).place(x=550,y=300)
graspt_btn = tk.Button(gui,text='Grab',command=grab).place(x=650,y=300)
home_btn = tk.Button(gui,text='Home',command=home).place(x=80,y=150)

x_plus = tk.Button(gui,text='+',command=lambda: change_pos('x+')).place(x=80,y=250)
x_minus = tk.Button(gui,text='-',command=lambda: change_pos('x-')).place(x=20,y=250)
x_m = tk.Label(text='X').place(x=60,y=250)

x_plus = tk.Button(gui,text='+',command=lambda: change_pos('y+')).place(x=80,y=300)
x_minus = tk.Button(gui,text='-',command=lambda: change_pos('y-')).place(x=20,y=300)
x_m = tk.Label(text='Y').place(x=60,y=300)

x_plus = tk.Button(gui,text='+',command=lambda: change_pos('z+')).place(x=80,y=350)
x_minus = tk.Button(gui,text='-',command=lambda: change_pos('z-')).place(x=20,y=350)
x_m = tk.Label(text='Z').place(x=60,y=350)

g = tk.Button(gui,text='G',command=lambda: save_point(x,y,z,action='pick')).place(x=150,y=250)
r = tk.Button(gui,text='R',command=lambda: save_point(x,y,z,action='place')).place(x=150,y=300)

save_pos = tk.Button(gui,text='Save Point', command=lambda: save_point(x,y,z)).place(x=20,y=400)
save_path = tk.Button(gui,text='Save Path', command=save_path).place(x=150,y=400)

browse_btn = tk.Button(gui,text='Browse File',command=open_file).place(x=20,y=440)
start_btn = tk.Button(gui,text='Start',command=follow_path).place(x=150,y=440)


gui.mainloop()