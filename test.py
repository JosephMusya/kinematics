import cv2 #computer vision library
import camera #camera module
import aruco #aruco marker for localizing objects
from trajectory import start #moves the robot arm joints
from cam_test import recognize

params = cv2.aruco.DetectorParameters_create() #Obtain the aruco marker parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)

ROBOT_LOC = [-41,-0] #Robot location from the work frame's (0,0)

while True:
    frame = camera.get_frame() #Obtain the camera frame, while loop is used to get continuos frames
    frame,corners,id = aruco.aruco_marker(frame,aruco_dict,params)
    try:
        if len(id) > 0: #If we have some markers that have been detected
        #Get the camera offset if we have id=1 and id=3 meaning we have two markers
            CAM_OFFSET = aruco.distance(corners,frame,id,kwargs=[])
            #measure the distance with the key word arguments being (x,y,width)
            dist = aruco.distance(corners, frame, id, kwargs = [300,300,89])
            xr = dist[0] - ROBOT_LOC[0] #Get the object x location relative to the robot arm origin
            yr = dist[1] - ROBOT_LOC[1] #Get the object y location relative to the robot arm origin
            zr = 0 #The Z location is always zero unless otherwise
            print("x: {} y: {}".format(xr,-yr))
            #action='pick'
            #start([28,0,11.903],[xr,yr,zr],action)
            print("Camera offset:  ",CAM_OFFSET)
            #print("X: {} Y: {} OFFSSET: {}".format(round(dist[0],2),round(dist[1],2),CAM_OFFSET))
    except Exception as e: #This executes when their is no IDs found
        #print("Error: ",e)
        pass
    
    
    
    #action='pick'
    #start(x,y,action)
    label,score = recognize(frame)
    if label and score:
        print("Label: {} Score: {} ".format(label,score))
    cv2.circle(frame,(300,300),4,(0,0,255),-1)
    frame = cv2.resize(frame,(320,320))
    cv2.imshow("Aruco Marker",frame)