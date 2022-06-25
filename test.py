import cv2
import camera
import aruco
params = cv2.aruco.DetectorParameters_create()
aruco_dict = cv2.aruco.Dictionary_get(
    cv2.aruco.DICT_5X5_50)

while True:    
    frame = camera.get_frame()   
    frame,corners,id = aruco.aruco_marker(frame,aruco_dict,params)
    try:
        if len(id)>0:
            CAM_OFFSET = aruco.distance(corners,frame,id)
            print(CAM_OFFSET)
    except TypeError:
        print("Markers not detected")

    frame = cv2.flip(frame,1)             

    cv2.imshow("Aruco Marker",frame)