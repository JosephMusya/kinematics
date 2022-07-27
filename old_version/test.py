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
        if len(id) > 0:
            CAM_OFFSET = aruco.distance(corners,frame,id,kwargs=[])            
            dist = aruco.distance(corners, frame, id, kwargs = [300,300,89])
            # print("Camera offset:  ",CAM_OFFSET)
            print("Distance: ",round(dist[0],2),round(dist[1],2),round(dist[2],2))
    except Exception as e:
        print("Error: ",e)

    cv2.circle(frame,(300,300),4,(0,0,0),-1)
    cv2.imshow("Aruco Marker",frame)
