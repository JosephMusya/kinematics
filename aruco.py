import cv2
import numpy as np
cap = cv2.VideoCapture(0)
params = cv2.aruco.DetectorParameters_create()
aruco_dict = cv2.aruco.Dictionary_get(
    cv2.aruco.DICT_5X5_50
)
def get_ratio(corners):
   aruco_peri = cv2.arcLength(corners[0],True) 
   px_to_cm = aruco_peri/20
   return px_to_cm
def measure(px,ratio):
   dist = px/ratio
while True:
   _,frame = cap.read() 
   corners,_,_= cv2.aruco.detectMarkers(frame,
                                    aruco_dict,
                                    parameters=params)
   int_corners = np.int0(corners)   

   cv2.polylines(frame,int_corners,True,(0,255,0),2)
#    print(corners)  
   if corners: 
      ratio = get_ratio(corners)
      
      # w = w_px/ratio

   cv2.imshow("Aruco Marker",frame)
   
   if cv2.waitKey(1) & 0xFF == ord('q'):
       break
cap.release()
cv2.destroyAllWindows()