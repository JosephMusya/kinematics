import cv2
import numpy as np

WIDTH = 640
HEIGHT = 480

cap = cv2.VideoCapture(0)
params = cv2.aruco.DetectorParameters_create()
aruco_dict = cv2.aruco.Dictionary_get(
    cv2.aruco.DICT_5X5_50
)
def get_ratio(corners,frame):
   aruco_peri = cv2.arcLength(corners[0],True) 
   px_to_cm = aruco_peri/20
   
   x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
   y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
   
   x_cp = x_sum*.25
   y_cp = y_sum*.25
   
   cv2.circle(frame,(int(x_cp),int(y_cp)),5,(255,0,0),-1)
   
   return px_to_cm,x_cp,y_cp

def measure(px,ratio):
   dist = px/ratio
   return dist

while True:
   _,frame = cap.read() 
   frame = cv2.resize(frame,(WIDTH,HEIGHT))
   corners,id,_= cv2.aruco.detectMarkers(frame,
                                    aruco_dict,
                                    parameters=params)
   int_corners = np.int0(corners)   
   print(id)
   cv2.polylines(frame,int_corners,True,(0,255,0),2)
#    print(corners)  
   if corners: 
      ratio,x_cp,y_cp = get_ratio(corners,frame)
      frame = cv2.line(frame,(int(x_cp),int(y_cp)),(HEIGHT,int(y_cp)),(0,255,0),2)
   
      # dist = measure(ratio,param)
   frame = cv2.flip(frame,1)
   cv2.imshow("Aruco Marker",frame)
   
   if cv2.waitKey(1) & 0xFF == ord('q'):
       break
cap.release()
cv2.destroyAllWindows()