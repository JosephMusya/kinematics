import cv2
import numpy as np
  
def aruco_marker(frame,aruco_dict,params):
   corners,id,_= cv2.aruco.detectMarkers(frame,
                                    aruco_dict,
                                    parameters=params)
   # cv2.polylines(frame,int_corners,True,(0,255,0),2)
   if corners:                        
      for (markerCorner, markerID) in zip(corners, id):   
         corner = markerCorner.reshape((4, 2))
               
         (topLeft, topRight, bottomRight, bottomLeft) = corner
         # convert each of the (x, y)-coordinate pairs to integers
         topRight = (int(topRight[0]), int(topRight[1]))
         bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
         bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
         topLeft = (int(topLeft[0]), int(topLeft[1]))
         
         cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
         cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
         cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
         cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
         
         cX = int((topLeft[0] + bottomRight[0]) / 2.0)
         cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                  
         cv2.putText(frame, str(markerID),(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
			0.5, (0, 255, 0), 1)         
      return frame,corners,id
   
   return frame,corners,id

def get_ratio(corners):
   aruco_peri = cv2.arcLength(corners[0],True) 
   px_to_cm = aruco_peri/20
   return px_to_cm

def measure(ratio,loc,**kwargs):
   kwargs = kwargs['kwargs']
   x_dist = loc[0]/ratio
   y_dist = loc[1]/ratio
   if kwargs:
      width = kwargs[0]/ratio
      return x_dist,y_dist,width
   return x_dist,y_dist

WIDTH = 640
HEIGHT = 480
REF_ID_START = 1
REF_ID_OFFSET = 3
   # frame = cv2.resize(frame,(WIDTH,HEIGHT))  
def distance(corners,frame,id,**kwargs):
   kwargs = kwargs['kwargs']
   if ([REF_ID_START] in id):
      index = np.where(id==REF_ID_START)[0]
      for i in index:
         ref_corner = (corners[i])
         ref_corner = ref_corner.reshape((4, 2))
         tL, _, bR, _ = ref_corner[0],0,ref_corner[2],0            
         x_center = int((tL[0] + bR[0]) / 2.0)
         y_center = int((tL[1] + bR[1]) / 2.0)
         cv2.circle(frame, (int(x_center),int(y_center)), 4, (255, 0, 0), -1)                       
         ratio = get_ratio(corners)
         
   if ([REF_ID_OFFSET] in id) and len(id) > 0:
      index = np.where(id==REF_ID_OFFSET)[0]
      for i in index:
         ref_corner = (corners[i])
         ref_corner = ref_corner.reshape((4, 2))
         tL, _, bR, _ = ref_corner[0],0,ref_corner[2],0            
         x_pos = int((tL[0] + bR[0]) / 2.0)
         y_pos = int((tL[1] + bR[1]) / 2.0)

   else:
      #Refrence marker not located
      pass   
   if ([REF_ID_START] in id):
      x_c,y_c = measure(ratio,[x_center,y_center],kwargs=[])                       
         
      if kwargs:
         x_px = kwargs[0]
         y_px = kwargs[1]
         width = kwargs[2]
         
         x_pos,y_pos,width = measure(ratio,[x_px,y_px],kwargs=[width])
         return [x_pos-x_c,y_pos-y_c,width]
      
      if([REF_ID_OFFSET] in id):
         x_dist,y_dist = measure(ratio,[x_pos,y_pos],kwargs=[])
         return y_dist-y_c
   
      
