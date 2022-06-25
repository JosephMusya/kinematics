import cv2
cap = cv2.VideoCapture(0)
params = cv2.aruco.DetectorParameters_create()

def get_frame():   
    _,frame = cap.read()
    
    if cv2.waitKey(1) & 0xFF == ord('q'):                
        cap.release()
        cv2.destroyAllWindows()
    return frame
