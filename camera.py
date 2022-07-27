import cv2
import sys
cap = cv2.VideoCapture(-1)
#cap.set(3,640)
#cap.set(4,480)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT,320)
def get_frame():
    _,frame = cap.read()
    if cv2.waitKey(1)== ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        sys.exit()
    return frame
if __name__== '__main__':
    while True:        
        frame = get_frame()
        print(frame)
        cv2.imshow("CAM TEST",frame)
    
