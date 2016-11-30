import sys
import cv2

def exitScript(vidSource):
    cv2.destroyAllWindows()
    vidSource.release()
    sys.exit("User pressed ESC. Quit.")  
