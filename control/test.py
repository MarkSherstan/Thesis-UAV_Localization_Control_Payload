from T265 import T265
from draw import Draw
import numpy as np
import cv2

def main():
    # Camera matrices 
    mtx1 = np.array([[284.65527842,  0.0,            420.10118496],
                     [0.0,           285.43891088,   403.82029423],
                     [0.0,           0.0,            1.0         ]])
    dist1 = np.array([[-0.01159942, 0.00393409, 0.00059457, -0.0002535, -0.0006091]])

    mtx2 = np.array([[287.8954394,  0.0,            418.40412543],
                     [0.0,           287.99235758,   410.12408383],
                     [0.0,           0.0,            1.0         ]])
    dist2 = np.array([[-0.00818909, 0.00187817, 0.00132013, -0.00018278, -0.00044735]])

    # Start up classes
    cam = T265()
    d1 = Draw(mtx1, dist1)
    d2 = Draw(mtx2, dist2)
    
    while(True):
        # Process the frame (ArUco board)
        img1 = d1.arucoBoard(cam.Img1)
        img2 = d2.arucoBoard(cam.Img2)
        
        # Process the frame (ArUco marker)
        # img1 = d1.arucoMarker(cam.Img1)
        # img2 = d2.arucoMarker(cam.Img2)

        # Show the image frames 
        showFrame = np.concatenate((img1, img2), axis=1)
        cv2.imshow('Frame', showFrame)
        
        # Exit
        key = cv2.waitKey(1)
        if key != -1:
            if key & 0xFF == ord('q'):
                break
            if key & 0xFF == ord(' '):
                cv2.imwrite('Cam1.png',img1)
                cv2.imwrite('Cam2.png',img2)
    
    cv2.destroyAllWindows()    
    cam.close()   

if __name__ == "__main__":
    main()
