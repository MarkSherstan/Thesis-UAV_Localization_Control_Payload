from threading import Thread
import pyrealsense2 as rs
import numpy as np
import math
import time
import cv2

class T265:
    def __init__(self):
        # Data
        self.rawImg1 = None
        self.rawImg2 = None
        self.Img1    = None
        self.Img2    = None
        self.psiRate = None
        self.vx      = None
        self.vy      = None
        self.vz      = None 

        # Capture threading parameters
        self.isReceivingFrame = False
        self.isRunFrame = True
        self.frameThread = None

        # Performance
        self.frameStartTime = None
        self.frameCount = 0
        
        # Mapping coeffcients
        self.map1A = None
        self.map1B = None
        self.map2A = None
        self.map2B = None
        
        # Pipeline 
        self.pipe = None
        
        # Start up procedure
        self.start()

    def start(self):
        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()

        # Build config object and stream
        cfg = rs.config()
        cfg.enable_stream(rs.stream.fisheye, 1)
        cfg.enable_stream(rs.stream.fisheye, 2)       
        cfg.enable_stream(rs.stream.pose)

        # Start streaming
        self.pipe.start(cfg)        
        
        # Calculate the camera mappings 
        self.createMaps()
        
        # Start streaming data in another thread
        self.startFrameThread()

    def startFrameThread(self):
        # Create a thread
        if self.frameThread == None:
            self.frameThread = Thread(target=self.acquireFrame)
            self.frameThread.start()
            print('Capture thread start')

            # Block till we start receiving values
            while self.isReceivingFrame != True:
                time.sleep(0.1)

            # Start the timer 
            self.frameCount = 0
            self.frameStartTime = time.time()
            
    def acquireFrame(self):
        # Acquire until closed
        while(self.isRunFrame):
            # Wait for data (blocking)
            frames = self.pipe.wait_for_frames()
            
            # Extract fisheye and pose data
            f1 = frames.get_fisheye_frame(1)
            f2 = frames.get_fisheye_frame(2)
            pose = frames.get_pose_frame()
            
            # Error checking
            if not f1 or not f2 or not pose:
                continue

            # Data type conversion and extraction
            self.rawImg1 = np.asanyarray(f1.get_data())
            self.rawImg2 = np.asanyarray(f2.get_data())
            self.psiRate = math.degrees(pose.get_pose_data().angular_velocity.y)
            self.vx = pose.get_pose_data().velocity.x
            self.vy = pose.get_pose_data().velocity.y
            self.vz = pose.get_pose_data().velocity.z 
        
            # Undistort the images
            self.Img1 = cv2.remap(self.rawImg1, self.map1A, self.map1B, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            self.Img2 = cv2.remap(self.rawImg2, self.map2A, self.map2B, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            
            # Performance and threading
            self.frameCount += 1
            self.isReceivingFrame = True

    def cameraMatrix(self, intrinsics):
        # Returns a camera matrix K from librealsense intrinsics
        return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                        [            0,  intrinsics.fy, intrinsics.ppy],
                        [            0,             0,              1]])

    def fisheyeDistortion(self, intrinsics):
        # Returns the fisheye distortion from librealsense intrinsics
        return np.array(intrinsics.coeffs[:4])
                            
    def createMaps(self):
        # Retreive the stream and intrinsic properties for both cameras
        profiles = self.pipe.get_active_profile()
        streams = {"f1" : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
                   "f2" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}
        intrinsics = {"f1" : streams["f1"].get_intrinsics(),
                      "f2" : streams["f2"].get_intrinsics()}
    
        # Translate the intrinsics from librealsense into OpenCV
        K1 = self.cameraMatrix(intrinsics["f1"])
        D1 = self.fisheyeDistortion(intrinsics["f1"])
        K2 = self.cameraMatrix(intrinsics["f2"])
        D2 = self.fisheyeDistortion(intrinsics["f2"])

        # Find mappings for fixing image
        (self.map1A, self.map1B) = cv2.fisheye.initUndistortRectifyMap(K1, D1, np.eye(3), K1, (848, 800), cv2.CV_16SC2)
        (self.map2A, self.map2B) = cv2.fisheye.initUndistortRectifyMap(K2, D2, np.eye(3), K2, (848, 800), cv2.CV_16SC2)

    def close(self):
        # Close the capture thread
        self.isRunFrame = False
        self.frameThread.join()
        print('Capture thread closed')
        
        # Performance
        print('Frame rate T265: ', round(self.frameCount / (time.time() - self.frameStartTime),1))

        # Close the pipe
        self.pipe.stop()

def main():
    cam = T265()
    
    while(True):
        # Show the image frames 
        showFrame = np.concatenate((cam.Img1, cam.Img2), axis=1)
        cv2.imshow('Frame', showFrame)
        
        # Exit
        key = cv2.waitKey(1)
        if key != -1:
            if key & 0xFF == ord('q'):
                break
            if key & 0xFF == ord(' '):
                cv2.imwrite('raw.png',cam.Img1)
                cv2.imwrite('flat.png',cam.Img2)
    
    cv2.destroyAllWindows()    
    cam.close()   

if __name__ == "__main__":
    main()
