import pyrealsense2 as rs
import numpy as np
import time
import cv2


class T265:
    def __init__(self):
        # Data
        self.rawImg1 = None
        self.rawImg2 = None
        
        # Capture threading parameters
        self.isReceivingFrame = False
        self.isRunFrame = True
        self.frameThread = None
        self.frame = None
        
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
        
        # Start streaming data in another thread
        self.startFrameThread()
                
    def camera_matrix(self, intrinsics):
        # Returns a camera matrix K from librealsense intrinsics
        return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                        [            0, intrinsics.fy, intrinsics.ppy],
                        [            0,             0,              1]])

    def fisheye_distortion(self, intrinsics):
        # Returns the fisheye distortion from librealsense intrinsics
        return np.array(intrinsics.coeffs[:4])
        
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
            self.frameStartTime = time.time()
            
    def acquireFrame(self, cam):
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

            # Data type conversion
            self.rawImg1 = np.asanyarray(f1.get_data())
            self.rawImg2 = np.asanyarray(f2.get_data())
            poseData = pose.get_pose_data()
        
            # Performance and threading
            self.frameCount += 1
            self.isReceivingFrame = True
                        
    def prepCamera(self):
        # Retreive the stream and intrinsic properties for both cameras
        profiles = self.pipe.get_active_profile()
        streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
                   "right" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}
        intrinsics = {"left"  : streams["left"].get_intrinsics(),
                      "right" : streams["right"].get_intrinsics()}
    
        # Translate the intrinsics from librealsense into OpenCV
        K_left  = self.camera_matrix(intrinsics["left"])
        D_left  = self.fisheye_distortion(intrinsics["left"])
        K_right = self.camera_matrix(intrinsics["right"])
        D_right = self.fisheye_distortion(intrinsics["right"])

        # Method two
        (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, np.eye(3), K_left, (800, 848), cv2.CV_16SC2)
        (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, np.eye(3), K_right, (800, 848), cv2.CV_16SC2)
        self.undistort_rectify = {"left"  : (lm1, lm2),
                                  "right" : (rm1, rm2)}
                
    def close(self):
        self.pipe.stop()

def main():
    cam = T265()
    
    try:
        # Show the image frames 
        showFrame = np.concatenate((cam.rawImg1, cam.rawImg2), axis=1)
        cv2.imshow('Frame', showFrame)

        # Small delay
        time.sleep(1/45)
        
        # Exit
        key = cv2.waitKey(1)
        if key != -1:
            if key & 0xFF == ord('q'):
                break
    except:
        cam.close()


if __name__ == "__main__":
    main()







# class T265:
#     def __init__(self):
#         # Set up a mutex to share data between threads 
#         self.frameMutex = Lock()
#         self.frameData = {"left"  : None,
#                           "right" : None,
#                           "timestamp_ms" : None}
        
#         # Declare RealSense pipeline, encapsulating the actual device and sensors
#         self.pipe = rs.pipeline()

#         # Build config object and stream everything
#         cfg = rs.config()

#         # Start streaming with our callback
#         self.pipe.start(cfg, self.callback)
        
#         # prep the camera
#         self.prepCamera()
        
#         # Image frames
#         self.L = None 
#         self.R = None

#     def callback(self, frame):
#         """
#         This callback is called on a separate thread, so we must use a mutex
#         to ensure that data is synchronized properly. We should also be
#         careful not to do much work on this thread to avoid data backing up in the
#         callback queue.
#         """
#         if frame.is_frameset():
#             frameset = frame.as_frameset()
#             f1 = frameset.get_fisheye_frame(1).as_video_frame()
#             f2 = frameset.get_fisheye_frame(2).as_video_frame()
#             left_data = np.asanyarray(f1.get_data())
#             right_data = np.asanyarray(f2.get_data())
#             ts = frameset.get_timestamp()
            
#             self.frameMutex.acquire()
#             self.frameData["left"] = left_data
#             self.frameData["right"] = right_data
#             self.frameData["timestamp_ms"] = ts
#             self.frameMutex.release()

#     def camera_matrix(self, intrinsics):
#         # Returns a camera matrix K from librealsense intrinsics
#         return np.array([[intrinsics.fx,             0, intrinsics.ppx],
#                         [            0, intrinsics.fy, intrinsics.ppy],
#                         [            0,             0,              1]])

#     def fisheye_distortion(self, intrinsics):
#         # Returns the fisheye distortion from librealsense intrinsics
#         return np.array(intrinsics.coeffs[:4])

#     def prepCamera(self):
#         # Retreive the stream and intrinsic properties for both cameras
#         profiles = self.pipe.get_active_profile()
#         streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
#                    "right" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}
#         intrinsics = {"left"  : streams["left"].get_intrinsics(),
#                       "right" : streams["right"].get_intrinsics()}
    
#         # Translate the intrinsics from librealsense into OpenCV
#         K_left  = self.camera_matrix(intrinsics["left"])
#         D_left  = self.fisheye_distortion(intrinsics["left"])
#         K_right = self.camera_matrix(intrinsics["right"])
#         D_right = self.fisheye_distortion(intrinsics["right"])

#         # Method two
#         (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, np.eye(3), K_left, (800, 848), cv2.CV_16SC2)
#         (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, np.eye(3), K_right, (800, 848), cv2.CV_16SC2)
#         self.undistort_rectify = {"left"  : (lm1, lm2),
#                                   "right" : (rm1, rm2)}

#     def run(self):
#         while True:
#             # Check if the camera has acquired any frames
#             self.frameMutex.acquire()
#             valid = self.frameData["timestamp_ms"] is not None
#             self.frameMutex.release()

#             print(valid)

#             # If frames are ready to process
#             if valid:
#                 # Hold the mutex only long enough to copy the stereo frames
#                 self.frameMutex.acquire()
#                 frame_copy = {"left"  : self.frameData["left"].copy(),
#                               "right" : self.frameData["right"].copy()}
#                 self.frameMutex.release()

#                 # Undistort and crop the center of the frames
#                 center_undistorted = {"left" : cv2.remap(src = frame_copy["left"],
#                                             map1 = self.undistort_rectify["left"][0],
#                                             map2 = self.undistort_rectify["left"][1],
#                                             interpolation = cv2.INTER_LINEAR,
#                                             borderMode=cv2.BORDER_CONSTANT),
#                                       "right" : cv2.remap(src = frame_copy["right"],
#                                             map1 = self.undistort_rectify["right"][0],
#                                             map2 = self.undistort_rectify["right"][1],
#                                             interpolation = cv2.INTER_LINEAR,
#                                             borderMode=cv2.BORDER_CONSTANT)}

#                 # Left and right stream
#                 L = center_undistorted["left"]
#                 R = center_undistorted["right"]
                
#                 # Show the image frames 
#                 showFrame = np.concatenate((L, R), axis=1)
#                 cv2.imshow('Frame', showFrame)

#                 # Check keyboard commands
#                 #   'space' -> Snapshot
#                 #   'q' -> Quit
#                 key = cv2.waitKey(1)

#                 if key != -1:
#                     if key & 0xFF == ord('q'):
#                         break
                
#     def close(self):
#         self.pipe.stop()

# def main():
#     cam = T265()
    
#     try:
#         cam.run()
#     except:
#         cam.close()


# if __name__ == "__main__":
#     main()
