# from threading import Lock
# import pyrealsense2 as rs
# import numpy as np
# import time
# import cv2

# # Documentation -> https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.html#module-pyrealsense2

# """
# In this section, we will set up the functions that will translate the camera
# intrinsics and extrinsics from librealsense into parameters that can be used
# with OpenCV.

# The T265 uses very wide angle lenses, so the distortion is modeled using a four
# parameter distortion model known as Kanalla-Brandt. OpenCV supports this
# distortion model in their "fisheye" module, more details can be found here:

# https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html
# """

# def camera_matrix(intrinsics):
#     # Returns a camera matrix K from librealsense intrinsics
#     return np.array([[intrinsics.fx,             0, intrinsics.ppx],
#                      [            0, intrinsics.fy, intrinsics.ppy],
#                      [            0,             0,              1]])

# def fisheye_distortion(intrinsics):
#     # Returns the fisheye distortion from librealsense intrinsics
#     return np.array(intrinsics.coeffs[:4])

# # Set up a mutex to share data between threads 
# frame_mutex = Lock()
# frame_data = {"left"  : None,
#               "right" : None,
#               "timestamp_ms" : None
#               }

# def callback(frame):
#     """
#     This callback is called on a separate thread, so we must use a mutex
#     to ensure that data is synchronized properly. We should also be
#     careful not to do much work on this thread to avoid data backing up in the
#     callback queue.
#     """
#     global frame_data
#     if frame.is_frameset():
#         frameset = frame.as_frameset()
#         f1 = frameset.get_fisheye_frame(1).as_video_frame()
#         f2 = frameset.get_fisheye_frame(2).as_video_frame()
#         left_data = np.asanyarray(f1.get_data())
#         right_data = np.asanyarray(f2.get_data())
#         ts = frameset.get_timestamp()
        
#         frame_mutex.acquire()
#         frame_data["left"] = left_data
#         frame_data["right"] = right_data
#         frame_data["timestamp_ms"] = ts
#         frame_mutex.release()

# # Declare RealSense pipeline, encapsulating the actual device and sensors
# pipe = rs.pipeline()

# # Build config object and stream everything
# cfg = rs.config()

# # Start streaming with our callback
# pipe.start(cfg, callback)

# try:
#     # Set up an OpenCV window to visualize the results
#     WINDOW_TITLE = 'Realsense'
#     cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)

#     # Retreive the stream and intrinsic properties for both cameras
#     profiles = pipe.get_active_profile()
#     streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
#                "right" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}
#     intrinsics = {"left"  : streams["left"].get_intrinsics(),
#                   "right" : streams["right"].get_intrinsics()}
   
#     # Translate the intrinsics from librealsense into OpenCV
#     K_left  = camera_matrix(intrinsics["left"])
#     D_left  = fisheye_distortion(intrinsics["left"])
#     K_right = camera_matrix(intrinsics["right"])
#     D_right = fisheye_distortion(intrinsics["right"])

#     # Method two
#     DIM = (800, 848)
#     (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, np.eye(3), K_left, DIM, cv2.CV_16SC2)
#     (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, np.eye(3), K_right, DIM, cv2.CV_16SC2)
    
#     undistort_rectify = {"left"  : (lm1, lm2),
#                          "right" : (rm1, rm2)}

#     while True:
#         # Check if the camera has acquired any frames
#         frame_mutex.acquire()
#         valid = frame_data["timestamp_ms"] is not None
#         frame_mutex.release()

#         # If frames are ready to process
#         if valid:
#             # Hold the mutex only long enough to copy the stereo frames
#             frame_mutex.acquire()
#             frame_copy = {"left"  : frame_data["left"].copy(),
#                           "right" : frame_data["right"].copy()}
#             frame_mutex.release()

#             # Undistort and crop the center of the frames
#             center_undistorted = {"left" : cv2.remap(src = frame_copy["left"],
#                                           map1 = undistort_rectify["left"][0],
#                                           map2 = undistort_rectify["left"][1],
#                                           interpolation = cv2.INTER_LINEAR,
#                                           borderMode=cv2.BORDER_CONSTANT),
#                                   "right" : cv2.remap(src = frame_copy["right"],
#                                           map1 = undistort_rectify["right"][0],
#                                           map2 = undistort_rectify["right"][1],
#                                           interpolation = cv2.INTER_LINEAR,
#                                           borderMode=cv2.BORDER_CONSTANT)}

#             # Left and right stream
#             L = center_undistorted["left"]
#             R = center_undistorted["right"]

#             LL = frame_copy["left"]
#             RR = frame_copy["right"]

#             print(L.shape, R.shape, LL.shape, RR.shape)
#             # Get some info in prep for processing
#             # print(type(L), type(R), L.shape, R.shape)

#             # Show the frame
#             # cv2.imshow(WINDOW_TITLE, np.hstack((L, R)))

#             # Create matrix image
#             A = np.concatenate((frame_copy["left"], frame_copy["right"]), axis=1)
#             B = np.concatenate((center_undistorted["left"], center_undistorted["right"]), axis=1)
#             C = np.concatenate((A, B), axis=0)
#             cv2.imshow(WINDOW_TITLE, C)


#         # Actual display sequencing 
#         key = cv2.waitKey(1)
#         if key == ord('q') or cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
#             break

# finally:
#     pipe.stop()










import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.fisheye, 1)
config.enable_stream(rs.stream.fisheye, 2)       
config.enable_stream(rs.stream.pose)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        f1 = frames.get_fisheye_frame(1)
        f2 = frames.get_fisheye_frame(2)
        pose = frames.get_pose_frame()
        
        if not f1 or not f2 or not pose:
            continue

        img1 = np.asanyarray(f1.get_data())
        img2 = np.asanyarray(f2.get_data())
        data = pose.get_pose_data()

        # Stack both images horizontally
        images = np.hstack((img1, img2))
        print(data.angular_velocity)

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        key = cv2.waitKey(1)

        # Exit conditions
        if key != -1:
            if key & 0xFF == ord('q'):
                break

finally:
    # Stop streaming
    pipeline.stop()
    