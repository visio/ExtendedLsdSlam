# Install package 
# sudo apt-get install ros-indigo-

# Get camera info
# v4l2-ctl --list-formats-ext

# Camera calibration 
# rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.034 image:=/usb_cam/image_raw camera:=

# Check camera output
# rosrun image_view image_view image:=/usb_cam/image_raw

# *********************************************************
# ********* libuvc_CAM ******************
# Run camera
# rosrun libuvc_camera camera_node _frame_rate:=60.0  _width:=640 _height:=480 _video_mode:="rgb"

# rosrun libuvc_camera camera_node _vendor:=1415 _product=2000 _index:=1 _frame_rate=60.0  _width=640.0 _height=480.0 _video_mode="rgb"

# rosrun libuvc_camera camera_node _vendor="0x1415" _product="0x2000" _frame_rate=60.0  _width=640.0 _height=480.0 _video_mode="rgb"

# ********* uvc_CAMera ******************
# rosrun uvc_camera uvc_camera_node _device:="/dev/video1" _fps:="60" _width:="640" _height:="480" 

# ********* uEYE_CAM ******************
# rosrun ueye camera 

# Run lsd_slam
# rosrun lsd_slam_core live_slam /image:=/image_raw _calib:=/home/sergey/MyLsdSlamProject/CalibrationData/Eye_B50_184559/Matlab/Ps3Eye_640_w.cfg
# rosrun lsd_slam_core live_slam /image:=/image_raw _calib:=/home/sergey/MyLsdSlamProject/CalibrationData/OpenCV_example_calib.cfg
# rosrun lsd_slam_core live_slam /image:=/image_raw _calib:=/home/sergey/MyLsdSlamProject/CalibrationData/Eye_B50_184559/Ros/Ps3Eye_640_w_ros.cfg

# ********* USB_CAM *********************
# Run camera
# rosrun usb_cam 	usb_cam_node 	_video_device:="/dev/video2"  	_framerate:="30" 	_image_width:="640" 	_image_height:="480" _pixel_format:="uyvy"

# Run lsd_slam
# rosrun lsd_slam_core live_slam /image:=/usb_cam/image_raw _calib:=/home/sergey/MyLsdSlamProject/CalibrationData/Eye_B50_184559/Matlab/Ps3Eye_640_w.cfg
# rosrun lsd_slam_core live_slam /image:=/usb_cam/image_raw _calib:=/home/sergey/MyLsdSlamProject/CalibrationData/OpenCV_example_calib.cfg
# rosrun lsd_slam_core live_slam /image:=/usb_cam/image_raw _calib:=/home/sergey/MyLsdSlamProject/CalibrationData/Eye_B50_184559/Ros/Ps3Eye_640_w_ros.cfg

# Run image set
# rosrun lsd_slam_core dataset_slam _files:='/home/sergey/MyLsdSlamProject/Projects/TestProject/frames'  _calib:='/home/sergey/MyLsdSlamProject/Projects/ColoredCubesLarge/TestCamera.cfg' _groundTruth:='/home/sergey/MyLsdSlamProject/Projects/TestProject/blenderCoordinates/blenderCameraPositions.txt' _doSlam:=false _hz=20

# Append for save files
#  _saveResults:=true


# ********* Reconfigure *****************
# rosrun rqt_reconfigure rqt_reconfigure




