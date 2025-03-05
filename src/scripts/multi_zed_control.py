#!/usr/bin/env python3

import rospy
import os
import cv2
import pyzed.sl as sl
import numpy as np
import threading

from sensor_msgs.msg import Joy, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

stop_running = False

class ZedCamera(object):
    """Class to handle ZED Camera Operations using Logitech F710 Controller and ROS"""

    def __init__(self, camera_serial, lock):
        """Initialize the ZED Camera object with a camera ID"""
        super(ZedCamera, self).__init__()
        self.camera_serial = camera_serial # Identifier for the camera

        # Initialize button mapping
        self.capture_depth_map_button = 2 # X
        self.capture_depth_video_button = 1 # B
        self.toggle_camera_button = 7 # Start
        self.shutdown_button = 6 # Back


        #Lock for threads
        self.lock = lock

        #Flag for shutdown
        #self.is_running = True

        # Initialize zed object
        self.zed = None
        self.init_params = None

        # Initialize variables
        self.capture_depth_map = False
        self.continuous_capture = False
        

        # Initialize save directory
        self.save_directory = os.path.join('/media/jetsond/9F4F-28D0', f"data_camera_{camera_serial}")

        if not os.path.isdir(self.save_directory):
            rospy.loginfo("Creating save Directory")
            os.makedirs(self.save_directory)

        
        
        # Initialize joy message subscribers
        rospy.Subscriber('/joy_teleop/joy', Joy, self.handle_joy_message, queue_size=3, buff_size=2**16)

        # Initialize stream publisher
        self.image_publisher = rospy.Publisher('/zed_left_camera', Image, queue_size=10)
        
        #self.lock.acquire()
        # Initialize params
        self.initialize_parameters()
        #self.lock.release()

        # Define sleep rate
        self.rate = rospy.Rate(10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()

        # Data Storage for Velocity Messages
        self.twist_data_file = os.path.join(self.save_directory, "twist_data.csv")
        with open(self.twist_data_file, 'w') as f:
            f.write("timestamp,x_linear,z_angular\n")  # CSV header

        # Data Storage for Pose data
        self.pose_data_file = os.path.join(self.save_directory, "pose_data.csv")
        with open(self.pose_data_file, 'w') as f:
            f.write("zed_timestamp,pose_timestamp,tx,ty,tz,ox,oy,oz,ow\n")  # CSV header

        # Data Storage for IMU Sensor Data
        #self.imu_data_file = os.path.join(self.save_directory, "imu_data.csv")
        #with open(self.imu_data_file, 'w') as f:
            #f.write("timestamp,quarternion,imu_linear_acceleration,imu_angular_velocity\n")  # CSV header

    def handle_twist_message(self, twist_msg):
        
        if self.zed:    
            timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_milliseconds()
            x_linear = twist_msg.linear.x
            z_angular = twist_msg.angular.z
            # Write to CSV
            with open(self.twist_data_file, "a") as f:
                f.write(f"{timestamp},{x_linear},{z_angular}\n")

        else:
            rospy.loginfo("Cannot record Twist data: Camera is not initialized!")
        
        
    def handle_joy_message(self, joy_msg):
        # Massive if statement to handle joy mesages
        if joy_msg.buttons[self.capture_depth_map_button]:
            self.capture_depth_map ^= True
            rospy.sleep(1.)
        elif joy_msg.buttons[self.capture_depth_video_button]:
            if self.continuous_capture:
                self.continuous_capture ^= True
                rospy.sleep(1.)
            else:
                for i in range(2, -1, -1):
                    rospy.loginfo(f"Toggling continuous_capture in: {i}")
                    rospy.sleep(1.)
                self.continuous_capture ^= True
                
            rospy.loginfo(f"Continuous capture: {self.continuous_capture}")

        elif joy_msg.buttons[self.toggle_camera_button]:
            self.toggle_camera()
            # Subscribe to cmd_vel to record linear and angular velocity commands
            rospy.Subscriber("/joy_teleop/cmd_vel", Twist, self.handle_twist_message, queue_size = 10)

        elif joy_msg.buttons[self.shutdown_button]:
            global stop_running 
            stop_running = True

    
    def grab_frame(self, 
                   image_mat, 
                   depth_mat, 
                   pose_data, 
                   mirror_mat):
        
        # Check if camera was initialized
        if self.zed is None:
            rospy.loginfo("Camera is not initialized!")
            return  # Exit the function if camera is not initialized
        


        # Grab frame
        err = self.zed.grab(self.runtime_parameters)

        if err == sl.ERROR_CODE.SUCCESS:
            rospy.loginfo(f"Succesfully grabbed frame for {self.camera_serial}!")
            
            # Take image and matching depth map
            rospy.loginfo(f"Got image on {self.camera_serial}!")
            self.zed.retrieve_image(image_mat, sl.VIEW.LEFT)
            
            # Retrieve depth map. Depth is aligned on the left image
            rospy.loginfo(f"Got depth map on {self.camera_serial}!")
            self.zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
            
            # Retrieve Point Cloud
            #rospy.loginfo("Got point cloud!")
            #self.zed.retrieve_measure(point_cloud_mat, sl.MEASURE.XYZRGBA)

            # Get Sensor Data
            #rospy.loginfo("Got sensor data!")
            #self.zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT)

            # Get the pose of the camera relative to the world frame
            
            self.zed.get_position(pose_data, sl.REFERENCE_FRAME.WORLD)
            rospy.loginfo(f"Got pose data on {self.camera_serial}!")
            #if state == sl.ERROR_CODE.SUCCESS:
                #rospy.loginfo("Got pose data!")
                #py_translation = sl.Translation()
                #py_orientation = sl.Orientation()

            # Get time stamp for file name
            timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)

            # Get time stamp for pose
            pose_timestamp = pose_data.timestamp.get_milliseconds()
            
            # Get Depth data and save to file
            image_data = image_mat.get_data()
            depth_data = depth_mat.get_data()
            #point_cloud_data = point_cloud_mat.get_data()
            #point_cloud_data.dot(mirror_mat)

            #if self.stream_video:
                #self.publish_image_message(image_data)

            # Save data
            rospy.loginfo(f"Saving data on {self.camera_serial}...")
            self.save_data(image_data, depth_data, pose_data, timestamp, pose_timestamp)

        else:
            rospy.loginfo(f"Error grabbing frame in {self.camera_serial}: {err}")    
            #self.toggle_camera()

    def publish_image_message(self, image_data):

        # Publish image
        image_data = cv2.resize(image_data, None, fy=0.3, fx=0.3) 
        
        # Transform image to message
        img_msg = self.bridge.cv2_to_imgmsg(image_data, "bgra8")
        
        # Publish image
        self.image_publisher.publish(img_msg)

    
    def save_data(self, image_data, depth_data, pose_data, timestamp, pose_timestamp):

        timestamp_ms = timestamp.get_milliseconds()


        directory = os.path.join(self.save_directory, f"{timestamp_ms}")
        os.makedirs(directory)
        
        # Save Image
        file_name = f"zed_image_left_{timestamp_ms}.jpg"
        cv2.imwrite(os.path.join(directory, file_name), image_data)
        
        # Save depth map
        file_name = f"depth_map_{timestamp_ms}.npy"
        np.save(os.path.join(directory, file_name), depth_data)

        # Save depth map
        #file_name = f"point_cloud_{timestamp_ms}.npy"
        #np.save(os.path.join(directory, file_name), point_cloud_data)

        # Save sensors data
        #file_name = f"imu_sensor_data_{timestamp_ms}.txt"
        #quaternion = sensors_data.get_imu_data().get_pose().get_orientation().get()
        #linear_acceleration = sensors_data.get_imu_data().get_linear_acceleration()
        #angular_velocity = sensors_data.get_imu_data().get_angular_velocity()

        #with open (os.path.join(directory, file_name), 'w') as f:
            #f.write(f"{quaternion}, {linear_acceleration}, {angular_velocity}")

        #with open(self.imu_data_file, "a") as f:
            #f.write(f"{timestamp_ms},{quaternion}, {linear_acceleration}, {angular_velocity}\n")

        # Save pose data
        py_translation = sl.Translation()
        tx = round(pose_data.get_translation(py_translation).get()[0], 3)
        ty = round(pose_data.get_translation(py_translation).get()[1], 3)
        tz = round(pose_data.get_translation(py_translation).get()[2], 3)

        py_orientation = sl.Orientation()
        ox = round(pose_data.get_orientation(py_orientation).get()[0], 3)
        oy = round(pose_data.get_orientation(py_orientation).get()[1], 3)
        oz = round(pose_data.get_orientation(py_orientation).get()[2], 3)
        ow = round(pose_data.get_orientation(py_orientation).get()[3], 3)

        file_name = f"pose_data_{timestamp_ms}.txt"
        with open (os.path.join(directory, file_name), 'w') as f:
            f.write(f"Zed timestamp: {timestamp_ms}, Pose timestamp: {pose_timestamp}, {tx},{ty},{tz},{ox},{oy},{oz},{ow}\n")
        

        # Write to CSV
        with open(self.pose_data_file, "a") as f:
            f.write(f"{timestamp_ms},{pose_timestamp},{tx},{ty},{tz},{ox},{oy},{oz},{ow}\n")

        rospy.loginfo(f"Data saved on {self.camera_serial}!")
        
    def toggle_camera(self):
        
        # Initialize zed object if it doesn't exist
        # Otherwise, close the camera
        if not self.zed:
            rospy.loginfo(f"Initializing camera {self.camera_serial}!")
            
            # Create a Camera object
            zed = sl.Camera()
            err = zed.open(self.init_params)

            if err != sl.ERROR_CODE.SUCCESS:
                rospy.loginfo(f"Zed cam failed to initialize: {err}")
                self.zed = None  # Ensure zed is None if initialization fails
            else:
                self.zed = zed            
                rospy.loginfo(f"Camera {self.camera_serial} initialized!")

            # Enable positional tracking with default parameters
            tracking_parameters = sl.PositionalTrackingParameters()
            err = zed.enable_positional_tracking(tracking_parameters)

            if err != sl.ERROR_CODE.SUCCESS:
                rospy.loginfo(f"Positional Tracking failed to initialize: {err}")
            else:
                rospy.loginfo(f"Positional tracking enabled on {self.camera_serial}")
            
            rospy.loginfo(f"Setting Runtime Parameters for {self.camera_serial}")
            
            #self.lock.acquire()
            # Create and set RuntimeParameters after opening the camera
            self.runtime_parameters = sl.RuntimeParameters()
            self.runtime_parameters.enable_fill_mode = True  # Replace the old sensing mode
            self.runtime_parameters.confidence_threshold = 95 # Changed from 100 to 95
            rospy.loginfo(f"Runtime parameters for {self.camera_serial} set!")
            #self.lock.release()
            
            # Setting the depth confidence parameters
            #
            #self.runtime_parameters.textureness_confidence_threshold = 100  # No more textureness_confidence_threshold in the API

        else:
            rospy.loginfo(f"Closing camera {self.camera_serial}!")
            
            self.zed.close()
            self.zed = None

            rospy.loginfo(f"Camera {self.camera_serial} closed!")

    def initialize_parameters(self):

        rospy.loginfo(f"Setting InitParams on {self.camera_serial}...")

        self.init_params = sl.InitParameters()
        self.init_params.set_from_serial_number(self.camera_serial) # Initializes specific camera based on camera serial
        self.init_params.depth_mode = sl.DEPTH_MODE.QUALITY  # Use PERFORMANCE depth mode
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP # Use a right-handed Y-up coordinate system
        self.init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
        self.init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode
        self.init_params.camera_fps = 24  # Set fps at 24

       
        rospy.loginfo(f"InitParams on {self.camera_serial} set!")

    def toggle_continuous_capture(self):
        self.continuous_capture ^= True
        rospy.loginfo(f"Continuous capture: {self.continuous_capture}")
        rospy.sleep(1.)

    def start(self):
        # Keeps python from exiting until node is stopped
        rospy.loginfo(f"Starting Zed Camera Node for {self.camera_serial}...")
        
        

        # Initialize depth map, image, and point cloud objects
        rospy.loginfo("Initializing depth/image objects")
        image_mat = sl.Mat()
        depth_mat = sl.Mat()
        point_cloud_mat = sl.Mat()
        #sensors_data = sl.SensorsData()
        pose_data = sl.Pose()

        # Prepare for distance calculation
        mirror_ref = sl.Transform()
        mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
        tr_np = mirror_ref.m
	
        
            
        while not rospy.is_shutdown() and not stop_running:
            self.lock.acquire()
            if self.capture_depth_map:
                if not self.zed:
                    rospy.loginfo(f"Capture depth map error {self.camera_serial}: Initialize Camera First!")
                else:
                    rospy.loginfo(f"Capturing depth map/image pair on {self.camera_serial}...")
                    self.grab_frame(image_mat, depth_mat, pose_data, tr_np)
                    self.capture_depth_map = False

            elif self.continuous_capture:
                rospy.loginfo(f"Capturing depth map/image pair on {self.camera_serial}...")
                self.grab_frame(image_mat, depth_mat, pose_data, tr_np)
            self.lock.release()
                          
            self.rate.sleep()
        
        if self.zed:
            self.zed.close()
        rospy.loginfo(f"Zed Camera Script has been stopped on {self.camera_serial}.")


def camera_thread(camera_serial, lock):   
    try:
        """Thread function to run each camera"""
        zed_camera = ZedCamera(camera_serial, lock)
        zed_camera.start()
    except Exception as e:
        global stop_running
        stop_running = True
        rospy.loginfo(f"Error in camera thread with serial {camera_serial}: {e}")

if __name__ == '__main__':
    try:
        # Initialize zed_ros node
        rospy.init_node('zed_ros', anonymous=False)

        # Get the list of connected cameras
        cameras = sl.Camera.get_device_list()
        if not cameras:
            rospy.loginfo("No ZED cameras detected")
            exit()

        lock = threading.Lock()    
        threads = []
        for cam in cameras:
            camera_serial = cam.serial_number
            cam = ZedCamera(camera_serial, lock)
            rospy.loginfo(f"Starting camera with serial number: {camera_serial}")
            thread = threading.Thread(target=cam.start, args=())
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

    except KeyboardInterrupt:
        rospy.loginfo("Shutdown signal received.")
        stop_running = True
        for thread in threads:
            thread.join()



