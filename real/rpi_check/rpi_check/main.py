#!/usr/bin/env python3

# --- TODO ---
# check the tracker node separately
# topics + data
# camera params
# main() ---> GUIDED --> OFFBOARD

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped, Vector3, TransformStamped, Twist
from sensor_msgs.msg import Image, CameraInfo, Imu
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float64
# from mrs_msgs.msg import Float64Stamped, VelocityReference, VelocityReferenceStamped, SpeedTrackerCommand
# from mavros_msgs.msg import Altitude, State, PositionTarget
from detection_msgs.msg import PublishData
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import tf2_ros
from tf2_geometry_msgs import do_transform_point

import numpy as np
import math
from cv_bridge import CvBridge

import time
# import tf_transformations
# import pyrealsense2 as rs2

from px4_msgs.msg import VehicleLocalPosition, SensorGps, VehicleAttitude, TrajectorySetpoint, VehicleGlobalPosition, VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

class BoundingBox2D:
    def __init__(self, center_x=0, center_y=0, size_x=0, size_y=0):
        self.center = Point(center_x, center_y)
        self.size_x = size_x
        self.size_y = size_y

class DetectedObject:
    def __init__(self, id, center_x, center_y, size_x, size_y):
        self.id = id
        self.center_x = center_x
        self.center_y = center_y
        self.size_x = size_x
        self.size_y = size_y
        
    def update(self, id, center_x, center_y, size_x, size_y):
        self.id = id
        self.center_x = center_x
        self.center_y = center_y
        self.size_x = size_x
        self.size_y = size_y

class LostCounter:
    def __init__(self, counting=False):
        self.counter = 10
        self.counting = counting
        self.lost = False
        # self.get_logger().info("Lost Counter Initialized")
        
    def update(self):
        if self.counter > 0:
            self.counter -= 1
            return True
        elif self.counter == 0:
            self.counting = False
            self.lost = True
            return False
            
    def check_counting(self):
        return self.counting
        
    def is_lost(self):
        return self.lost

class FollowTarget(Node):
    def __init__(self):
        super().__init__('follow_target')
        self.get_logger().info("Node started")
        
        # change 1 - ACCORDING TO PERSON HEIGHT
        self.constant = 1000
        self.horizontal_distance = 0

        # Kalman Filters initialization (same as original)
        self.KF = KalmanFilter(dim_x=6, dim_z=4)
        self.KF.H = np.array([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])
        self.KF.P *= 5
        self.KF.R = np.array([[2, 0, 0, 0],
                            [0, 2, 0, 0],
                            [0, 0, 20, 0],
                            [0, 0, 0, 20]])
        self.KF.Q = np.array([[2, 0, 0, 0, 0, 0],
                            [0, 2, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])

        self.KF_target = KalmanFilter(dim_x=4, dim_z=2)
        self.KF_target.H = np.array([[1, 0, 0, 0],
                                    [0, 1, 0, 0]])
        self.KF_target.P *= 5
        self.KF_target.R = np.array([[100, 0],
                                    [0, 100]])
        self.KF_target.Q = np.array([[1, 0, 0, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])

        self.intrinsics = None
        self.target_position = Point()
        self.altitude = 0
        # self.depth = 0
        self.target_detected = False
        self.first_detection = False
        self.bridge = CvBridge()
        self.target_lost = False
        self.target_id = 0
        self.adjustment_period = 10
        self.lost_counter = LostCounter()
        self.velocity_msg = Twist()

        # Initialize variables
        self.previous_error = 0.0
        self.integral = 0
        self.heading = 0
        self.heading_rate = 0

        # Define Gains for controller
        self.kh = 0.3
        self.kp_altitude = 0.6
        self.kp_vel = 0.35
        self.kd_vel = 0.0
        self.ki_vel = 0.01
        self.alpha = 0.10  # low_pass filter
        self.filtered_value = None

        # Parameters for the Slew Rate limiter
        self.max_rate = 0.5
        self.last_update_time = time.time()
        self.last_update_time_KF = time.time()
        self.current_value = 0.0
        # self.not_using_depth = True

        
        # --- check for these params
        self.fx = 615
        self.fy = 615
        self.ppx = 318.76
        self.ppy = 247.39
        self.camera_width = 640
        self.camera_height = 480
        
        self.distance = 0
        # self.previous_depth = 0
        # self.pitch_camera = np.pi/6
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.uav_pose = Point()
        self.recent_distances = []
        self.detected_objects = []
        self.current_state = 0   # or None
        self.bool = True
        self.publish_data = PublishData()

        # --- check for these subscriptions + data format coming from them
        # Subscribers

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.vehicle_status_subscriber = self.create_subscription(
          VehicleStatus, '/fmu/out/vehicle_status', self.check_mode, qos_profile)

        # self.sub_state = self.create_subscription(
        #     State, "/mavros/state", self.check_mode, 10)

        
        self.altitude_sub = self.create_subscription(
            VehicleGlobalPosition, 
            '/fmu/out/vehicle_global_position', 
            self.get_altitude, 
            qos_profile=qos_profile)
        
        # self.altitude_sub = self.create_subscription(
        #     Float64, '/mavros/global_position/rel_alt', self.get_altitude, 10)
        
        # CHANGE 2 TOPIC
        self.target_sub = self.create_subscription(
            Detection2DArray, '/detections', self.target_callback, 10)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        # Publishers
        # self.pub_vel = self.create_publisher(
        #     Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self.publisher = self.create_publisher(
            PublishData, '/rosbag_data', 10)
        self.pub_rate = self.create_publisher(
            Float64, '/rate_rate', 10)

    # nav state = 4 (OFFBOARD) - https://github.com/PX4/PX4-user_guide/blob/main/en/ros2/px4_ros2_control_interface.md
    def check_mode(self, state):
        # self.current_state = state
        self.current_state = state.nav_state  
        
    # def create_velocity_msg(self, Vx, Vy, Vz, yaw_rate):
    #     velocity_msg = Twist()                
    #     velocity_msg.linear.x = Vx
    #     velocity_msg.linear.z = -Vz
    #     velocity_msg.angular.z = yaw_rate
    #     self.target_detected = True
    #     return velocity_msg
    
    # -- yaw rate??   -->> self.publisher = self.create_publisher(
            # VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', 10)

    def create_velocity_msg(self, Vx, Vy, Vz, yaw_rate):
        msg = TrajectorySetpoint()
        msg.velocity[0] = Vx
        msg.velocity[2] = -Vz
        msg.yawspeed = float(yaw_rate)        
        self.target_detected = True

        print(f"Setpoint NED: Vx={Vx}, Vy={Vy}, Vz={Vz}")
        return msg
    


    # tl - top left 
    # br - bottom right
    def calculate_iou(self, box1, box2):
        x1, y1, w1, h1 = box1
        x2, y2, w2, h2 = box2
        half_w1, half_h1 = w1 / 2, h1 / 2
        half_w2, half_h2 = w2 / 2, h2 / 2
        x1_tl, y1_tl = x1 - half_w1, y1 - half_h1
        x1_br, y1_br = x1 + half_w1, y1 + half_h1
        x2_tl, y2_tl = x2 - half_w2, y2 - half_h2
        x2_br, y2_br = x2 + half_w2, y2 + half_h2
        x_inter = max(x1_tl, x2_tl)
        y_inter = max(y1_tl, y2_tl)
        x2_inter = min(x1_br, x2_br)
        y2_inter = min(y1_br, y2_br)
        intersection_area = max(0, x2_inter - x_inter) * max(0, y2_inter - y_inter)     #RelU
        area1 = w1 * h1
        area2 = w2 * h2
        union_area = area1 + area2 - intersection_area
        iou = intersection_area / union_area if union_area > 0 else 0
        return iou

    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def get_altitude(self, altitude_msg):
        # self.altitude = altitude_msg.data
        self.altitude = altitude_msg.alt

    # def get_depth(self, center_x, center_y):
    #     window_size = 15
    #     depth_matrix = None
    #     if hasattr(self, 'depth_image'):
    #         depth_matrix = self.depth_image[
    #             int(center_y) - window_size : int(center_y) + window_size + 1,
    #             int(center_x) - window_size : int(center_x) + window_size + 1,
    #         ]
    #     if depth_matrix is None:
    #         return 0
            
    #     depth_matrix_copy = np.copy(depth_matrix)
    #     depth_matrix_copy[np.isnan(depth_matrix_copy)] = -1.0
    #     average_depth = np.mean(depth_matrix_copy[depth_matrix_copy != -1.0])
    #     return average_depth/1000
    
    # new_value = vel PID
    def rate_limiter(self, new_value):
        current_time = time.time()
        time_elapsed = current_time - self.last_update_time
        self.last_update_time = current_time

        if time_elapsed == 0:
            return self.current_value

        # acceleration
        rate_of_change = (new_value - self.current_value) / time_elapsed

        if self.bool:
            self.current_value = 0
            self.bool = False
            return self.current_value

        if abs(rate_of_change) <= self.max_rate:
            self.current_value = new_value
            self.last_update_time = current_time
        else:
            sign = 1 if new_value > self.current_value else -1
            # acceleration * time --> vel
            self.current_value += sign * self.max_rate * time_elapsed
            self.last_update_time = current_time

        return self.current_value
    
    # new_value = dist
    def low_pass_filter(self, new_value):
        if self.filtered_value is None:
            self.filtered_value = new_value
        else:
            self.filtered_value = self.alpha * new_value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value
    
    # def depth_callback(self, msg):
    #     try:
    #         self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
    #     except Exception as e:
    #         self.get_logger().error(f"Error converting depth image: {str(e)}")

    def camera_info_callback(self, cameraInfo):
        # self.fx = cameraInfo.k[0]
        # self.fy = cameraInfo.k[4]
        # self.ppx = cameraInfo.k[2]
        # self.ppy = cameraInfo.k[5]
        # self.K_intrinsics = np.array(cameraInfo.k).reshape(3, 3)
        
        self.ppx=545.010339
        self.ppy=263.934326

        # try:
        #     if self.intrinsics:
        #         return
        #     self.intrinsics = rs2.intrinsics()
        #     self.intrinsics.width = cameraInfo.width
        #     self.intrinsics.height = cameraInfo.height
        #     self.intrinsics.ppx = cameraInfo.k[2]
        #     self.intrinsics.ppy = cameraInfo.k[5]
        #     self.intrinsics.fx = cameraInfo.k[0]
        #     self.intrinsics.fy = cameraInfo.k[4]
        #     if cameraInfo.distortion_model == 'plumb_bob':
        #         self.intrinsics.model = rs2.distortion.brown_conrady
        #     elif cameraInfo.distortion_model == 'equidistant':
        #         self.intrinsics.model = rs2.distortion.kannala_brandt4
        #     self.intrinsics.coeffs = [i for i in cameraInfo.d]
        # except Exception as e:
        #     self.get_logger().error(f"Error setting camera intrinsics: {str(e)}")

    def velocity_controller(self, error):
        derivative = error - self.previous_error
        self.integral += error 
        
        if self.integral > 2:
            self.integral = 2
        elif self.integral < -2:
            self.integral = -2

        self.previous_error = error            
        control_output = (error * self.kp_vel + self.kd_vel * derivative + self.ki_vel * self.integral)

        if control_output > 4:
            control_output = 4
            self.integral -= error
        elif control_output < -4:
            control_output = -4
            self.integral -= error

        control_output = self.rate_limiter(control_output)
        
        # Vx = horizontal
        if self.horizontal_distance < 6 and control_output > 0:
            control_output = control_output/2
        elif self.horizontal_distance < 4 and control_output > 0:
            control_output = 0

        self.publish_data.control_output = control_output
        # -----------------------------

        # Vz = vertical dist
        normalized_y = ((self.KF.x[1] / self.ppy) - 1)
        
        Vz = self.kp_altitude*normalized_y

        # height = 8 (max)    3.5 (min)  
        if self.altitude > 8 and Vz < 0:
            Vz = 0
        elif self.altitude < 3.5 and Vz > 0:
            Vz = 0
        # -----------------------------------
        
        velocity_msg = self.create_velocity_msg(control_output, 0, Vz, self.heading_rate)
        self.publish_data.vz = Vz
        return velocity_msg

    def update_objects(self, data, target_found):
        target_bbox = BoundingBox2D()  
        best_match_value = 0
        best_detection = None
    
        if not target_found:
            my_target = next((obj for obj in self.detected_objects if obj.id == self.target_id), None)
            my_target_position_increased = (my_target.center_x, my_target.center_y, my_target.size_x + 150, my_target.size_y + 100)

            target_bbox.center.x = self.KF.x[0]
            target_bbox.center.y = self.KF.x[1]
            target_bbox.size_x = self.KF.x[4]
            target_bbox.size_y = self.KF.x[5]

            if not self.target_lost and not self.lost_counter.check_counting():
                self.target_lost = True
                self.lost_counter.counting = True
                self.lost_counter.counter = 10
                self.get_logger().info(f"START LOST COUNTER, CHECK = {self.lost_counter.check_counting()}")
            else:
                self.lost_counter.update()
        else:
            self.lost_counter.counting = False
            self.lost_counter.counter = 10
            self.lost_counter.lost = False
            self.get_logger().info(f"target found, RESET COUNTER, CHECK = {self.lost_counter.check_counting()}")

        for detection in data.detections:
            bounding_box = detection.bbox
            id = detection.results[0].id
            center_x = bounding_box.center.x
            center_y = bounding_box.center.y
            size_x = bounding_box.size_x
            size_y = bounding_box.size_y

            existing_object = next((obj for obj in self.detected_objects if obj.id == id), None)

            if existing_object:
                existing_object.update(id, center_x, center_y, size_x, size_y)
                if existing_object.id == self.target_id:    
                    target_bbox = detection.bbox
                    self.target_lost = False
            else:
                if target_found and id != 0:
                    new_object = DetectedObject(id, center_x, center_y, size_x, size_y)
                    self.detected_objects.append(new_object)
                elif id != 0:
                    iou = self.calculate_iou((center_x, center_y, size_x + 150, size_y + 100), my_target_position_increased)
                    new_object = DetectedObject(id, center_x, center_y, size_x, size_y)
                    self.detected_objects.append(new_object)
                    if iou > best_match_value:
                        best_match_value = iou
                        best_detection = detection
                        best_id = id

        if not target_found and best_detection is not None:
            if best_match_value > 0.1:
                delete_id = self.target_id
                self.target_id = best_id
                my_target.update(best_id, center_x, center_y, size_x, size_y)
                delete_obj = next((obj for obj in self.detected_objects if obj.id == delete_id), None)
                if delete_obj is not None:    
                    delete_obj.update(999, 0, 0, 0, 0)    
                target_bbox = detection.bbox
                target_found = 1
                self.target_lost = False
                if self.lost_counter.is_lost():
                    self.filtered_value = None
                    self.bool = True
                    self.lost_counter.lost = False
                    self.adjustment_period = 10
                    self.get_logger().info("RESET VALUES")
                self.lost_counter.counting = False
                self.get_logger().info(f"TARGET REDETECTED, RESET COUNTER {self.lost_counter.check_counting()} {id}")
                self.lost_counter.counter = 10
            else:
                target_found = 0
                self.get_logger().info(f"No suitable match found. {best_match_value}")                

        return target_bbox

    def target_callback(self, data):
        current_time = time.time()
        dt = current_time - self.last_update_time_KF
        self.last_update_time_KF = current_time
        self.KF.F = np.array([[1, 0, dt, 0, 0, 0],
                            [0, 1, 0, dt, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])
        self.KF.predict()

        self.KF_target.F = np.array([[1, 0, dt, 0],
                                    [0, 1, 0, dt],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])
        self.KF_target.predict()

        if not (data.detections or self.target_lost):
            self.KF.update(None)
            self.KF_target.update(None)

        if not data.detections and self.first_detection:
            self.first_detection = True
            self.target_lost = True

        if data.detections or self.target_lost:
            if not self.first_detection:
                self.target_id = data.detections[0].results[0].id
                self.get_logger().info(f"target detected id first frame: {self.target_id}")
                self.first_detection = True
                new_object = DetectedObject(
                    self.target_id,
                    data.detections[0].bbox.center.x,
                    data.detections[0].bbox.center.y,
                    data.detections[0].bbox.size_x,
                    data.detections[0].bbox.size_y
                )
                self.detected_objects.append(new_object)

            target_found = 0
            for detection in data.detections:
                if detection.results[0].id == self.target_id:
                    target_found = 1
                    
            self.publish_data.target_lost = target_found
                    
            bounding_box = self.update_objects(data, target_found)    

            if bounding_box is not None:
                center_x = bounding_box.center.x
                center_y = bounding_box.center.y
                size_x = bounding_box.size_x
                size_y = bounding_box.size_y
                
                if not self.target_lost:
                    self.KF.update([center_x, center_y, size_x, size_y])
                self.publish_data.distance_cnst = self.KF.x[0]
                self.publish_data.distance_singer = self.KF.x[1]

                normalized_x = (self.KF.x[0] / self.ppx) - 1    #for yaw
                normalized_y = (self.KF.x[1] / self.ppy) - 1    #for altitude/Vz
                size_y = self.KF.x[5]

                self.heading = np.deg2rad(self.heading)
                self.heading_rate = -self.kh * normalized_x
                
                self.publish_data.hdg_rate = -self.kh * normalized_x
                self.publish_data.id = self.target_id

                Vz = self.kp_altitude * normalized_y
                self.publish_data.error_altitude = -Vz

                # d = in fig 4 = slop-like dist
                if size_y != 0:
                    distance = self.constant / size_y
                else:
                    distance = 10
                
                # ---------------------------------------------
                # if 1:  # target_found:
                #     depth = self.get_depth(320.5, 440.5)
                #     x = (center_x - self.ppx) / self.fx
                #     y = (center_y - self.ppy) / self.fy
                #     dist = np.sqrt(x**2 + y**2 + depth**2)
                #     self.publish_data.depth = depth
                #     self.publish_data.depth_distance = dist
                # else:
                #     depth = 0

                # self.not_using_depth = True
                
                # if depth > 3 and depth < 7:
                #     value = self.low_pass_filter(dist)
                #     self.not_using_depth = False
                #     self.get_logger().info("using depth")                
                # else:
                #     value = self.low_pass_filter(distance)
                # ---------------------------------------------
                
                value = self.low_pass_filter(distance)

                estimated_distance = value
                self.horizontal_distance = np.sqrt(estimated_distance**2 - self.altitude**2)

                # d_desired = 8m
                error = estimated_distance - 8
                
                self.publish_data.horizontal_distance = self.horizontal_distance
                self.publish_data.altitude = self.altitude
                self.publish_data.estimated_distance = estimated_distance
                self.publish_data.use_depth = False
                self.publish_data.center_x = center_x
                self.publish_data.center_y = center_y
                self.publish_data.controller_error = error

                # adjust = first det + reacquired = to center the target (in img frame)
                if self.adjustment_period > 0:
                    if not self.target_lost:
                        self.adjustment_period -= 1    
                    
                    # as mentioned in the thesis - for center-AIMing = vertical+yaw
                    # 0.1 * yaw -- a little rotate to see the target
                    self.get_logger().info(f"ADJUSTING {self.target_id}")                
                    self.velocity_msg = self.create_velocity_msg(0, 0, Vz, 0.1 * self.heading_rate)
                    self.publish_data.mode = 0    

                elif self.target_lost and not self.lost_counter.check_counting() and self.lost_counter.is_lost():
                    self.get_logger().info(f"---- LOST ---- {self.target_id} {self.lost_counter.counter}")        
                    control_output = self.rate_limiter(0)    # 0 vel = stop moving drone 
                    self.publish_data.control_output = control_output    
                    self.velocity_msg = self.create_velocity_msg(control_output, 0, 0, 0)
                    self.publish_data.mode = 3
                else:
                    # missing - slow down + search (with prev KF prediction)
                    if self.target_lost:
                        self.velocity_msg = self.velocity_controller(error - 1)     #err-1 = prev err/dist
                        self.lost_altitude = self.altitude
                        self.get_logger().info(f"---- MISSING ---- {self.target_id} {self.lost_counter.counter} {error}")
                        self.publish_data.mode = 2
                    else:       #adjustment=0 --> full PID -- keep following
                        self.velocity_msg = self.velocity_controller(error)
                        self.get_logger().info(f"---- REGULAR ---- {self.target_id}")
                        self.publish_data.mode = 1

def main(args=None):
    rclpy.init(args=args)
    follower = FollowTarget()
    
    try:
        while rclpy.ok():
            follower.publish_data.target_detected = follower.target_detected
            follower.publisher.publish(follower.publish_data)
            follower.pub_rate.publish(Float64(data=1.0))


            # if follower.target_detected and follower.current_state.mode == "GUIDED/OFFBOARD":
            if follower.target_detected and follower.current_state == 4:
                follower.pub_vel.publish(follower.velocity_msg)
            elif follower.current_state != 4:
                follower.first_detection = False
                follower.filtered_value = None
                follower.bool = True
                follower.lost_counter.lost = False
                follower.adjustment_period = 10
            else:
                follower.velocity_msg = Twist()     #vel=0
                
            rclpy.spin_once(follower)
            
    except KeyboardInterrupt:
        pass
    finally:
        follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()