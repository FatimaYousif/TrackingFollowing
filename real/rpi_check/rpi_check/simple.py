#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from vision_msgs.msg import Detection2DArray
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleAttitude


class FollowTarget(Node):
    def __init__(self):
        super().__init__('follow_target')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', 
            qos_profile
        )
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode', 
            qos_profile
        )

        self.create_subscription(
            Detection2DArray,
            '/detections',
            self.target_callback,
            10
        )


        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.odometry_callback,
            qos_profile
        )

        self.yaw_kp = 0.5
        self.dist_kp = 1.5
        self.height_kp = -1.5
        self.current_yaw = 0.0


    def odometry_callback(self, msg: VehicleAttitude):
            q = msg.q  # [w, x, y, z]
            w, x, y, z = q[0], q[1], q[2], q[3]
            # Yaw extraction from quaternion (FRD to NED)
            siny_cosp = 2.0 * (w * z + x * y)
            cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def vel_callback(self, yaw_rate, vbx, vbz, detections):
          if detections == True:      
            traj_msg = TrajectorySetpoint()
            off_msg = OffboardControlMode()
            traj_msg.position[0] = float('nan')
            traj_msg.position[1] = float('nan')
            traj_msg.position[2] = float('nan')
            traj_msg.yaw = float('nan')
            traj_msg.yawspeed = yaw_rate

            # Convert body frame velocity to NED frame
            ned_vx = vbx * math.cos(self.current_yaw)
            ned_vy = vbx * math.sin(self.current_yaw)

            traj_msg.velocity[0] = ned_vx
            traj_msg.velocity[1] = ned_vy
            traj_msg.velocity[2] = vbz

            off_msg.position = False
            off_msg.velocity = True
            off_msg.acceleration = False
            off_msg.attitude = False
            off_msg.body_rate = True
            off_msg.thrust_and_torque = False
            off_msg.direct_actuator = False
            print("ADJUSTED!!!")
          else:
            self.get_logger().info("No detections")
            traj_msg = TrajectorySetpoint()
            traj_msg.position[0] = float('nan')
            traj_msg.position[1] = float('nan')
            traj_msg.position[2] = -10.0
            traj_msg.yaw = float('nan')
            traj_msg.yawspeed = 0.2

            off_msg = OffboardControlMode()
            off_msg.position = True
            off_msg.velocity = False
            off_msg.acceleration = False
            off_msg.attitude = False
            off_msg.body_rate = True
            off_msg.thrust_and_torque = False
            off_msg.direct_actuator = False
            self.trajectory_setpoint_pub.publish(traj_msg)
            self.offboard_control_mode_pub.publish(off_msg)
    
          self.trajectory_setpoint_pub.publish(traj_msg)
          self.offboard_control_mode_pub.publish(off_msg)



    def target_callback(self, data: Detection2DArray):

      
        if len(data.detections) > 0:
            # self.get_logger().info(f"First detection: {data.detections[0].bbox.center.position.x}, {data.detections[0].bbox.center.position.y}, {data.detections[0].bbox.size_x}, {data.detections[0].bbox.size_y}")
            x = data.detections[0].bbox.center.position.x
            y = data.detections[0].bbox.center.position.y
            size_y = data.detections[0].bbox.size_y

            print(size_y)
            vbx = self.dist_kp * (220 - size_y) / 220   #error /control_op
            vbz = self.height_kp * (540 - y) / 540   #error /control_op
            yaw_rate = -self.yaw_kp * (960 - x) / 960   #target_hdg

            detections = True
            self.vel_callback(yaw_rate, vbx, vbz, detections)          

        else:
            detections=False
            self.vel_callback(0, 0, 0, detections)          

          


def main(args=None):
    rclpy.init(args=args)
    follower = FollowTarget()
    
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()