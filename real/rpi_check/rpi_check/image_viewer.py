#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class SimpleImageViewer(Node):
    def __init__(self):
        super().__init__('simple_image_viewer')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/annotated_image',  # Change to your image topic if different
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('Simple Image Viewer started')
        
    def image_callback(self, msg):
        try:
            # Convert ROS CompressedImage message to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            
            # Display the image
            cv2.imshow('Hailo Detections', cv_image)
            cv2.waitKey(1)  # Refresh display
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    viewer = SimpleImageViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()