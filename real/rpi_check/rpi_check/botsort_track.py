#!/usr/bin/env python3
"""
ROS2 node for Hailo object detection and tracking
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image, CompressedImage
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import supervision as sv
import numpy as np
import cv2
import queue
import sys
import os
from typing import Dict, List, Tuple
import threading
from pathlib import Path

# Import BoT-SORT from boxmot
from boxmot import BotSort

# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from .utils import HailoAsyncInference


class HailoDetectionNode(Node):
    def __init__(self):
        super().__init__('hailo_detection_node')
        
        # /home/pi/new_ws/src/rpi_check/resource/yolov5m_wo_spp_60p.hef
        # Declare and get parameters
        self.declare_parameter('model_path', '/home/pi/new_ws/src/rpi_check/resource/yolov8s.hef')
        self.declare_parameter('labels_path', '/home/pi/new_ws/src/rpi_check/resource/coco.txt')
        self.declare_parameter('score_threshold', 0.5)
        self.declare_parameter('reid_weights_path', '/home/pi/new_ws/src/rpi_check/resource/osnet_x0_25_msmt17.pt')  # Empty by default
        
        self.model_path = self.get_parameter('model_path').value
        self.labels_path = self.get_parameter('labels_path').value
        self.score_threshold = self.get_parameter('score_threshold').value
        self.reid_weights_path = self.get_parameter('reid_weights_path').value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Set up publishers and subscribers
        self.detection_pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self.annotated_pub = self.create_publisher(CompressedImage, 'annotated_image', 10)
        # self.marker_pub = self.create_publisher(MarkerArray, 'detection_markers', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Initialize Hailo inference
        self.input_queue = queue.Queue()
        self.output_queue = queue.Queue()
        self.hailo_inference = HailoAsyncInference(
            hef_path=self.model_path,
            input_queue=self.input_queue,
            output_queue=self.output_queue,
        )
        # print("...........", self.model_path)
        self.model_h, self.model_w, _ = self.hailo_inference.get_input_shape()

        # Initialize annotation
        self.box_annotator = sv.RoundBoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()
        
        reid_weights = Path(self.reid_weights_path) if self.reid_weights_path and os.path.exists(self.reid_weights_path) else None

        print("==============================")
        
        # Initialize BoT-SORT tracker instead of ByteTrack
        self.tracker = BotSort(
            track_high_thresh=0.5,  # High confidence threshold
            track_low_thresh=0.1,   # Low confidence threshold
            new_track_thresh=0.6,   # New track threshold
            track_buffer=30,        # Track buffer size
            match_thresh=0.8,       # Matching threshold
            frame_rate=30,           # Frame rate
            reid_weights=reid_weights,      # Path to reid weights (None if not using reid)
            device='cpu',           # Device to run on ('cpu' or 'cuda')
            half=False       
        )

        

        # Load class names
        with open(self.labels_path, "r", encoding="utf-8") as f:
            self.class_names = f.read().splitlines()

        # Start inference thread
        self.inference_thread = threading.Thread(target=self.hailo_inference.run)
        self.inference_thread.start()

    def image_callback(self, msg):
        # Convert ROS Image to CV2
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        video_h, video_w = frame.shape[:2]

        # Rotate 180 degrees
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        # Swap r and b channels, then multiply r by 0.5 to fix the colors
        frame = frame[:, :, ::-1]
        frame[:, :, 0] = frame[:, :, 0] * 0.5

        # Preprocess frame
        preprocessed_frame = self.preprocess_frame(frame, self.model_h, self.model_w, video_h, video_w)

        # Run inference
        self.input_queue.put([preprocessed_frame])
        _, results = self.output_queue.get()

        if len(results) == 1:
            results = results[0]

        # Process detections
        detections = self.extract_detections(results, video_h, video_w, self.score_threshold)
        
        if detections["num_detections"] == 0:
            # No detections found, skip processing
            return

        # Convert detections to BoT-SORT format
        dets_for_tracking = np.column_stack([
            detections["xyxy"],
            detections["confidence"],
            detections["class_id"]
        ])

        # Update tracker with current frame and detections
        tracked_dets = self.tracker.update(dets_for_tracking, frame)

        if len(tracked_dets) == 0:
            return

        # Extract tracking results
        tracked_xyxy = tracked_dets[:, :4]
        tracked_ids = tracked_dets[:, 4].astype(int)
        tracked_class_ids = tracked_dets[:, 5].astype(int)
        tracked_confidences = tracked_dets[:, 6]

        # Create Detection2DArray message
        detection_msg = Detection2DArray()
        detection_msg.header = msg.header

        # Create MarkerArray message
        marker_array = MarkerArray()

        # Convert detections to ROS messages
        for i in range(len(tracked_dets)):
            if tracked_class_ids[i] != 0:  # Only publish class ID 0 (person)
                continue

            det = Detection2D()
            det.bbox.center.position.x = float((tracked_xyxy[i][0] + tracked_xyxy[i][2]) / 2)
            det.bbox.center.position.y = float((tracked_xyxy[i][1] + tracked_xyxy[i][3]) / 2)
            det.bbox.size_x = float(tracked_xyxy[i][2] - tracked_xyxy[i][0])
            det.bbox.size_y = float(tracked_xyxy[i][3] - tracked_xyxy[i][1])
            det.id = str(int(tracked_ids[i]))
            print("Track ID: ", det.id)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(tracked_class_ids[i])
            hyp.hypothesis.score = float(tracked_confidences[i])
            det.results.append(hyp)

            detection_msg.detections.append(det)

            # Create marker for bounding box
            marker = Marker()
            marker.header = msg.header
            marker.ns = "detection_boxes"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.01  # Line width
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Add points to form rectangle
            x1, y1 = float(tracked_xyxy[i][0]), float(tracked_xyxy[i][1])
            x2, y2 = float(tracked_xyxy[i][2]), float(tracked_xyxy[i][3])
            points = [
                (x1, y1, 0.0),
                (x2, y1, 0.0),
                (x2, y2, 0.0),
                (x1, y2, 0.0),
                (x1, y1, 0.0)  # Close the rectangle
            ]
            for x, y, z in points:
                p = Point()
                p.x = x
                p.y = y
                p.z = z
                marker.points.append(p)

            marker_array.markers.append(marker)

        # Publish detections
        self.detection_pub.publish(detection_msg)
        # self.marker_pub.publish(marker_array)

        # Create and publish annotated image
        annotated_frame = self.postprocess_detections(
            frame, tracked_dets, self.class_names,
            self.box_annotator, self.label_annotator
        )
        
        _, jpg_buffer = cv2.imencode('.jpg', annotated_frame)
        annotated_msg = CompressedImage()
        annotated_msg.format = "jpeg"
        annotated_msg.data = jpg_buffer.tobytes()
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

    def preprocess_frame(
        self, frame: np.ndarray, model_h: int, model_w: int, video_h: int, video_w: int
    ) -> np.ndarray:
        if model_h != video_h or model_w != video_w:
            frame = cv2.resize(frame, (model_w, model_h))
        return frame

    def extract_detections(
        self, hailo_output: List[np.ndarray], h: int, w: int, threshold: float = 0.5
    ) -> Dict[str, np.ndarray]:
        xyxy: List[np.ndarray] = []
        confidence: List[float] = []
        class_id: List[int] = []
        num_detections: int = 0

        for i, detections in enumerate(hailo_output):
            if len(detections) == 0:
                continue
            for detection in detections:
                bbox, score = detection[:4], detection[4]

                if score < threshold:
                    continue

                bbox[0], bbox[1], bbox[2], bbox[3] = (
                    bbox[1] * w,
                    bbox[0] * h,
                    bbox[3] * w,
                    bbox[2] * h,
                )

                xyxy.append(bbox)
                confidence.append(score)
                class_id.append(i)
                num_detections += 1

        return {
            "xyxy": np.array(xyxy),
            "confidence": np.array(confidence),
            "class_id": np.array(class_id),
            "num_detections": num_detections,
        }

    def postprocess_detections(
        self, frame: np.ndarray,
        tracked_dets: np.ndarray,
        class_names: List[str],
        box_annotator: sv.RoundBoxAnnotator,
        label_annotator: sv.LabelAnnotator,
    ) -> np.ndarray:
        # Extract components from tracked detections
        xyxy = tracked_dets[:, :4]
        tracker_ids = tracked_dets[:, 4].astype(int)
        class_ids = tracked_dets[:, 5].astype(int)
        confidences = tracked_dets[:, 6]

        # Create supervision detections object
        sv_detections = sv.Detections(
            xyxy=xyxy,
            confidence=confidences,
            class_id=class_ids,
        )
        
        # Add tracker IDs to the detections
        sv_detections.tracker_id = tracker_ids

        labels: List[str] = [
            f"#{tracker_id} {class_names[class_id]} {confidence:.2f}"
            for class_id, tracker_id, confidence in zip(class_ids, tracker_ids, confidences)
        ]

        annotated_frame = box_annotator.annotate(
            scene=frame.copy(), detections=sv_detections
        )
        annotated_labeled_frame = label_annotator.annotate(
            scene=annotated_frame, detections=sv_detections, labels=labels
        )
        return annotated_labeled_frame


def main(args=None):
    rclpy.init(args=args)
    node = HailoDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()