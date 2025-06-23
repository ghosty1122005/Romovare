import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import numpy as np
import cv2
import time
from functools import lru_cache

from picamera2 import MappedArray, Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import (NetworkIntrinsics, postprocess_nanodet_detection)

# Globals
last_detections = []
last_results = []

class Detection:
    def __init__(self, coords, category, conf, metadata, picam2, imx500):
        self.category = category
        self.conf = conf
        self.box = imx500.convert_inference_coords(coords, metadata, picam2)


class IMX500Node(Node):
    def __init__(self):
        super().__init__('imx500_node')

        self.declare_parameter("model", "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk")
        self.declare_parameter("threshold", 0.55)
        self.declare_parameter("iou", 0.65)
        self.declare_parameter("max_detections", 10)

        self.model_path = self.get_parameter("model").get_parameter_value().string_value
        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value
        self.iou = self.get_parameter("iou").get_parameter_value().double_value
        self.max_detections = self.get_parameter("max_detections").get_parameter_value().integer_value

        self.imx500 = IMX500(self.model_path)
        self.intrinsics = self.imx500.network_intrinsics or NetworkIntrinsics()
        if self.intrinsics.task != "object detection":
            self.get_logger().error("Model is not for object detection")
            exit(1)
        self.intrinsics.update_with_defaults()

        if self.intrinsics.labels is None:
            with open("/usr/share/imx500-models/assets/coco_labels.txt", "r") as f:
                self.intrinsics.labels = f.read().splitlines()

        self.picam2 = Picamera2(self.imx500.camera_num)
        config = self.picam2.create_preview_configuration(controls={"FrameRate": self.intrinsics.inference_rate}, buffer_count=12)
        self.picam2.start(config, show_preview=False)

        if self.intrinsics.preserve_aspect_ratio:
            self.imx500.set_auto_aspect_ratio()

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "/imx500/image", 10)
        self.detection_pub = self.create_publisher(String, "/imx500_detections", 10)

        # Timer to run detection
        self.timer = self.create_timer(1.0 / self.intrinsics.inference_rate, self.timer_callback)

        self.picam2.pre_callback = self.draw_detections
        self.get_logger().info("IMX500 detection node initialized.")

    def parse_detections(self, metadata):
        global last_detections
        np_outputs = self.imx500.get_outputs(metadata, add_batch=True)
        input_w, input_h = self.imx500.get_input_size()
        if np_outputs is None:
            return last_detections
        if self.intrinsics.postprocess == "nanodet":
            boxes, scores, classes = postprocess_nanodet_detection(
                outputs=np_outputs[0], conf=self.threshold, iou_thres=self.iou,
                max_out_dets=self.max_detections)[0]
            from picamera2.devices.imx500.postprocess import scale_boxes
            boxes = scale_boxes(boxes, 1, 1, input_h, input_w, False, False)
        else:
            boxes, scores, classes = np_outputs[0][0], np_outputs[1][0], np_outputs[2][0]
            if self.intrinsics.bbox_normalization:
                boxes = boxes / input_h
            if self.intrinsics.bbox_order == "xy":
                boxes = boxes[:, [1, 0, 3, 2]]
            boxes = np.array_split(boxes, 4, axis=1)
            boxes = zip(*boxes)

        last_detections = [
            Detection(box, category, score, metadata, self.picam2, self.imx500)
            for box, score, category in zip(boxes, scores, classes)
            if score > self.threshold
        ]
        return last_detections

    @lru_cache
    def get_labels(self):
        labels = self.intrinsics.labels
        if self.intrinsics.ignore_dash_labels:
            labels = [label for label in labels if label and label != "-"]
        return labels

    def draw_detections(self, request, stream="main"):
        detections = last_detections
        if detections is None:
            return
        labels = self.get_labels()
        with MappedArray(request, stream) as m:
            for detection in detections:
                x, y, w, h = detection.box
                label = f"{labels[int(detection.category)]} ({detection.conf:.2f})"
                (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                text_x = x + 5
                text_y = y + 15
                overlay = m.array.copy()
                cv2.rectangle(overlay, (text_x, text_y - text_height), (text_x + text_width, text_y + baseline), (255, 255, 255), cv2.FILLED)
                alpha = 0.30
                cv2.addWeighted(overlay, alpha, m.array, 1 - alpha, 0, m.array)
                cv2.putText(m.array, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                cv2.rectangle(m.array, (x, y), (x + w, y + h), (0, 255, 0), thickness=2)

            if self.intrinsics.preserve_aspect_ratio:
                b_x, b_y, b_w, b_h = self.imx500.get_roi_scaled(request)
                cv2.putText(m.array, "ROI", (b_x + 5, b_y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                cv2.rectangle(m.array, (b_x, b_y), (b_x + b_w, b_y + b_h), (255, 0, 0), thickness=2)

    def timer_callback(self):
        global last_results
        metadata = self.picam2.capture_metadata()
        last_results = self.parse_detections(metadata)

        # Publish detections as String
        if last_results:
            labels = self.get_labels()
            detection_strings = []
            for det in last_results:
                label = labels[int(det.category)] if det.category < len(labels) else "Unknown"
                detection_strings.append(f"{label}: {det.conf:.2f} at {det.box}")
            msg = String()
            msg.data = "\n".join(detection_strings)
            self.detection_pub.publish(msg)

        # Publish annotated image
        request = self.picam2.capture_request()
        if request:
            frame = request.make_array("main")
            self.draw_detections(request)
            request.release()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            img_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
            self.image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMX500Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
