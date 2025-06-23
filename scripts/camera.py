import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import time

from picamera2 import CompletedRequest, Picamera2, MappedArray
from picamera2.devices import IMX500
from picamera2.devices.imx500 import NetworkIntrinsics
from picamera2.devices.imx500.postprocess import softmax

LABELS = None


class Classification:
    def __init__(self, idx: int, score: float):
        self.idx = idx
        self.score = score


class IMX500ClassifierNode(Node):
    def __init__(self):
        super().__init__('imx500_classifier')
        self.publisher = self.create_publisher(String, 'classification', 10)
        self.image_pub = self.create_publisher(Image, 'camera/image', 10)
        self.bridge = CvBridge()

        self.get_logger().info("Loading model...")
        self.imx500 = IMX500("/usr/share/imx500-models/imx500_network_mobilenet_v2.rpk")
        self.intrinsics = self.imx500.network_intrinsics or NetworkIntrinsics()
        self.intrinsics.task = "classification"
        self.intrinsics.update_with_defaults()

        if self.intrinsics.labels is None:
            with open("assets/imagenet_labels.txt", "r") as f:
                self.intrinsics.labels = f.read().splitlines()

        self.picam2 = Picamera2(self.imx500.camera_num)
        config = self.picam2.create_preview_configuration(
            controls={"FrameRate": self.intrinsics.inference_rate}, buffer_count=12)
        self.picam2.start(config, show_preview=False)

        if self.intrinsics.preserve_aspect_ratio:
            self.imx500.set_auto_aspect_ratio()

        self.picam2.pre_callback = self.classify_and_publish
        self.get_logger().info("Node initialized, starting camera...")

        # Timer to keep the node alive
        self.create_timer(0.5, lambda: None)

    def classify_and_publish(self, request: CompletedRequest):
        global LABELS
        np_outputs = self.imx500.get_outputs(request.get_metadata())
        if np_outputs is None:
            return

        np_output = np_outputs[0]
        if self.intrinsics.softmax:
            np_output = softmax(np_output)

        if LABELS is None:
            LABELS = self.intrinsics.labels
            if len(LABELS) == 1001:
                LABELS = LABELS[1:]

        top_indices = np.argpartition(-np_output, 3)[:3]
        top_indices = top_indices[np.argsort(-np_output[top_indices])]
        results = [Classification(idx, np_output[idx]) for idx in top_indices]

        # Publish the top classification result
        label = LABELS[results[0].idx]
        score = results[0].score
        msg = String()
        msg.data = f"{label}: {score:.3f}"
        self.publisher.publish(msg)

        # Publish annotated image
        with MappedArray(request, "main") as m:
            image = m.array
            text = f"{label}: {score:.3f}"
            cv2.putText(image, text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
            self.image_pub.publish(ros_image)


def main(args=None):
    rclpy.init(args=args)
    node = IMX500ClassifierNode()
    rclpy.spin(node)
    node.picam2.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
