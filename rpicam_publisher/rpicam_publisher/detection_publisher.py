import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

import re


class DetectionPublisher(Node):
    def __init__(self):
        super().__init__('detection_publisher')
        self.marker_pub = self.create_publisher(MarkerArray, '/detections', 10)
        self.sub = self.create_subscription(String, 'imx500_detections', self.detection_callback, 10)
        self.marker_id = 0

    def detection_callback(self, msg: String):
        try:
            # Regex pattern to match: label: score at (x1, y1, x2, y2)
            pattern = r"(\w+): ([0-9.]+) at \((\d+), (\d+), (\d+), (\d+)\)"
            matches = re.findall(pattern, msg.data)

            if not matches:
                self.get_logger().warn("No valid detections found in message")
                return

            marker_array = MarkerArray()
            self.marker_id = 0  # reset marker IDs each time

            for match in matches:
                label, score, x1, y1, x2, y2 = match
                x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))

                # Convert to center and dimensions
                x_center = float((x1 + x2) / 2.0)
                y_center = float((y1 + y2) / 2.0)
                width = float(abs(x2 - x1))
                height = float(abs(y2 - y1))

                # Box marker
                box = Marker()
                box.header.frame_id = "map"
                box.header.stamp = self.get_clock().now().to_msg()
                box.ns = "detections"
                box.id = self.marker_id
                self.marker_id += 1
                box.type = Marker.CUBE
                box.action = Marker.ADD
                box.pose.position.x = x_center
                box.pose.position.y = y_center
                box.pose.position.z = 0.0
                box.scale.x = width
                box.scale.y = height
                box.scale.z = 0.1
                box.color.r = 0.0
                box.color.g = 1.0
                box.color.b = 0.0
                box.color.a = 0.8
                marker_array.markers.append(box)

                # Label marker
                text = Marker()
                text.header.frame_id = "map"
                text.header.stamp = self.get_clock().now().to_msg()
                text.ns = "labels"
                text.id = self.marker_id
                self.marker_id += 1
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x = x_center
                text.pose.position.y = y_center
                text.pose.position.z = 0.15
                text.scale.z = 0.1
                text.color.r = 1.0
                text.color.g = 1.0
                text.color.b = 1.0
                text.color.a = 1.0
                text.text = f"{label} ({score})"
                marker_array.markers.append(text)

            self.marker_pub.publish(marker_array)
            self.get_logger().info(f"Published {len(marker_array.markers)} markers")

        except Exception as e:
            self.get_logger().error(f"Error parsing detection message: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DetectionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
