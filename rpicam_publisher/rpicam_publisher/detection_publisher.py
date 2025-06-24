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
        self.prev_marker_count = 0
        self.scale_factor = 0.01  # Adjust as needed (e.g., pixel to meter conversion)

    def detection_callback(self, msg: String):
        try:
            pattern = r"(\w+): ([0-9.]+) at \((\d+), (\d+), (\d+), (\d+)\)"
            matches = re.findall(pattern, msg.data)

            marker_array = MarkerArray()
            current_id = 0
            now = self.get_clock().now().to_msg()

            if not matches:
                self.get_logger().info("No detections — removing all previous markers")
                # Delete all previously active markers
                for marker_id in range(self.prev_marker_count):
                    delete_marker = Marker()
                    delete_marker.header.frame_id = "map"
                    delete_marker.header.stamp = now
                    delete_marker.ns = "detections" if marker_id % 2 == 0 else "labels"
                    delete_marker.id = marker_id
                    delete_marker.action = Marker.DELETE
                    marker_array.markers.append(delete_marker)

                self.prev_marker_count = 0
                self.marker_pub.publish(marker_array)
                return

            for match in matches:
                label, score, x1, y1, x2, y2 = match
                x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))

                # Convert to center and dimensions
                x_center = ((x1 + x2) / 2.0) * self.scale_factor
                y_center = ((y1 + y2) / 2.0) * self.scale_factor
                width = abs(x2 - x1) * self.scale_factor
                height = abs(y2 - y1) * self.scale_factor

                # Box marker
                box = Marker()
                box.header.frame_id = "map"
                box.header.stamp = now
                box.ns = "detections"
                box.id = current_id
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
                box.lifetime.sec = 1  # Ensures RViz removes marker if not updated
                marker_array.markers.append(box)
                current_id += 1

                # Text marker
                text = Marker()
                text.header.frame_id = "map"
                text.header.stamp = now
                text.ns = "labels"
                text.id = current_id
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x = x_center
                text.pose.position.y = y_center
                text.pose.position.z = 0.15
                text.scale.z = 0.15
                text.color.r = 1.0
                text.color.g = 1.0
                text.color.b = 1.0
                text.color.a = 1.0
                text.text = f"{label} ({score})"
                text.lifetime.sec = 1  # Same here
                marker_array.markers.append(text)
                current_id += 1

            # Delete any old markers not used this time
            for marker_id in range(current_id, self.prev_marker_count):
                delete_marker = Marker()
                delete_marker.header.frame_id = "map"
                delete_marker.header.stamp = now
                delete_marker.ns = "detections" if marker_id % 2 == 0 else "labels"
                delete_marker.id = marker_id
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)

            self.prev_marker_count = current_id
            self.marker_pub.publish(marker_array)
            self.get_logger().info(f"Published {current_id} markers")

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
