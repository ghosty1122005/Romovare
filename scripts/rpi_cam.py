import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class RPiCamPublisher(Node):
    def __init__(self):
        super().__init__('rpicam_imx500_node')
        self.pub = self.create_publisher(String, 'imx500_detections', 10)

        # Launch rpicam-hello with IMX500 processing
        self.proc = subprocess.Popen(
            [
                'rpicam-hello',
                '-t', '0',
                '--post-process-file', '/usr/share/rpi-camera-assets/imx500_mobilenet_ssd.json',
                '--viewfinder-width', '1920',
                '--viewfinder-height', '1080',
                '--framerate', '30'
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True
        )

        self.create_timer(0.1, self.read_output)

    def read_output(self):
        if self.proc.poll() is not None:
            self.get_logger().warn("rpicam-hello exited")
            return

        line = self.proc.stdout.readline()
        if line:
            if "Post-Process Result" in line:
                msg = String()
                msg.data = line.strip()
                self.pub.publish(msg)
                self.get_logger().info(f"Published detection: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = RPiCamPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
