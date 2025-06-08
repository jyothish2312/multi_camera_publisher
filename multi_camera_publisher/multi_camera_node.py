import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class MultiCameraPublisher(Node):
    def __init__(self):
        super().__init__('multi_camera_publisher')
        self.bridge = CvBridge()

        self.declare_parameter("max_devices", 10)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("exposure", -1)  # -1 = auto

        max_devices = self.get_parameter("max_devices").value
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        exposure = self.get_parameter("exposure").value

        self.captures = []
        self.camera_publishers = []
        self.camera_paths = []

        # 🔍 Try opening /dev/video0 to /dev/videoN
        for i in range(max_devices):
            path = f"/dev/video{i}"
            cap = cv2.VideoCapture(path)

            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                    if exposure >= 0:
                        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
                        cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure))
                    else:
                        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)

                    self.get_logger().info(f"✅ Camera detected: {path}")
                    self.captures.append(cap)
                    self.camera_paths.append(path)
                    pub = self.create_publisher(Image, f"/camera_{len(self.captures)-1}/image_raw", 10)
                    self.camera_publishers.append(pub)
                else:
                    self.get_logger().warn(f"⚠️ {path} opened but failed to read frame.")
                    cap.release()
            else:
                self.get_logger().info(f"❌ Skipping {path} (not opened)")

        if not self.captures:
            self.get_logger().error("❌ No usable cameras detected.")
        else:
            self.timer = self.create_timer(0.03, self.publish_frames)

    def publish_frames(self):
        for idx, cap in enumerate(self.captures):
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn(f"⚠️ Frame capture failed from {self.camera_paths[idx]}")
                continue
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.camera_publishers[idx].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

