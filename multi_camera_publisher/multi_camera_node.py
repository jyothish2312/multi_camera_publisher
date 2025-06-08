import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor

class MultiCameraPublisher(Node):
    def __init__(self):
        super().__init__('multi_camera_publisher')
        self.bridge = CvBridge()

        self.declare_parameter("camera_ids", [0, 1, 2])
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("exposure", -1)  # -1 = auto

        self.camera_ids = self.get_parameter("camera_ids").value
        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        self.exposure = self.get_parameter("exposure").value

        self.captures = []
        self.camera_publishers = []

        for idx, cam_id in enumerate(self.camera_ids):
            cap = cv2.VideoCapture(cam_id)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if self.exposure >= 0:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
                cap.set(cv2.CAP_PROP_EXPOSURE, float(self.exposure))
            else:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)

            if not cap.isOpened():
                self.get_logger().error(f"Failed to open camera {cam_id}")
                continue

            self.captures.append(cap)
            pub = self.create_publisher(Image, f'/camera_{idx}/image_raw', 10)
            self.camera_publishers.append(pub)

        self.timer = self.create_timer(0.03, self.publish_frames)  # ~30 FPS

    def publish_frames(self):
        for idx, cap in enumerate(self.captures):
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warning(f"Camera {self.camera_ids[idx]} frame not captured.")
                continue
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.camera_publishers[idx].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
