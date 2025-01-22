import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy


class FocusedLineFollowerNode(Node):
    def __init__(self):
        super().__init__('focused_line_follower')

        self.declare_parameter('speed_drive', 0.1)
        self.declare_parameter('speed_turn', 0.2)
        self.declare_parameter('threshold', 160)

        self.bridge = CvBridge()
        self.line_position = None
        self.last_line_position = None
        self.width = 640
        self.current_state = 'FOLLOW_LINE'  # Initialzustand

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscriber
        self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos_profile)

        self.create_subscription(
            String,
            '/state_change',
            self.state_callback,
            10)

        # Publisher
        self.publisher_ = self.create_publisher(Twist, '/line_cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Focused Line Follower Node gestartet")

    def state_callback(self, msg):
        """
        Aktualisiert den Zustand basierend auf Nachrichten des state_managers.
        """
        self.current_state = msg.data
        self.get_logger().info(f"Zustand geändert: {self.current_state}")

    def image_callback(self, msg):
        """
        Verarbeitet das Kamerabild und erkennt die Linie.
        """
        try:
            if self.current_state != 'FOLLOW_LINE':
                # Ignoriere Kameradaten, wenn der Zustand nicht 'FOLLOW_LINE' ist
                return

            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            threshold = self.get_parameter('threshold').get_parameter_value().integer_value
            _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

            height, width = binary.shape
            self.width = width

            mid_left = max(0, (width // 2) - 60)
            mid_right = min(width, (width // 2) + 60)
            crop = binary[height - 1:height, mid_left:mid_right]

            moments = cv2.moments(crop)
            if moments['m00'] > 0:
                self.line_position = int(moments['m10'] / moments['m00']) + mid_left
                self.last_line_position = self.line_position
                self.get_logger().info(f"Line position detected: {self.line_position}")
            else:
                self.line_position = None
                self.get_logger().warning("No line detected")

            cv2.imshow("IMG", gray)
            cv2.imshow("IMG_ROW", binary)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in image processing: {e}")

    def timer_callback(self):
        """
        Steuerungslogik für die Linienverfolgung.
        """
        if self.current_state != 'FOLLOW_LINE':
            # Keine Bewegungsbefehle senden, wenn der Zustand nicht 'FOLLOW_LINE' ist
            return

        msg = Twist()
        speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
        speed_turn = self.get_parameter('speed_turn').get_parameter_value().double_value

        if self.line_position is not None:
            if self.width // 2 - 20 < self.line_position < self.width // 2 + 20:
                msg.linear.x = speed_drive
                msg.angular.z = 0.0
                self.get_logger().info("Driving straight")
            elif self.line_position < self.width // 2:
                msg.linear.x = speed_drive / 2
                msg.angular.z = speed_turn
                self.get_logger().info("Turning left")
            elif self.line_position > self.width // 2:
                msg.linear.x = speed_drive / 2
                msg.angular.z = -speed_turn
                self.get_logger().info("Turning right")
        elif self.last_line_position is not None:
            if self.last_line_position < self.width // 2:
                msg.linear.x = 0.0
                msg.angular.z = speed_turn / 2
                self.get_logger().info("Searching line: Turning left")
            else:
                msg.linear.x = 0.0
                msg.angular.z = -speed_turn / 2
                self.get_logger().info("Searching line: Turning right")
        else:
            msg.linear.x = 0.0
            msg.angular.z = speed_turn / 2
            self.get_logger().info("Searching line")

        self.publisher_.publish(msg)

    def cleanup(self):
        """
        Bereinigt Ressourcen beim Beenden.
        """
        self.get_logger().info("Cleaning up resources...")
        try:
            self.timer.cancel()
        except Exception as e:
            pass
        cv2.destroyAllWindows()
        self.get_logger().info("Node clean-up complete.")


def main(args=None):
    rclpy.init(args=args)
    node = FocusedLineFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node gracefully...")
        node.cleanup()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
