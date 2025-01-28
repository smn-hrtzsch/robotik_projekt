import rclpy  # ROS 2 Python API
from rclpy.node import Node  # Basisklasse für Nodes in ROS 2
import cv2  # OpenCV für Bildverarbeitung
from cv_bridge import CvBridge  # Konvertierung von ROS-Bilddaten zu OpenCV
from sensor_msgs.msg import CompressedImage  # ROS-Nachrichtentyp für komprimierte Bilder
from geometry_msgs.msg import Twist  # ROS-Nachrichtentyp für Bewegungsbefehle

# Definition der Node für die Linienverfolgung
class FocusedLineFollowerNode(Node):
    def __init__(self):
        super().__init__('focused_line_follower')  # Initialisierung der Node mit Namen

        # Parameter deklarieren, die während der Laufzeit angepasst werden können
        self.declare_parameter('speed_drive', 0.1)  # Geschwindigkeit für Geradeausfahren
        self.declare_parameter('speed_turn', 0.2)  # Geschwindigkeit für Drehungen
        self.declare_parameter('threshold', 180)  # Schwellenwert für die Bildbinarisierung

        self.bridge = CvBridge()  # Brücke zwischen ROS-Bilddaten und OpenCV
        self.line_position = None  # Aktuelle Position der Linie (wenn erkannt)
        self.last_line_position = None  # Letzte bekannte Position der Linie
        self.width = 640  # Breite des Bildes (Standardwert)

        # Subscriber für das Kamerabild
        self.subscription = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, 10)

        # Publisher für Bewegungsbefehle
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer für regelmäßige Steuerung
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Focused Line Follower Node gestartet")

    # Callback-Funktion für empfangene Kamerabilder
    def image_callback(self, msg):
        try:
            # Konvertierung des ROS-Bildes in ein OpenCV-Bild
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Binarisierung des Bildes basierend auf dem Schwellenwert
            threshold = self.get_parameter('threshold').get_parameter_value().integer_value
            _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

            # Dimensionen des Bildes
            height, width = binary.shape
            self.width = width

            # Auswahl des mittleren Bereichs (120 Pixel breit)
            mid_left = max(0, (width // 2) - 60)
            mid_right = min(width, (width // 2) + 60)
            crop = binary[height-1:height, mid_left:mid_right]

            # Berechnung der Bildmomente zur Bestimmung der Linienposition
            moments = cv2.moments(crop)
            if moments['m00'] > 0:
                self.line_position = int(moments['m10'] / moments['m00']) + mid_left
                self.last_line_position = self.line_position
                self.get_logger().info(f"Line position detected: {self.line_position}")
            else:
                self.line_position = None
                self.get_logger().warning("No line detected")

            # Debug-Ansicht des Bildes
            cv2.imshow("IMG", gray)
            cv2.imshow("IMG_ROW", binary)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in image processing: {e}")

    # Timer-Callback für Bewegungssteuerung
    def timer_callback(self):
        msg = Twist()
        speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
        speed_turn = self.get_parameter('speed_turn').get_parameter_value().double_value

        if self.line_position is not None:
            if self.width // 2 - 20 < self.line_position < self.width // 2 + 20:
                msg.linear.x = speed_drive
                msg.angular.z = 0.0
            elif self.line_position < self.width // 2:
                msg.linear.x = speed_drive / 2
                msg.angular.z = speed_turn
            elif self.line_position > self.width // 2:
                msg.linear.x = speed_drive / 2
                msg.angular.z = -speed_turn
        elif self.last_line_position is not None:
            if self.last_line_position < self.width // 2:
                msg.linear.x = 0.0
                msg.angular.z = speed_turn / 2
            else:
                msg.linear.x = 0.0
                msg.angular.z = -speed_turn / 2
        else:
            msg.linear.x = 0.0
            msg.angular.z = speed_turn / 2

        self.publisher_.publish(msg)

# Hauptprogramm
def main(args=None):
    rclpy.init(args=args)

    # Haupt-Node erstellen
    node = FocusedLineFollowerNode()

    try:
        rclpy.spin(node)  # Haupt-Node läuft, bis ein KeyboardInterrupt kommt
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt detected. StopNode wird den Shutdown steuern.")
    finally:
        # Ressourcen der Haupt-Node freigeben, aber KEIN `rclpy.shutdown()` aufrufen
        node.destroy_node()

