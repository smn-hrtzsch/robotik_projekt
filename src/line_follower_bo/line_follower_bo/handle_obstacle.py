import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class HandleObstacleNode(Node):
    def __init__(self):
        super().__init__('handle_obstacle')

        self.declare_parameter('distance_to_stop', 0.3)  # Abstand zum Hindernis
        self.declare_parameter('rotate_speed', 0.75)  # Drehgeschwindigkeit
        self.declare_parameter('scan_angle_range', 30)  # Öffnungswinkel (±30°)

        # Statusvariablen
        self.closest_distance = None
        self.is_rotating = False
        self.cooldown_active = False
        self.initial_yaw = None
        self.target_yaw = None
        self.state = "FOLLOW_LINE"  # Initialzustand
        self.cooldown_timer = None  # Timer-Referenz für Cooldown

        # QoS-Einstellungen
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber und Publisher
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_policy)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_publisher = self.create_publisher(Twist, '/rotate_cmd_vel', 10)
        self.state_publisher = self.create_publisher(String, '/state_change', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_logic)

    def normalize_angle(self, angle):
        """
        Normalisiert einen Winkel in den Bereich -pi bis pi.
        """
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def scan_callback(self, msg):
        """
        Verarbeitet Laserscan-Daten und sucht das nächste Hindernis.
        """
        if self.is_rotating or self.cooldown_active or self.state != "FOLLOW_LINE":
            return  # Keine Hinderniserkennung während Rotation oder Cooldown

        valid_ranges = []
        angle_increment = msg.angle_increment
        min_angle = msg.angle_min
        scan_angle_range = math.radians(self.get_parameter('scan_angle_range').value)

        for i, distance in enumerate(msg.ranges):
            angle = min_angle + i * angle_increment
            angle = self.normalize_angle(angle)

            if not math.isfinite(distance) or angle < -scan_angle_range or angle > scan_angle_range:
                continue

            if msg.range_min < distance < msg.range_max:
                valid_ranges.append(distance)

        if valid_ranges:
            self.closest_distance = min(valid_ranges)
            self.get_logger().info(f"Hindernis erkannt: Entfernung = {self.closest_distance:.2f} m")
        else:
            self.closest_distance = None

    def odom_callback(self, msg):
        """
        Verarbeitet Odometrie-Daten, um die aktuelle Orientierung des Roboters zu verfolgen.
        """
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.initial_yaw = current_yaw

        if self.is_rotating:
            yaw_diff = abs(self.normalize_angle(current_yaw - self.target_yaw))
            self.get_logger().info(f"Aktueller Yaw: {math.degrees(current_yaw):.2f}°, Ziel-Yaw: {math.degrees(self.target_yaw):.2f}°, Differenz: {math.degrees(yaw_diff):.2f}°")

            if yaw_diff < 0.1:
                self.stop_rotation()

    def control_logic(self):
        """
        Kontrolliert das Verhalten basierend auf der Hinderniserkennung.
        """
        if self.cooldown_active or self.is_rotating or self.state != "FOLLOW_LINE":
            return

        if self.closest_distance is not None and self.closest_distance < self.get_parameter('distance_to_stop').value:
            self.start_rotation()

    def start_rotation(self):
        """
        Startet eine Drehung um 180°.
        """
        if self.is_rotating or self.cooldown_active:
            return

        if self.initial_yaw is None:
            self.get_logger().error("Odometrie-Daten nicht verfügbar. Drehung kann nicht gestartet werden.")
            return

        self.get_logger().info("Hindernis erkannt. Stoppen und 180°-Drehung starten.")
        self.is_rotating = True
        self.state = "ROTATE"  # Zustand setzen

        # Zustand ändern
        state_msg = String()
        state_msg.data = "ROTATE"
        self.state_publisher.publish(state_msg)

        self.target_yaw = self.normalize_angle(self.initial_yaw + math.pi)

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.get_parameter('rotate_speed').value
        self.cmd_publisher.publish(twist)

    def stop_rotation(self):
        """
        Stoppt die Drehung und aktiviert den Cooldown.
        """
        self.get_logger().info("180°-Drehung abgeschlossen. Cooldown aktiv.")
        self.is_rotating = False
        self.cooldown_active = True
        self.state = "COOLDOWN"

        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_publisher.publish(stop_msg)

        # Cooldown starten (wenn noch kein Timer aktiv ist)
        if self.cooldown_timer is None:
            self.cooldown_timer = self.create_timer(1.0, self.end_cooldown)

    def end_cooldown(self):
        """
        Beendet den Cooldown und setzt den Zustand zurück auf FOLLOW_LINE.
        """
        self.get_logger().info("Cooldown beendet. Zurück zur Linienverfolgung.")
        self.cooldown_active = False
        self.state = "FOLLOW_LINE"

        # Timer löschen, damit er nicht erneut ausgelöst wird
        if self.cooldown_timer is not None:
            self.cooldown_timer.cancel()
            self.cooldown_timer = None

        # Zustand zu FOLLOW_LINE ändern
        state_msg = String()
        state_msg.data = "FOLLOW_LINE"
        self.state_publisher.publish(state_msg)

        # Hindernis zurücksetzen, um erneute Erkennung zu vermeiden
        self.closest_distance = None


def main(args=None):
    rclpy.init(args=args)
    node = HandleObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
