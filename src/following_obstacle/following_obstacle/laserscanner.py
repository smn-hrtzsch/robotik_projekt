import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np

class FollowObstacle(Node):
    def __init__(self):
        super().__init__('follow_obstacle')

        # Parameter deklarieren
        self.declare_parameter('distance_to_stop', 0.3)  # Minimale Entfernung zum Hindernis, bei der der Roboter stoppt
        self.declare_parameter('distance_to_follow', 1.0)  # Maximale Entfernung, innerhalb derer der Roboter einem Hindernis folgt
        self.declare_parameter('speed_linear', 0.15)  # Maximale Vorwärtsgeschwindigkeit des Roboters
        self.declare_parameter('speed_angular', 0.5)  # Maximale Drehgeschwindigkeit des Roboters
        self.declare_parameter('scan_angle_range', 90)  # Öffnungswinkel für die Hinderniserkennung in Grad

        # QoS-Profil für drahtlose Übertragung
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # Beste Bemühungen zur Datenübertragung
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,  # Behalte nur die letzten Nachrichten
            depth=1  # Nur eine Nachricht im Puffer
        )

        # Subscriber und Publisher erstellen
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_policy
        )  # Dieser Subscriber empfängt kontinuierlich Laserscandaten für die Hinderniserkennung
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)  # Veröffentlicht Bewegungsbefehle

        # Nächstgelegenes Hindernis
        self.closest_distance = None  # Entfernung zum nächsten Hindernis
        self.closest_angle = 0.0  # Winkel zum nächsten Hindernis

        # Timer für Bewegungslogik
        self.timer = self.create_timer(0.1, self.control_logic)  # Ruft die Bewegungslogik alle 100 ms auf

    def normalize_angle(self, angle):
        """
        Normalisiert einen Winkel in den Bereich -pi bis pi, um Berechnungen zu erleichtern.
        """
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def scan_callback(self, msg):
        """
        Verarbeitet Laserscan-Daten und sucht das nächste Hindernis. Hindernisse hinter dem Roboter werden ignoriert.
        """
        total_scans = len(msg.ranges)
        angle_increment = msg.angle_increment  # Schrittweite zwischen zwei Scandaten in Radiant
        min_angle = msg.angle_min  # Startwinkel des Scans

        valid_ranges = []  # Speichert gültige Distanzen
        valid_angles = []  # Speichert die entsprechenden Winkel

        # Konfigurierbarer Öffnungswinkel
        scan_angle_range = math.radians(self.get_parameter('scan_angle_range').value)  # Öffnungswinkel in Radiant

        for i, distance in enumerate(msg.ranges):
            angle = min_angle + i * angle_increment  # Berechnet den Winkel für die aktuelle Messung
            angle = self.normalize_angle(angle)  # Normalisiert den Winkel

            if not math.isfinite(distance):  # Überspringt ungültige oder unendliche Werte
                continue

            # Füge alle Hindernisse hinzu, die im gültigen Bereich und im Öffnungswinkel liegen
            # Dies sicherstellt, dass nur Hindernisse im gültigen Bereich und im definierten Winkelbereich betrachtet werden
            if msg.range_min < distance < msg.range_max and -scan_angle_range <= angle <= scan_angle_range:
                valid_ranges.append(distance)
                valid_angles.append(angle)

        if valid_ranges:
            # Finde das nächste Hindernis im definierten Winkelbereich
            self.closest_distance = float(np.min(valid_ranges))  # Kürzeste Distanz
            self.closest_angle = valid_angles[np.argmin(valid_ranges)]  # Winkel zur kürzesten Distanz
            self.get_logger().info(f"Hindernis erkannt: Entfernung = {self.closest_distance:.2f} m, Winkel = {math.degrees(self.closest_angle):.1f}°")
        else:
            self.closest_distance = None  # Kein Hindernis erkannt
            self.closest_angle = 0.0

    def control_logic(self):
        """
        Kontrolliert die Bewegung des Roboters basierend auf der Position des Hindernisses.
        """
        distance_to_stop = self.get_parameter('distance_to_stop').value  # Mindestabstand zum Stoppen
        distance_to_follow = self.get_parameter('distance_to_follow').value  # Maximale Entfernung zum Verfolgen
        speed_linear = self.get_parameter('speed_linear').value  # Maximale Vorwärtsgeschwindigkeit
        speed_angular = self.get_parameter('speed_angular').value  # Maximale Drehgeschwindigkeit

        twist_msg = Twist()  # Erstellt eine neue Nachricht für Bewegungsbefehle

        if self.closest_distance is None or self.closest_distance > distance_to_follow:
            # Kein Hindernis erkannt oder zu weit entfernt
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.get_logger().info("Kein Hindernis erkannt oder zu weit entfernt. Roboter bleibt stehen.")
        elif self.closest_distance < distance_to_stop:
            # Zu nah am Hindernis
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.get_logger().info(f"Zu nah am Hindernis (Entfernung: {self.closest_distance:.2f} m). Roboter stoppt.")
        else:
            # Hindernis verfolgen
            twist_msg.linear.x = min(speed_linear, self.closest_distance / 2.5)  # Reduziert Geschwindigkeit proportional zur Entfernung

            # Drehgeschwindigkeit basierend auf dem Winkel
            angular_correction = math.degrees(self.closest_angle) * 0.05  # Verstärkung des Winkels (angepasst)
            twist_msg.angular.z = max(-speed_angular, min(speed_angular, angular_correction))  # Begrenzung der Drehgeschwindigkeit

            self.get_logger().info(f"Hindernis verfolgen: Geschwindigkeit = {twist_msg.linear.x:.2f} m/s, Winkelgeschwindigkeit = {twist_msg.angular.z:.2f} rad/s")

        self.publisher_.publish(twist_msg)  # Veröffentlicht die Bewegungsbefehle

    def destroy_node(self):
        """
        Sicherer Shutdown der Node.
        """
        self.get_logger().info("Node wird beendet.")
        if self.subscription is not None:
            self.subscription.destroy()
        if self.publisher_ is not None:
            self.publisher_.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowObstacle()
    try:
        rclpy.spin(node)  # Startet den Node und lässt ihn laufen
    except KeyboardInterrupt:
        node.get_logger().info("Programm wird beendet (KeyboardInterrupt).")
    finally:
        node.destroy_node()  # Beendet den Node sicher
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
