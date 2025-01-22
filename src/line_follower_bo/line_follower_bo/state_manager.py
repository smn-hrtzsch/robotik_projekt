import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')

        self.state = 'FOLLOW_LINE'  # Initialzustand

        # Subscriber für Zustandänderungen
        self.create_subscription(String, '/state_change', self.state_callback, 10)
        # Subscriber für Bewegungsbefehle der Linie und Drehung
        self.create_subscription(Twist, '/line_cmd_vel', self.line_cmd_callback, 10)
        self.create_subscription(Twist, '/rotate_cmd_vel', self.rotate_cmd_callback, 10)

        # Publisher für endgültige Bewegungsbefehle
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("StateManager gestartet")

    def state_callback(self, msg):
        """
        Reagiert auf Änderungen des Zustands.
        """
        new_state = msg.data
        if new_state != self.state:  # Nur bei Zustandswechsel
            self.get_logger().info(f"Zustand geändert: {new_state}")
            self.state = new_state

    def line_cmd_callback(self, msg):
        """
        Leitet Bewegungsbefehle der Linienverfolgung weiter, wenn im Zustand 'FOLLOW_LINE'.
        """
        if self.state == 'FOLLOW_LINE':
            self.cmd_publisher.publish(msg)
            self.get_logger().info("Befehl für Linienverfolgung weitergeleitet")
        else:
            self.get_logger().debug("Linienverfolgung blockiert, aktueller Zustand: ROTATE")

    def rotate_cmd_callback(self, msg):
        """
        Leitet Bewegungsbefehle für Drehung weiter, wenn im Zustand 'ROTATE'.
        """
        if self.state == 'ROTATE':
            self.cmd_publisher.publish(msg)
            self.get_logger().info("Befehl für Drehung weitergeleitet")
        else:
            self.get_logger().debug("Drehbefehl blockiert, aktueller Zustand: FOLLOW_LINE")


def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("StateManager wird beendet")
    finally:
        rclpy.shutdown()
