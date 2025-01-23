import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class StopNode(Node):
    def __init__(self):
        super().__init__('stop')
        self.get_logger().info("StopNode gestartet.")

        # Publisher für Stop-Nachrichten
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Shutdown-Flag
        self.shutdown_called = False

    def send_stop_command(self):
        """Sendet Stop-Befehle und wartet, bevor der Kontext heruntergefahren wird."""
        if self.shutdown_called:
            return
        self.shutdown_called = True

        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0

        try:
            # Mehrfaches Senden des Stop-Befehls
            for i in range(5):
                self.publisher_.publish(stop_msg)
                self.get_logger().info(f"Stop message sent ({i+1}/5).")
                time.sleep(0.2)  # 200 ms Verzögerung zwischen Veröffentlichungen

            self.get_logger().info("Stop-Befehl erfolgreich gesendet. Shutdown wird eingeleitet.")

        except Exception as e:
            self.get_logger().warning(f"Fehler beim Senden des Stop-Befehls: {e}")

    def shutdown_node(self):
        """Verzögerter Shutdown des ROS-Kontexts."""
        self.get_logger().info("StopNode wird beendet.")
        self.send_stop_command()  # Stelle sicher, dass der Roboter angehalten wird
        time.sleep(1.0)  # Warte kurz, damit die Stop-Nachricht verarbeitet werden kann
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = StopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt erkannt. StopNode wird aktiv.")
        node.shutdown_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
