import rclpy                            # ROS2-API für Python
from rclpy.node import Node             # Basisklasse für Nodes
from std_msgs.msg import Int32, Float32 # Nachrichtentypen Int32 und Float32
from time import time                   # Zeitfunktion zur Berechnung der Differenzen

# Definition der MinimalSubscriber-Klasse, abgeleitet von rclpy.Node
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')  # Initialisiere die Node mit dem Namen "minimal_subscriber"

        # Abonniere das Topic "number" mit einer Warteschlange von 10
        self.subscription = self.create_subscription(
            Int32,
            'number',
            self.listener_callback,
            10)

        # Erstelle einen Publisher für das Topic "diff" (Float32-Nachrichten)
        self.publisher_ = self.create_publisher(Float32, 'diff', 10)

        self.last_time = None  # Speichert die Zeit der letzten Nachricht

    # Callback-Funktion: Wird bei jeder empfangenen Nachricht aufgerufen
    def listener_callback(self, msg):
        current_time = time()  # Aktuelle Systemzeit

        # Berechne die Differenz, falls es eine vorherige Nachricht gibt
        if self.last_time is not None:
            diff = Float32()                   # Erstelle eine Float32-Nachricht
            diff.data = current_time - self.last_time  # Berechne die Zeitdifferenz
            self.publisher_.publish(diff)     # Veröffentliche die Zeitdifferenz auf dem Topic "diff"
            self.get_logger().info(f"Published time diff: {diff.data:.2f} seconds") # Logge die Differenz

        self.last_time = current_time  # Aktualisiere die Zeit der letzten Nachricht

# Main-Funktion: Initialisiert ROS2, startet die Node und führt sie aus
def main(args=None):
    rclpy.init(args=args)                    # Initialisiere ROS2
    minimal_subscriber = MinimalSubscriber() # Erstelle die Subscriber-Node
    rclpy.spin(minimal_subscriber)           # Starte die Node
    minimal_subscriber.destroy_node()        # Zerstöre die Node (nach Beendigung)
    rclpy.shutdown()                         # Beende ROS2

# Einstiegspunkt
if __name__ == '__main__':
    main()
