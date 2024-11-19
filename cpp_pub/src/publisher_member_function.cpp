#include "rclcpp/rclcpp.hpp"       // ROS2-Header für die C++-Node-API
#include "std_msgs/msg/int32.hpp" // Header für den Nachrichtentyp Int32

// Definition der MinimalPublisher-Klasse, abgeleitet von rclcpp::Node
class MinimalPublisher : public rclcpp::Node {
public:
    // Konstruktor: Initialisiert die Node und richtet Publisher und Timer ein
    MinimalPublisher() : Node("minimal_publisher"), count_(0) {
        // Erstelle einen Publisher für das Topic "number" mit einer Warteschlange von 10
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("number", 10);

        // Erstelle einen Timer, der alle 1000 ms die Callback-Funktion aufruft
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    // Timer-Callback-Funktion: Wird bei jedem Timer-Tick aufgerufen
    void timer_callback() {
        auto message = std_msgs::msg::Int32();  // Erstelle eine Int32-Nachricht
        message.data = count_++;               // Setze die Nachricht auf den aktuellen Zählerwert
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data); // Logge die Nachricht
        publisher_->publish(message);          // Veröffentliche die Nachricht
    }

    // Membervariablen
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_; // Publisher-Objekt
    rclcpp::TimerBase::SharedPtr timer_;       // Timer-Objekt
    int count_;                                // Zähler für die Nachrichten
};

// Main-Funktion: Initialisiert ROS2, startet die Node und führt sie aus
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                  // Initialisiere ROS2
    rclcpp::spin(std::make_shared<MinimalPublisher>()); // Starte die Node
    rclcpp::shutdown();                        // Beende ROS2
    return 0;
}
