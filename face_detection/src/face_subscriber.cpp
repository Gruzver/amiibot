#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <serial/serial.h> // Install the 'serial' library if not already installed
#include <iostream>
#include <vector>

class FaceSubscriber : public rclcpp::Node
{
public:
    FaceSubscriber() : Node("face_subscriber")
    {
        // Initialize the serial connection
        try {
            serial_conn.setPort("/dev/ttyACM0"); // Update this to match your Arduino's port
            serial_conn.setBaudrate(9600);       // Match the baud rate configured on the Arduino
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_conn.setTimeout(timeout);
            serial_conn.open();
            RCLCPP_INFO(this->get_logger(), "Serial connection opened successfully.");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial connection: %s", e.what());
            return;
        }

        // Subscribe to the face_coordinates topic
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "face_coordinates", 10,
            std::bind(&FaceSubscriber::face_coordinates_callback, this, std::placeholders::_1));
    }

private:

    void face_coordinates_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Extract the coordinates from the message
        const std::vector<float>& coordinates = msg->data;

        if (coordinates.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Received insufficient coordinates. Skipping...");
            return;
        }

        // Get the first two values as X and Y
        uint8_t x = static_cast<uint8_t>(std::clamp(coordinates[0], 0.0f, 255.0f));
        uint8_t y = static_cast<uint8_t>(std::clamp(coordinates[1], 0.0f, 255.0f));

        // Publish the coordinates to the terminal
        RCLCPP_INFO(this->get_logger(), "X: %d, Y: %d", x, y);

        // Send the data over the serial connection as two bytes
        try {
            uint8_t data[2] = { x, y };
            serial_conn.write(data, 2);  // Corrección aquí
            RCLCPP_INFO(this->get_logger(), "Sent bytes to Arduino: [%d, %d]", x, y);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error writing to serial: %s", e.what());
        }
    }



    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    serial::Serial serial_conn; // Serial connection object
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FaceSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}