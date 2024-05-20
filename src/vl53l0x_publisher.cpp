#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/range.hpp>
#include "/home/admin1/vama_ws/src/vl53l0x_driver/include/vl53l0x_driver/vl53l0x.h"
#include <boost/asio.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("vl53l0x_publisher");
    auto pub = node->create_publisher<sensor_msgs::msg::Range>("range_data", 1000);

    std::string port;
    int baud_rate;
    std::string frame_id;
    int firmware_number;

    node->declare_parameter("port", "/dev/ttyACM0");
    node->declare_parameter("baud_rate", 57600);
    node->declare_parameter("frame_id", "ir_range");
    node->declare_parameter("firmware_version", 1);

    node->get_parameter("port", port);
    node->get_parameter("baud_rate", baud_rate);
    node->get_parameter("frame_id", frame_id);
    node->get_parameter("firmware_version", firmware_number);

    boost::asio::io_service io;

    try {
        range_sensor::Vl53l0x range(port, baud_rate, firmware_number, io);

        while (rclcpp::ok()) {
            auto range_scan = std::make_shared<sensor_msgs::msg::Range>();
            range_scan->header.frame_id = frame_id;
            range_scan->header.stamp = node->now();
            range.poll(range_scan);
            pub->publish(*range_scan);
        }
        range.close();
        return 0;
    } catch (const std::exception& ex) {
        // Handle exception
        return -1;
    }
}
