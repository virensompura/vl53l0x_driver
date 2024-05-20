#include "/home/admin1/vama_ws/src/vl53l0x_driver/include/vl53l0x_driver/vl53l0x.h"
#include <rclcpp/rclcpp.hpp>

namespace range_sensor {

Vl53l0x::Vl53l0x(const std::string &port, uint32_t baud_rate, uint32_t firmware, boost::asio::io_service &io)
    : port_(port), baud_rate_(baud_rate), firmware_(firmware), shutting_down_(false), serial_(io, port_) {
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
}

Vl53l0x::~Vl53l0x() {
    // Add any necessary cleanup code here
}

void Vl53l0x::poll(sensor_msgs::msg::Range::SharedPtr range_scan_1, sensor_msgs::msg::Range::SharedPtr range_scan_2) {
    uint8_t temp_char;
    bool got_scan = false;
    uint8_t start_count = 0;

    while (!shutting_down_ && !got_scan) {
        boost::asio::read(serial_, boost::asio::buffer(&temp_char, 1));
        if (start_count == 0) {
            if (temp_char == 0xAD) {
                start_count = 1;
            }
        } else if (start_count == 1) {
            if (temp_char == 0xEF) {
                // Initialization failed
                start_count = 0;
                got_scan = true;
            } else if (temp_char == 0xE0) {
                // Initialization successful
                start_count = 0;
                got_scan = true;
            } else if (temp_char == 0xEA) {
                // Out of range
                start_count = 0;
                got_scan = true;
            } else if (temp_char == 0x1D) {
                got_scan = true;
                boost::asio::read(serial_, boost::asio::buffer(&ir_range_1_, 2));
                boost::asio::read(serial_, boost::asio::buffer(&ir_range_2_, 2));

                range_scan_1->radiation_type = sensor_msgs::msg::Range::INFRARED;
                range_scan_1->field_of_view = 0.44;  // 25 degrees
                range_scan_1->min_range = 0.03;
                range_scan_1->max_range = 1.2;
                range_scan_1->range = ir_range_1_ / 1000.0f;  // Convert mm to m

                range_scan_2->radiation_type = sensor_msgs::msg::Range::INFRARED;
                range_scan_2->field_of_view = 0.44;  // 25 degrees
                range_scan_2->min_range = 0.03;
                range_scan_2->max_range = 1.2;
                range_scan_2->range = ir_range_2_ / 1000.0f;  // Convert mm to m

                start_count = 0;
            }
        }
    }
}

}  // namespace range_sensor
