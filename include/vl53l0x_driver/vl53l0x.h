#ifndef VL53L0X_DRIVER_VL53L0X_H_
#define VL53L0X_DRIVER_VL53L0X_H_

#include <sensor_msgs/msg/range.hpp>
#include <boost/asio.hpp>
#include <string>

namespace range_sensor {

class Vl53l0x {
public:
    Vl53l0x(const std::string &port, uint32_t baud_rate, uint32_t firmware, boost::asio::io_service &io);

    ~Vl53l0x();

    void poll(sensor_msgs::msg::Range::SharedPtr range_scan_1, sensor_msgs::msg::Range::SharedPtr range_scan_2);
    void close() { shutting_down_ = true; };

private:
    std::string port_;
    uint32_t baud_rate_;
    uint32_t firmware_;
    bool shutting_down_;
    boost::asio::serial_port serial_;
    uint16_t ir_range_1_;
    uint16_t ir_range_2_;
};

}  // namespace range_sensor

#endif  // VL53L0X_DRIVER_VL53L0X_H_
