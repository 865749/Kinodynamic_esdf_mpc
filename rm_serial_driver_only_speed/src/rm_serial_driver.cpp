#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "packet.hpp"
#include "rm_serial_driver.hpp"

namespace rm_serial_driver {
    RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions &options)
        : Node("rm_serial_driver", options),
          serial_port(io_context) {
        RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

        getParams();

        try {
            openPort();
            RCLCPP_INFO(get_logger(), "serial open OK!");
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(get_logger(), "Error creating serial port: %s - %s", config.device_name.c_str(), ex.what());
            throw ex;
        }

        action_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", rclcpp::SensorDataQoS(),
                std::bind(&RMSerialDriver::sendPacket, this, std::placeholders::_1));
    }

    RMSerialDriver::~RMSerialDriver() {
        if (serial_port.is_open()) {
            closePort();
        }
        if (!io_context.stopped()) {
            boost::asio::post(io_context, [this]() {
                io_context.stop();
            });
        }
    }

    void RMSerialDriver::sendPacket(const geometry_msgs::msg::Twist::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "x: %.2f, y: %.2f,  rz: %.2f", msg->linear.x, msg->linear.y, msg->angular.z);
        try {
            SendPacket sendPacket;
            sendPacket.line_vel_x = static_cast<float>(msg->linear.x);
            sendPacket.line_vel_y = static_cast<float>(msg->linear.y);
            sendPacket.angle_vel_z = static_cast<float>(msg->angular.z);
            sendPacket.is_use_top = true;
            sendPacket.priority = 0x00;
            serial_port.write_some(boost::asio::buffer(&sendPacket, sizeof(sendPacket)));
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
            reopenPort();
        }
    }

    void RMSerialDriver::openPort() {
        serial_port.open(config.device_name);
        serial_port.set_option(boost::asio::serial_port_base::baud_rate(config.baud_rate));
        serial_port.set_option(boost::asio::serial_port_base::flow_control(config.flow_control));
        serial_port.set_option(boost::asio::serial_port_base::parity(config.parity));
        serial_port.set_option(boost::asio::serial_port_base::stop_bits(config.stop_bits));
    }

    void RMSerialDriver::closePort() {
        boost::system::error_code error;
        serial_port.close(error);
        if (error) {
            RCLCPP_ERROR(get_logger(), "close failed: %s", error.message().c_str());
        }
    }

    void RMSerialDriver::reopenPort() {
        RCLCPP_WARN(get_logger(), "Attempting to reopen port");
        try {
            if (serial_port.is_open()) {
                closePort();
            }
            openPort();
            RCLCPP_INFO(get_logger(), "Successfully reopened port");
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
            if (rclcpp::ok()) {
                rclcpp::sleep_for(std::chrono::seconds(1));
                reopenPort();
            }
        }
    }

    void RMSerialDriver::getParams() {
        try {
            std::string device_name_ = declare_parameter<std::string>("device_name", "/tty/ACM0");
            config.device_name = device_name_;
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
            throw ex;
        }

        try {
            uint32_t baud_rate = declare_parameter<long>("baud_rate", 0);
            config.baud_rate = baud_rate;
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
            throw ex;
        }

        try {
            std::string flowControlString = declare_parameter<std::string>("flow_control", "");

            if (flowControlString == "none") {
                config.flow_control = boost::asio::serial_port_base::flow_control::none;
            } else if (flowControlString == "hardware") {
                config.flow_control = boost::asio::serial_port_base::flow_control::hardware;
            } else if (flowControlString == "software") {
                config.flow_control = boost::asio::serial_port_base::flow_control::software;
            } else {
                throw std::invalid_argument{"The flow_control parameter must be one of: none, software, or hardware."};
            }
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
            throw ex;
        }

        try {
            std::string parityString = declare_parameter<std::string>("parity", "");

            if (parityString == "none") {
                config.parity = boost::asio::serial_port_base::parity::none;
            } else if (parityString == "odd") {
                config.parity = boost::asio::serial_port_base::parity::odd;
            } else if (parityString == "even") {
                config.parity = boost::asio::serial_port_base::parity::even;
            } else {
                throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
            }
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
            throw ex;
        }

        try {
            std::string stopBitsString = declare_parameter<std::string>("stop_bits", "");

            if (stopBitsString == "1" || stopBitsString == "1.0") {
                config.stop_bits = boost::asio::serial_port_base::stop_bits::one;
            } else if (stopBitsString == "1.5") {
                config.stop_bits = boost::asio::serial_port_base::stop_bits::onepointfive;
            } else if (stopBitsString == "2" || stopBitsString == "2.0") {
                config.stop_bits = boost::asio::serial_port_base::stop_bits::two;
            } else {
                throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
            }
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
            throw ex;
        }
    }

}// namespace rm_serial_driver


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
