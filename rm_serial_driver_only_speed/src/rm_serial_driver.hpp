#pragma once

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <boost/asio.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

namespace rm_serial_driver {

    struct SerialPortConfig {
        std::string device_name = "/dev/ttyNav";
        uint32_t baud_rate = 115200;
        boost::asio::serial_port_base::flow_control::type flow_control = boost::asio::serial_port_base::flow_control::none;
        boost::asio::serial_port_base::parity::type parity = boost::asio::serial_port_base::parity::none;
        boost::asio::serial_port_base::stop_bits::type stop_bits = boost::asio::serial_port_base::stop_bits::one;
    };

    class RMSerialDriver : public rclcpp::Node {
        SerialPortConfig config;
        boost::asio::io_context io_context;
        boost::asio::basic_serial_port<boost::asio::io_context::executor_type> serial_port;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr action_sub_;//订阅要发送到底盘的信息

    public:
        explicit RMSerialDriver(const rclcpp::NodeOptions &options);

        ~RMSerialDriver() override;

    private:
        void getParams();

        void sendPacket(const geometry_msgs::msg::Twist::SharedPtr msg);// 通过串口发送数据包给下位机

        void openPort();

        void closePort();

        void reopenPort();
    };
}// namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
