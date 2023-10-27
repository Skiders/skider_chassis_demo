#pragma once

#include <memory>
#include <string>
#include <functional>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <skider_excutor/msg/sbus.hpp>
// #include <skider_excutor/msg/shooter_output.hpp>
#include <skider_excutor/msg/gimbal_command.hpp>
#include <skider_excutor/msg/chassis_state.hpp>
#include <skider_excutor/msg/chassis_command.hpp>
#include <skider_excutor/msg/gimbal_state.hpp>

#include <sensor_msgs/msg/imu.hpp>


#include "usbcdc_transporter.hpp"
#include "transport_package.h"
#include "can.hpp"

using namespace std::chrono_literals;

class GimbalHWNode
{
public:
    explicit GimbalHWNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() {
        return gimbal_hw_node_->get_node_base_interface();
    }
private:
    void loop_10000Hz();
    void loop_1000Hz();
    // void shooter_output_msg_callback(const skider_excutor::msg::ShooterOutput & msg);

    void gimbal_command_msg_callback(const skider_excutor::msg::GimbalCommand & msg);
    void chassis_command_msg_callback(const skider_excutor::msg::ChassisCommand & msg);



private:

    rclcpp::CallbackGroup::SharedPtr send_call_backgroup_;

    rclcpp::Node::SharedPtr gimbal_hw_node_;
    rclcpp::TimerBase::SharedPtr timer_10000Hz_;
    rclcpp::TimerBase::SharedPtr timer_1000Hz_;
    //
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_publisher_;
    
    rclcpp::Publisher<skider_excutor::msg::Sbus>::SharedPtr sbus_publisher_;

    rclcpp::Publisher<skider_excutor::msg::ChassisState>::SharedPtr chassis_state_publisher_;

    rclcpp::Publisher<skider_excutor::msg::GimbalState>::SharedPtr gimbal_state_publisher_;

    
    rclcpp::Subscription<skider_excutor::msg::GimbalCommand>::SharedPtr gimbal_command_subscription_;

    rclcpp::Subscription<skider_excutor::msg::ChassisCommand>::SharedPtr chassis_command_subscription_;

    // rclcpp::Subscription<skider_excutor::msg::ShooterOutput>::SharedPtr shooter_output_subscription_;

    std::shared_ptr<transporter_sdk::TransporterInterface> transporter_;

    // transport_package::GimbalHWTransmitPackage   transmit_package_;

    // rclcpp::TimerBase::SharedPtr gimbal_command_offline_timer_;
    // rclcpp::TimerBase::SharedPtr shooter_command_offline_timer_;


private:
    // protive topic timeout
    // bool gimbal_command_timeout_;
    // bool shooter_command_timeout_;

    std_msgs::msg::Header stamp_;

private:
    // params
    std::string imu_raw_publish_topic_name_;
    std::string sbus_publish_topic_name_;
    // std::string shooter_output_subscribe_topic_name_;
    int gimbal_interface_usb_vid_;
    int gimbal_interface_usb_pid_;
    int gimbal_interface_usb_read_endpoint_;
    int gimbal_interface_usb_write_endpoint_;
    int gimbal_interface_usb_read_timeout_;
    int gimbal_interface_usb_write_timeout_;

    transporter_sdk::Can can0_{transporter_sdk::Can(0)};
    transporter_sdk::Can can1_{transporter_sdk::Can(1)};

    // transporter_sdk::Can can0_(int 0);


    u_char buf_gimbal_[8];
    u_char buf_shooter_[8];
    u_char buf_chassis_[8];

public:
    // skider_excutor::msg::ChassisState chassis_state_msg_;
    double speed_[4];
    skider_excutor::msg::GimbalState gimbal_state_msg_;
    skider_excutor::msg::ChassisState chassis_state_msg_;

    

    
};
