#pragma once

#include <memory>
#include <string>
#include <functional>
#include <chrono>

#include <iostream>

#include <Eigen/Eigen>
#include <tf2/LinearMath/Matrix3x3.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>

// #include <skider_excutor/msg/shooter_output.hpp>
#include <skider_excutor/msg/gimbal_command.hpp>
#include <skider_excutor/msg/debug.hpp>
#include <skider_excutor/msg/imu.hpp>
#include <skider_excutor/msg/gimbal_state.hpp>

using namespace std::chrono_literals;

class Limit{

    public:
        Limit(){

        }
        Limit(double min, double max){

            max_ = max;
            min_ = min;
            out_ = 0;
        }
        
        double get(double in){
            
            if(out_+in < min_){
                out_ = min_;
            }
            else if(out_+in > max_){
                out_ = max_;
            }
            else{
                out_ += in;
            }
            return out_;
        }
    private:
        double max_, min_;
        double out_;

};

class PID{

    public:
        PID(){
            
        }
        PID(double kp, double ki, double kd){

            kp_ = kp;
            ki_ = ki;
            kd_ = kd;
            err_ = 0;
            err_last_ = 0;
            output_ = 0;

        }
        double calculate(double in, double state_now){

            err_ = in - state_now;
            //std::cout<<"err_: "<<err_<<std::endl;
            double i_sum_limit = i_sum_limit_.get(err_);
            //std::cout<<"i_sum_limit: "<<i_sum_limit<<std::endl;
            output_ = kp_*err_ + ki_*i_sum_limit + kd_*(err_-err_last_);

            err_last_ = err_;

            return output_;
        }
        Limit i_sum_limit_;

    private:
        double kp_, ki_, kd_;
        double err_, err_last_;
        double output_;

};

class GimbalControlerDemoNode
{
public:
    explicit GimbalControlerDemoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() {
        return gimbal_controler_demo_node_->get_node_base_interface();
    }

private:
    void joy_msg_callback(const sensor_msgs::msg::Joy & msg);
    void imu_msg_callback(const skider_excutor::msg::Imu & msg);
    void gimbal_msg_callback(const skider_excutor::msg::GimbalState & msg);
    
private:
    rclcpp::Node::SharedPtr gimbal_controler_demo_node_;
    rclcpp::Subscription<skider_excutor::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    // rclcpp::Publisher<skider_excutor::msg::ShooterOutput>::SharedPtr shooter_output_publisher_;
    rclcpp::Publisher<skider_excutor::msg::GimbalCommand>::SharedPtr gimbal_command_publisher_;
    rclcpp::Subscription<skider_excutor::msg::GimbalState>::SharedPtr gimbal_state_subscription_;

    rclcpp::Publisher<skider_excutor::msg::Debug>::SharedPtr debug_publisher_;
    skider_excutor::msg::Debug debug_msg_;


private:
    double imu_yaw_;
    double imu_pitch_;
    double imu_roll_;
    
    double yaw_angle_set_;
    double pitch_angle_set_;

    PID pid_yaw_in_, pid_yaw_out_;
    PID pid_pitch_in_, pid_pitch_out_;
    PID pid_ammor_, pid_ammol_, pid_rotor_;

    std::vector<double> pid_yaw_in_params_, pid_yaw_out_params_;
    std::vector<double> pid_pitch_in_params_, pid_pitch_out_params_;
    std::vector<double> pid_ammor_params_, pid_ammol_params_, pid_rotor_params_;

    double w_pitch_;
    double w_yaw_;
    double ammo_goal_speed_, rotor_goal_speed_;

    //gimbal state feedback
    double yaw_angle_, pitch_angle_;
    double ammor_speed_, ammol_speed_, rotor_speed_;



    // double yaw_in_kp_, yaw_in_ki_, yaw_in_kd_;
    // double yaw_out_kp_, yaw_out_ki_, yaw_out_kd_;

    // double pitch_in_kp_, pitch_in_ki_, pitch_in_kd_;
    // double pitch_out_kp_, pitch_out_ki_, pitch_out_kd_;

public:
    skider_excutor::msg::GimbalCommand gimbal_command_msg_;


    

};

