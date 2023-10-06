#include "gimbal_demo_node.hpp"

GimbalControlerDemoNode::GimbalControlerDemoNode(const rclcpp::NodeOptions & options)
{
    gimbal_controler_demo_node_ = std::make_shared<rclcpp::Node>("gimbal_controler_node", options);
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Node Begin");

    std::map<std::string, double> pid_params {
        {"yaw_in_kp", 0.0},
        {"yaw_in_ki", 0.0},
        {"yaw_in_kd", 0.0},
        {"yaw_out_kp", 0.0},
        {"yaw_out_ki", 0.0},
        {"yaw_out_kd", 0.0},
        {"pitch_in_kp", 0.0},
        {"pitch_in_ki", 0.0},
        {"pitch_in_kd", 0.0},
        {"pitch_out_kp", 0.0},
        {"pitch_out_ki", 0.0},
        {"pitch_out_kd", 0.0},
    };

    gimbal_controler_demo_node_->declare_parameters("", pid_params);
    // pid parameter
    gimbal_controler_demo_node_->get_parameter<double>("yaw_in_kp", yaw_in_kp_);
    gimbal_controler_demo_node_->get_parameter<double>("yaw_in_ki", yaw_in_ki_);
    gimbal_controler_demo_node_->get_parameter<double>("yaw_in_kd", yaw_in_kd_);
    gimbal_controler_demo_node_->get_parameter<double>("yaw_out_kp", yaw_out_kp_);
    gimbal_controler_demo_node_->get_parameter<double>("yaw_out_ki", yaw_out_ki_);
    gimbal_controler_demo_node_->get_parameter<double>("yaw_out_kd", yaw_out_kd_);
    gimbal_controler_demo_node_->get_parameter<double>("pitch_in_kp", pitch_in_kp_);
    gimbal_controler_demo_node_->get_parameter<double>("pitch_in_ki", pitch_in_ki_);
    gimbal_controler_demo_node_->get_parameter<double>("pitch_in_kd", pitch_in_kd_);
    gimbal_controler_demo_node_->get_parameter<double>("pitch_out_kp", pitch_out_kp_);
    gimbal_controler_demo_node_->get_parameter<double>("pitch_out_ki", pitch_out_ki_);
    gimbal_controler_demo_node_->get_parameter<double>("pitch_out_kd", pitch_out_kd_);

    std::cout<<"yaw_in_kp_: "<<yaw_in_kp_<<std::endl;
    std::cout<<"yaw_in_ki_: "<<yaw_in_ki_<<std::endl;
    std::cout<<"yaw_in_kd_: "<<yaw_in_kd_<<std::endl;
    std::cout<<"pitch_in_kp_: "<<pitch_in_kp_<<std::endl;
    std::cout<<"pitch_in_ki_: "<<pitch_in_ki_<<std::endl;
    std::cout<<"pitch_in_kd_: "<<pitch_in_kd_<<std::endl;


    std::string imu_subscribe_topic_name_("/skider/imu/data");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Subscribe IMU data : \"%s\"", imu_subscribe_topic_name_.c_str());
    imu_subscription_ = gimbal_controler_demo_node_->create_subscription<skider_excutor::msg::Imu>(
        imu_subscribe_topic_name_, 10, std::bind(&GimbalControlerDemoNode::imu_msg_callback, this, std::placeholders::_1));

    std::string joy_subscribe_topic_name_("/skider/joy/data");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Subscribe JOY data : \"%s\"", joy_subscribe_topic_name_.c_str());
    joy_subscription_ = gimbal_controler_demo_node_->create_subscription<sensor_msgs::msg::Joy>(
        joy_subscribe_topic_name_, 10, std::bind(&GimbalControlerDemoNode::joy_msg_callback, this, std::placeholders::_1));


    std::string shooter_output_publish_topic_name_("/skider/output/shooter");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Init Gimbal Output Publisher : \"%s\"", shooter_output_publish_topic_name_.c_str());
    shooter_output_publisher_ = gimbal_controler_demo_node_->create_publisher<skider_excutor::msg::ShooterOutput>(
        shooter_output_publish_topic_name_, 10);

    std::string gimbal_command_publish_topic_name_("/skider/command/gimbal");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Init Gimbal Command Publisher : ");
    gimbal_command_publisher_ = gimbal_controler_demo_node_->create_publisher<skider_excutor::msg::GimbalCommand>(
        gimbal_command_publish_topic_name_, 10);

    std::string gimbal_debug_publisg_topic_name_("/skider/debug");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Init Debug Publisher : ");
    debug_publisher_ = gimbal_controler_demo_node_->create_publisher<skider_excutor::msg::Debug>(
        gimbal_debug_publisg_topic_name_, 10);

    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Finish Init");

    yaw_angle_set_ = 0;
    pitch_angle_set_ = 0;

    this->pid_yaw_in_ = PID(yaw_in_kp_, yaw_in_ki_, yaw_in_kd_);
    this->pid_yaw_in_.i_sum_limit_ = Limit(-5000, 5000) ;

    this->pid_yaw_out_ = PID(yaw_out_kp_, yaw_out_ki_, yaw_out_kd_);
    this->pid_yaw_out_.i_sum_limit_ = Limit(-5000, 5000) ;

    this->pid_pitch_in_ = PID(pitch_in_kp_, pitch_in_ki_, pitch_in_kd_);
    this->pid_pitch_in_.i_sum_limit_ = Limit(-5000, 5000) ;

    this->pid_pitch_out_ = PID(pitch_out_kp_, pitch_out_ki_, pitch_out_kd_);
    this->pid_pitch_out_.i_sum_limit_ = Limit(-5000, 5000) ;
}

inline double speed_limit(double input, double max)
{
    if (input > max) {
        return max;
    }
    else if (input < -max) {
        return -max;
    }
    return input;
}

inline double get_relative_angle(double angle_aim, double angle_ref)
{
    double reletive_angle = angle_aim - angle_ref;

    while (reletive_angle > M_PI) {
        reletive_angle -= 2*M_PI;
    }
    while (reletive_angle < -M_PI) {
        reletive_angle += 2*M_PI;
    }

    return reletive_angle;
}

inline double aim_loop(double angle_aim)
{
    while (angle_aim > M_PI) {
        angle_aim -= 2*M_PI;
    }
    while (angle_aim < -M_PI) {
        angle_aim += 2*M_PI;
    }

    return angle_aim;
}

inline double aim_limut(double angle_aim, double max, double min)
{
    while (angle_aim > max) {
        return max;
    }
    while (angle_aim < min) {
        return min;
    }

    return angle_aim;
}



void GimbalControlerDemoNode::joy_msg_callback(const sensor_msgs::msg::Joy & msg)
{
    std::cout<<"pitch_in_kp_: "<<pitch_in_kp_<<std::endl;
    std::cout<<"pitch_in_ki_: "<<pitch_in_ki_<<std::endl;
    std::cout<<"pitch_in_kd_: "<<pitch_in_kd_<<std::endl;

    skider_excutor::msg::GimbalCommand gimbal;
    gimbal.header.set__frame_id("Controler Gimbal Command");
    gimbal.header.set__stamp(gimbal_controler_demo_node_->get_clock()->now());
    if ((msg.buttons[1] == true)  ||  (msg.buttons[2] == true)){        //(msg.buttons[1] == true)  ||  (msg.buttons[2] == true)

        std::cout<<"calculating: "<<std::endl;

        yaw_angle_set_ = aim_loop(yaw_angle_set_ + (-msg.axes[2])*0.1);
        double yaw_relative = get_relative_angle(yaw_angle_set_, imu_yaw_);
        yaw_angle_set_ = imu_yaw_ + yaw_relative;
        
        double yaw_w_goal = this->pid_yaw_out_.calculate(yaw_angle_set_, imu_yaw_);
        double yaw_current = this->pid_yaw_in_.calculate(yaw_w_goal, w_yaw_);
        // std::cout<<yaw_w_goal<<"\t"<<w_yaw_<<"\t"<<yaw_current<<std::endl;
        gimbal.yaw_current = (int16_t)((int)(speed_limit(yaw_current, 30000)));

        

        pitch_angle_set_ = aim_limut((pitch_angle_set_ + (msg.axes[3])*0.03), 0.25, -0.4);
        double pitch_w_goal = this->pid_pitch_out_.calculate(pitch_angle_set_, imu_pitch_);
        double pitch_current = this->pid_pitch_in_.calculate(pitch_w_goal, w_pitch_);
        gimbal.pitch_current = (int16_t)(speed_limit(pitch_current, 30000));
        
        std::cout<<pitch_angle_set_<<"\t"<<pitch_w_goal<<"\t"<<w_pitch_<<"\t"<<pitch_current<<std::endl;
        
        //debug
        // debug_msg_.header.stamp = gimbal_controler_demo_node_->get_clock()->now();



    }
    else{

        gimbal.yaw_current = 0;
        gimbal.pitch_current = 0;

    }
    // debug_publisher_->publish(debug_msg_);


    gimbal_command_publisher_->publish(gimbal);


    skider_excutor::msg::ShooterOutput shooter;
    shooter.header.set__frame_id("Controler Demo Shooter Output");
    shooter.header.set__stamp(gimbal_controler_demo_node_->get_clock()->now());
    if (msg.buttons[2] == true) {
        shooter.set__enable(true);
        shooter.set__rotor_kp(10);
        shooter.set__ammol_kp(20);
        shooter.set__ammor_kp(20);
        shooter.set__rotor_speed(-msg.axes[4]*10000);
        shooter.set__ammol_speed(-5000);
        shooter.set__ammor_speed(5000);
    }
    else {
        shooter.set__enable(false);
        shooter.set__rotor_kp(0);
        shooter.set__ammol_kp(0);
        shooter.set__ammor_kp(0);
        shooter.set__rotor_speed(0);
        shooter.set__ammol_speed(0);
        shooter.set__ammor_speed(0);
    }
    shooter_output_publisher_->publish(shooter);
}


void GimbalControlerDemoNode::imu_msg_callback(const skider_excutor::msg::Imu & msg)
{

    imu_yaw_ = msg.imu_yaw;
    imu_pitch_ = msg.imu_pitch;
    imu_roll_ = msg.imu_roll;

    w_pitch_ = msg.imu.angular_velocity.y;
    w_yaw_ = msg.imu.angular_velocity.z;

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto gimbal_controler_demo_node = std::make_shared<GimbalControlerDemoNode>();
    rclcpp::spin(gimbal_controler_demo_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}


