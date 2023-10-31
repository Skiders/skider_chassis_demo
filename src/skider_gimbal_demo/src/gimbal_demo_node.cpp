#include "gimbal_demo_node.hpp"

GimbalControlerDemoNode::GimbalControlerDemoNode(const rclcpp::NodeOptions & options)
{
    gimbal_controler_demo_node_ = std::make_shared<rclcpp::Node>("gimbal_controler_node", options);
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Node Begin");


    
    //kp, ki, kd
    std::map<std::string, std::vector<double>> pid_params {
        {"pid_yaw_remote_in", {0.0, 0.0, 0,0}},
        {"pid_yaw_remote_out", {0.0, 0.0, 0.0}},
        {"pid_yaw_init_in", {0.0, 0.0, 0,0}},
        {"pid_yaw_init_out", {0.0, 0.0, 0.0}},
        {"pid_pitch_remote_in", {0.0, 0.0, 0.0}},
        {"pid_pitch_remote_out", {0.0, 0.0, 0.0}},
        {"pid_ammor", {0.0, 0.0, 0.0}},
        {"pid_ammol", {0.0, 0.0, 0.0}},
        {"pid_rotor", {0.0, 0.0, 0.0}},

    };
    gimbal_controler_demo_node_->declare_parameters("", pid_params);
    // pid parameter



    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_yaw_remote_in", pid_yaw_remote_in_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_yaw_remote_out", pid_yaw_remote_out_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_yaw_init_in", pid_yaw_init_in_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_yaw_init_out", pid_yaw_init_out_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_pitch_remote_in", pid_pitch_remote_in_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_pitch_remote_out", pid_pitch_remote_out_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_ammor", pid_ammor_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_ammol", pid_ammol_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_rotor", pid_rotor_params_);

    std::cout<<" pid_yaw_remote_in_params_[0]: "<<pid_yaw_remote_in_params_[0]<<" pid_yaw_remote_in_params_[1]: "<<pid_yaw_remote_in_params_[1]<<" pid_yaw_remote_in_params_[2]: "<<pid_yaw_remote_in_params_[2]<<std::endl;

    std::string imu_subscribe_topic_name_("/skider/imu/data");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Subscribe IMU data : \"%s\"", imu_subscribe_topic_name_.c_str());
    imu_subscription_ = gimbal_controler_demo_node_->create_subscription<skider_excutor::msg::Imu>(
        imu_subscribe_topic_name_, 10, std::bind(&GimbalControlerDemoNode::imu_msg_callback, this, std::placeholders::_1));

    std::string joy_subscribe_topic_name_("/skider/joy/data");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Subscribe JOY data : \"%s\"", joy_subscribe_topic_name_.c_str());
    joy_subscription_ = gimbal_controler_demo_node_->create_subscription<sensor_msgs::msg::Joy>(
        joy_subscribe_topic_name_, 10, std::bind(&GimbalControlerDemoNode::joy_msg_callback, this, std::placeholders::_1));


    // std::string shooter_output_publish_topic_name_("/skider/output/shooter");
    // RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Init Gimbal Output Publisher : \"%s\"", shooter_output_publish_topic_name_.c_str());
    // shooter_output_publisher_ = gimbal_controler_demo_node_->create_publisher<skider_excutor::msg::ShooterOutput>(
    //     shooter_output_publish_topic_name_, 10);

    std::string gimbal_command_publish_topic_name_("/skider/command/gimbal");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Init Gimbal Command Publisher : ");
    gimbal_command_publisher_ = gimbal_controler_demo_node_->create_publisher<skider_excutor::msg::GimbalCommand>(
        gimbal_command_publish_topic_name_, 10);

    std::string gimbal_debug_publisg_topic_name_("/skider/debug");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Init Debug Publisher : ");
    debug_publisher_ = gimbal_controler_demo_node_->create_publisher<skider_excutor::msg::Debug>(
        gimbal_debug_publisg_topic_name_, 10);

    gimbal_state_subscription_ = gimbal_controler_demo_node_->create_subscription<skider_excutor::msg::GimbalState>(
        "/skider/gimbal_state", 10, std::bind(&GimbalControlerDemoNode::gimbal_msg_callback, this, std::placeholders::_1));

    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Finish Init");

    yaw_angle_set_ = 0;
    pitch_angle_set_ = 0;

    //TOCHECK
    ammo_goal_speed_ = 7000;
    rotor_goal_speed_ = 2000;


    // this->pid_yaw_remote_in_.i_sum_limit_ = Limit(-5000, 5000) ;


    this->pid_yaw_remote_in_ = PID(pid_yaw_remote_in_params_[0], pid_yaw_remote_in_params_[1], pid_yaw_remote_in_params_[2]);
    this->pid_yaw_remote_in_.i_sum_limit_ = Limit(-5000, 5000) ;
    this->pid_yaw_remote_out_ = PID(pid_yaw_remote_out_params_[0], pid_yaw_remote_out_params_[1], pid_yaw_remote_out_params_[2]);
    this->pid_yaw_init_in_ = PID(pid_yaw_init_in_params_[0], pid_yaw_init_in_params_[1], pid_yaw_init_in_params_[2]);
    this->pid_yaw_init_out_ = PID(pid_yaw_init_out_params_[0], pid_yaw_init_out_params_[1], pid_yaw_init_out_params_[2]);
    this->pid_pitch_remote_in_ = PID(pid_pitch_remote_in_params_[0], pid_pitch_remote_in_params_[1], pid_pitch_remote_in_params_[2]);
    this->pid_pitch_remote_out_ = PID(pid_pitch_remote_out_params_[0], pid_pitch_remote_out_params_[1], pid_pitch_remote_out_params_[2]);
    this->pid_ammor_ = PID(pid_ammor_params_[0], pid_ammor_params_[1], pid_ammor_params_[2]);
    this->pid_ammol_ = PID(pid_ammol_params_[0], pid_ammol_params_[1], pid_ammol_params_[2]);
    this->pid_rotor_ = PID(pid_rotor_params_[0], pid_rotor_params_[1], pid_rotor_params_[2]);

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

inline double get_relative_angle(double angle_aim, double angle_ref, int type)
{
    double reletive_angle = angle_aim - angle_ref;

    //弧度
    if(type == 1){
        while (reletive_angle > M_PI) {
            reletive_angle -= 2*M_PI;
        }
        while (reletive_angle < -M_PI) {
            reletive_angle += 2*M_PI;
        }
    }

    //机械角度
    if(type == 2){

        while (reletive_angle > 4096) {
            reletive_angle -= 2*4096;
        }
        while (reletive_angle < -4096) {
            reletive_angle += 2*4096;
        }

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

    //std::cout<<gimbal_controler_demo_node_->get_clock()->now().nanoseconds()<<std::endl;

    gimbal_command_msg_.header.set__frame_id("Controler Gimbal Command");
    gimbal_command_msg_.header.set__stamp(gimbal_controler_demo_node_->get_clock()->now());

    //云台有力
    if ((msg.buttons[1] == true)  ||  (msg.buttons[2] == true)){

        if(follow_init_ != true){

            //云台转到底盘处
            double yaw_relative = get_relative_angle(yaw_zero_angle_, yaw_angle_, 2);
            double yaw_init = yaw_angle_ + yaw_relative;

            double yaw_w_goal = this->pid_yaw_init_out_.calculate(yaw_init, yaw_angle_);
            double yaw_current = this->pid_yaw_init_in_.calculate(yaw_w_goal, w_yaw_);
            // std::cout<<"yaw_init: "<<yaw_init<<std::endl;
            // std::cout<<"yaw_angle_: "<<yaw_angle_<<std::endl;
            // std::cout<<"yaw_w_goal: "<<yaw_w_goal<<std::endl;
            // std::cout<<"w_yaw_: "<<w_yaw_<<std::endl;
            // std::cout<<"yaw_current: "<<yaw_current<<std::endl;

            std::cout<<"yaw_relative: "<<yaw_relative<<std::endl;
            std::cout<<"follow_init_: "<<follow_init_<<std::endl;

            gimbal_command_msg_.yaw_current = (int16_t)((int)(speed_limit(yaw_current, 30000)));


            //相差角度小于一定值，初始化完成
            if(yaw_relative > -50 && yaw_relative < 50){
                
                //以当前imu为相对零点
                yaw_angle_set_= imu_yaw_ ;
                follow_init_ = true;

            }
        }
        else{



            yaw_angle_set_ = aim_loop(yaw_angle_set_ + (-msg.axes[2])*0.01);
            //std::cout<<"yaw_angle_set_: "<<yaw_angle_set_<<std::endl;

            double yaw_relative = get_relative_angle(yaw_angle_set_, imu_yaw_, 1);
            yaw_angle_set_ = imu_yaw_   + yaw_relative;
            
            double yaw_w_goal = this->pid_yaw_remote_out_.calculate(yaw_angle_set_, imu_yaw_);
            double yaw_current = this->pid_yaw_remote_in_.calculate(yaw_w_goal, w_yaw_);
            //std::cout<<yaw_w_goal<<"\t"<<w_yaw_<<"\t"<<yaw_current<<std::endl;
            gimbal_command_msg_.yaw_current = (int16_t)((int)(speed_limit(yaw_current, 30000)));

        }
            

        pitch_angle_set_ = aim_limut((pitch_angle_set_ + (msg.axes[3])*0.03), 0.25, -0.4);
        double pitch_w_goal = this->pid_pitch_remote_out_.calculate(pitch_angle_set_, imu_pitch_);
        double pitch_current = this->pid_pitch_remote_in_.calculate(pitch_w_goal, w_pitch_);
        gimbal_command_msg_.pitch_current = (int16_t)(speed_limit(pitch_current, 30000));
            
        //std::cout<<pitch_angle_set_<<"\t"<<pitch_w_goal<<"\t"<<w_pitch_<<"\t"<<pitch_current<<std::endl;
        
        //debug
        // debug_msg_.header.stamp = gimbal_controler_demo_node_->get_clock()->now();



    }


    //摩擦轮转动TOCHECK
    if(msg.buttons[2] == true){

        //需要当前转速
        gimbal_command_msg_.ammor_current = this->pid_ammor_.calculate(ammo_goal_speed_, ammor_speed_);
        //std::cout<<ammo_goal_speed_<<"\t"<<ammor_speed_<<"\t"<<ammol_speed_<<"\t"<<std::endl;

        gimbal_command_msg_.ammol_current = this->pid_ammol_.calculate(-ammo_goal_speed_, ammol_speed_);


    }
    //rotor TOCHECK
    if(msg.axes[4] > 0.9f && msg.buttons[2] == true){
        
        std::cout<<111<<std::endl;

        gimbal_command_msg_.rotor_current = this->pid_rotor_.calculate(rotor_goal_speed_, rotor_speed_);
    }
    else if(msg.buttons[0] != true){

            gimbal_command_msg_.rotor_current = 0;//this->pid_rotor_.calculate(0, rotor_speed_);
    }


    if(msg.buttons[0] == true){

        gimbal_command_msg_.yaw_current = 0;
        gimbal_command_msg_.pitch_current = 0;
        gimbal_command_msg_.ammor_current = 0;
        gimbal_command_msg_.ammol_current = 0;
        gimbal_command_msg_.rotor_current = 0;

        follow_init_ = false;

    }
    // debug_publisher_->publish(debug_msg_);


    gimbal_command_publisher_->publish(gimbal_command_msg_);
    gimbal_command_msg_.yaw_current = 0;
    gimbal_command_msg_.pitch_current = 0;
    gimbal_command_msg_.ammor_current = 0;
    gimbal_command_msg_.ammol_current = 0;
    gimbal_command_msg_.rotor_current = 0;

}


void GimbalControlerDemoNode::imu_msg_callback(const skider_excutor::msg::Imu & msg)
{

    imu_yaw_ = msg.imu_yaw;
    imu_pitch_ = msg.imu_pitch;
    imu_roll_ = msg.imu_roll;

    w_pitch_ = msg.imu.angular_velocity.y;
    w_yaw_ = msg.imu.angular_velocity.z;

}

void GimbalControlerDemoNode::gimbal_msg_callback(const skider_excutor::msg::GimbalState & msg)
{

    //不用 
    yaw_angle_ = msg.yaw_angle;
    pitch_angle_ = msg.pitch_angle;
    
    
    ammor_speed_ = msg.ammor_speed;
    ammol_speed_ = msg.ammol_speed;
    rotor_speed_ = msg.rotor_speed;
    // std::cout<<"yaw_angle_: "<<yaw_angle_<<std::endl;


}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto gimbal_controler_demo_node = std::make_shared<GimbalControlerDemoNode>();
    rclcpp::spin(gimbal_controler_demo_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}


