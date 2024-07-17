/*
 * lrobot_base_ros.cpp
 *
 * Created on: Jun 10, 2024 13:23
 * Description:
 *
 * Copyright (c) 2024 LRobot Tzy.
 */

#include <rclcpp/rclcpp.hpp>
#include <lrobot_canopen/async_can.hpp>
#include <lrobot_base/lrobot_base_ros.hpp>

namespace LRobot
{
    LRobotBaseRos::LRobotBaseRos(std::string node_name) : rclcpp::Node(node_name), keep_running_(false)
    {
        std::cout << node_name << std::endl;
        this->declare_parameter("port_name", "can0");

        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("odom_topic_name", "odom");
        this->declare_parameter("control_rate", 50);

        LoadParameters();
    }

    void LRobotBaseRos::LoadParameters()
    {
        this->get_parameter_or<std::string>("port_name", port_name_, "can0");

        this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
        this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
        this->get_parameter_or<std::string>("odom_topic_name", odom_topic_name_,
                                            "odom");

        this->get_parameter_or<int>("control_rate", sim_control_rate_, 50);

        std::cout << "Loading parameters: " << std::endl;
        std::cout << "- port name: " << port_name_ << std::endl;
        std::cout << "- odom frame name: " << odom_frame_ << std::endl;
        std::cout << "- base frame name: " << base_frame_ << std::endl;
        std::cout << "- odom topic name: " << odom_topic_name_ << std::endl;
        std::cout << "- sim control rate: " << sim_control_rate_ << std::endl;
        std::cout << "----------------------------" << std::endl;
    }

    bool LRobotBaseRos::Initialize()
    {
        // connect Motor
    }

    void LRobotBaseRos::Stop() { keep_running_ = false; }

    void LRobotBaseRos::Run()
    {
        if (port_name_.find("can") != std::string::npos)
        {
            std::cout << "vcan0" << std::endl;
            // std::shared_ptr<AsyncCAN> can_;
            // using CANFrameRxCallback = AsyncCAN::ReceiveCallback;
            can_ = std::make_shared<AsyncCAN>(port_name_);
            // can_->Open();
            std::cout << "vcan0" << std::endl;
            if (can_->Open())
            {
                /* code */
                std::cout << "Using CAN bus to talk with the robot" << std::endl;
            }
            else
            {
                std::cout << "Failed to connect to the robot CAN bus" << std::endl;
                return;
            }
        }
        SetOdometryFrame(odom_frame_);
        SetBaseFrame(base_frame_);
        SetOdometryTopicName(odom_topic_name_);
        SetupSubscription();
        keep_running_ = true;
        SendSpSCommand(0x7);
        rclcpp::Rate rate(50);
        while (keep_running_)
        {
            rclcpp::spin_some(shared_from_this());
            rate.sleep();
        }
    }

    void LRobotBaseRos::SetupSubscription()
    {
        odom_pub_ =
            this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 50);

        // cmd subscriber
        motion_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 5,
            std::bind(&LRobotBaseRos::TwistCmdCallback, this,
                      std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    void LRobotBaseRos::TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
    {
        int16_t linear = (int16_t)(twist_msg->linear.x * 100);
        int16_t angular = (int16_t)(twist_msg->angular.z  * 2048);
        RCLCPP_INFO(this->get_logger(), "robot linear vel:%f",linear);
        RCLCPP_INFO(this->get_logger(),"robot angular vel:%f",twist_msg->angular.z);
        if (can_->IsOpened() && can_ != nullptr)
        {
            // if (linear != 0)
            // {
            can_frame tx_frame;
            tx_frame.can_id = CMD_ID;
            tx_frame.can_dlc = 8;
            MotionCommandLinearFrame mlframe;
            mlframe.func = 0x22;
            mlframe.subIndex = 0x2;
            mlframe.index_low_byte = (uint8_t)(CMD_SollVelProzentSPS & 0x00ff);
            mlframe.index_high_byte = (uint8_t)(CMD_SollVelProzentSPS >> 8);
            mlframe.linear_velocity_high_byte = (uint8_t)(linear >> 8);
            mlframe.linear_velocity_low_byte = (uint8_t)(linear & 0x00ff);
            mlframe.reserve_1 = 0x0;
            mlframe.reserve_2 = 0x0;
            memcpy(tx_frame.data, (uint8_t *)(&mlframe), tx_frame.can_dlc);
            can_->SendFrame(tx_frame);
            // }
            // if (angular!=0)
            // {
            /* code */
            //can_frame tx_frame;
            tx_frame.can_id = CMD_ID;
            tx_frame.can_dlc = 8;
            MotionCommandAngularFrame maframe;
            maframe.func = 0x22;
            maframe.subIndex = 0x1;
            maframe.index_low_byte = (uint8_t)(CMD_SollWinkelGradSPS & 0x00ff);
            maframe.index_high_byte = (uint8_t)(CMD_SollWinkelGradSPS >> 8);
            maframe.angular_velocity_high_byte = (uint8_t)(angular >> 8);
            maframe.angular_velocity_low_byte = (uint8_t)(angular & 0x00ff);
            maframe.reserve_1 = 0x0;
            maframe.reserve_2 = 0x0;
            memcpy(tx_frame.data, (uint8_t *)(&maframe), tx_frame.can_dlc);
            can_->SendFrame(tx_frame);
            // }
        }
    }
    ////Send Motion Command
    void LRobotBaseRos::SendMotionCommand(double linear_vel, double angular_vel)
    {
    }
    void LRobotBaseRos::SendSpSCommand(uint8_t cmd)
    {
        if (can_->IsOpened() && can_ != nullptr)
        {
            can_frame tx_frame;
            tx_frame.can_id = CMD_ID;
            tx_frame.can_dlc = 8;
            SpsCMDFrame frame;
            frame.func = 0x22;
            frame.index_low_byte = (uint8_t)(CMD_MyCmdSPS & 0x00ff);
            frame.index_high_byte = (uint8_t)(CMD_MyCmdSPS >> 8);
            frame.cmd = cmd;
            frame.subIndex = 0x1;
            frame.reserve_1 = 0x0;
            frame.reserve_2 = 0x0;
            frame.reserve_3 = 0x0;
            memcpy(tx_frame.data, (uint8_t *)(&frame), tx_frame.can_dlc);
            can_->SendFrame(tx_frame);
        }
    }

    bool LRobotBaseRos::EncodeCanFrame(LRobotmessage *msg, struct can_frame *tx_frame)
    {
        bool ret = true;
        switch (msg->type)
        {
        case LRobotMsgCmdSps:
            /* code */
            tx_frame->can_id = CMD_ID;
            tx_frame->can_dlc = 8;
            SpsCMDFrame frame;
            frame.func = 0x22;
            frame.index_low_byte = (uint8_t)(CMD_MyCmdSPS & 0x00ff);
            frame.index_high_byte = (uint8_t)(CMD_MyCmdSPS >> 8);
            frame.cmd = msg->body.mycmdsps_command_msg.cmd;
            frame.subIndex = 0x1;
            frame.reserve_1 = 0x0;
            frame.reserve_2 = 0x0;
            frame.reserve_3 = 0x0;
            memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
            break;
        case LRobotMsgMotionLinearCommand:
            std::cout << LRobotMsgMotionLinearCommand << std::endl;
            tx_frame->can_id = CMD_ID;
            tx_frame->can_dlc = 8;
            MotionCommandLinearFrame mframe;
            mframe.func = 0x22;
            mframe.subIndex = 0x2;
            mframe.index_low_byte = (uint8_t)(CMD_SollVelProzentSPS & 0x00ff);
            mframe.index_high_byte = (uint8_t)(CMD_SollVelProzentSPS >> 8);
            mframe.linear_velocity_high_byte = (uint8_t)(msg->body.motion_command_msg.linear >> 8);
            mframe.linear_velocity_low_byte = (uint8_t)(msg->body.motion_command_msg.linear & 0x00ff);
            mframe.reserve_1 = 0x0;
            mframe.reserve_2 = 0x0;
            memcpy(tx_frame->data, (uint8_t *)(&mframe), tx_frame->can_dlc);
            break;
        case LRobotMsgMotionAngularCommand:
            break;
        default:
            ret = false;
            break;
        }
        return ret;
    }
}