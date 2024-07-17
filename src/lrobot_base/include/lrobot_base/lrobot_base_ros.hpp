/*
 * lrobot_base_ros.hpp
 *
 * Created on: Jun 10, 2024 14:31
 * Description:
 *
 * Copyright (c) 2021 LRobot Tzy.
 */

#ifndef LROBOT_BASE_ROS_HPP
#define LROBOT_BASE_ROS_HPP

#include <memory.h>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <lrobot_canopen/async_can.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <lrobot_base/lrobot_message.h>
#include <lrobot_base/lrobot_protocol.hpp>
namespace LRobot
{
    class LRobotBaseRos : public rclcpp::Node
    {
    public:
        LRobotBaseRos(std::string node_name);
        bool Initialize();
        void Run();
        void Stop();

    private:
        std::shared_ptr<AsyncCAN> can_;
        std::string port_name_;
        std::string odom_frame_;
        std::string base_frame_;
        std::string odom_topic_name_;
        int sim_control_rate_ = 50;
        std::atomic<bool> keep_running_;
        geometry_msgs::msg::Twist current_twist_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        void LoadParameters();
        void SetOdometryFrame(std::string frame) { odom_frame_ = frame; }
        void SetBaseFrame(std::string frame) { base_frame_ = frame; }
        void SetOdometryTopicName(std::string name) { odom_topic_name_ = name; }
        void SetupSubscription();
        void SendMotionCommand(double linear_vel, double angular_vel);
        void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
        void SendSpSCommand(uint8_t cmd);
        bool EncodeCanFrame(LRobotmessage *msg,struct can_frame *tx_frame);
    };
}

#endif