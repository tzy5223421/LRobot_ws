/*
 * lrobot_system.hpp
 *
 * Created on: Jun 10, 2024 14:31
 * Description:
 *
 * Copyright (c) 2021 LRobot Tzy.
 */

#ifndef LROBOT_SYSTEM_HPP
#define LROBOT_SYSTEM_HPP
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visibility_control.h"
#include "canopen_402_driver/cia402_driver.hpp"
#include "canopen_core/configuration_manager.hpp"
#include "canopen_core/device_container.hpp"
#include "lrobot_control/cia402_data.hpp"

namespace lrobot_control
{
    class LRobotSystemHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(LRobotSystemHardware)

        LROBOT_CONTROL_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        LROBOT_CONTROL_PUBLIC
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        LROBOT_CONTROL_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        LROBOT_CONTROL_PUBLIC
        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        LROBOT_CONTROL_PUBLIC
        hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) override;

        LROBOT_CONTROL_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        LROBOT_CONTROL_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        LROBOT_CONTROL_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        // LROBOT_CONTROL_PUBLIC
        // hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State previous_state) override;

        LROBOT_CONTROL_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        LROBOT_CONTROL_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        LROBOT_CONTROL_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        std::shared_ptr<ros2_canopen::DeviceContainer> device_container_;
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
        std::vector<Cia402Data> robot_motor_data_;
        std::string bus_config_;
        std::string master_config_;
        std::string master_bin_;
        std::string can_interface_;
        std::unique_ptr<std::thread> spin_thread_;
        std::unique_ptr<std::thread> init_thread_;

        rclcpp::Logger robot_system_logger = rclcpp::get_logger("LRobotSystemHardware");

        std::vector<int> node_id_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_effort_;
        std::vector<double> hw_commands_positions_;
        std::vector<double> hw_commands_velocities_;
        std::vector<double> hw_commands_efforts_;
        std::shared_ptr<rclcpp::Node> node_;
        bool subscriber_is_active_ = false;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr fb_subscriber_ = nullptr;
        realtime_tools::RealtimeBox<std::shared_ptr<std_msgs::msg::Float64MultiArray>> received_fb_msg_ptr_{nullptr};
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> realtime_cmd_publisher_ = nullptr;
        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> cmd_publisher = nullptr;

        void spin();
        void clean();

        /**
         * @brief Initialize the device container
         *
         */
        void initDeviceContainer();
    };
} // namespace LRobotControll

#endif