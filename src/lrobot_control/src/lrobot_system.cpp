#include <lrobot_control/lrobot_system.hpp>
#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
    constexpr auto DEFAULT_COMMAND_TOPIC = "lrobot_cmd";
    constexpr auto DEFAULT_STATE_TOPIC = "lrobot_cmd";
}

namespace lrobot_control
{
    hardware_interface::CallbackReturn LRobotSystemHardware::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        robot_system_logger = rclcpp::get_logger(info_.name + "_interface");
        RCLCPP_INFO(robot_system_logger, "Registering hardware interface '%s'", info_.name.c_str());

        // Check bus config is specified.
        if (info_.hardware_parameters.find("bus_config") == info_.hardware_parameters.end())
        {
            RCLCPP_ERROR(
                robot_system_logger, "No bus_config parameter provided for '%s' hardware interface.",
                info_.name.c_str());
            return CallbackReturn::ERROR;
        }
        bus_config_ = info_.hardware_parameters["bus_config"];
        RCLCPP_INFO(
            robot_system_logger, "'%s' has bus config: '%s'", info_.name.c_str(), bus_config_.c_str());

        // Check master config is specified.
        if (info_.hardware_parameters.find("master_config") == info_.hardware_parameters.end())
        {
            RCLCPP_ERROR(
                robot_system_logger, "No master_config parameter provided for '%s' hardware interface.",
                info_.name.c_str());
            return CallbackReturn::ERROR;
        }
        master_config_ = info_.hardware_parameters["master_config"];
        RCLCPP_INFO(
            robot_system_logger, "'%s' has master config: '%s'", info_.name.c_str(),
            master_config_.c_str());

        // Check master bin is specified.
        if (info_.hardware_parameters.find("master_bin") != info_.hardware_parameters.end())
        {
            master_bin_ = info_.hardware_parameters["master_bin"];
            if (master_bin_ == "\"\"")
            {
                master_bin_ = "";
            }
            RCLCPP_INFO(
                robot_system_logger, "'%s' has master bin: '%s'", info_.name.c_str(), master_bin_.c_str());
        }
        else
        {
            master_bin_ = "";
        }

        // Check can_interface_name is specified.
        if (info_.hardware_parameters.find("can_interface_name") == info_.hardware_parameters.end())
        {
            RCLCPP_ERROR(
                robot_system_logger, "No can_interface_name parameter provided for '%s' hardware interface.",
                info_.name.c_str());
            return CallbackReturn::ERROR;
        }
        can_interface_ = info_.hardware_parameters["can_interface_name"];
        RCLCPP_INFO(
            robot_system_logger, "'%s' has can interface: '%s'", info_.name.c_str(),
            can_interface_.c_str());

        ros2_canopen::ConfigurationManager config(bus_config_);
        config.init_config();

        // Load joint data
        for (auto joint : info.joints)
        {
            RCLCPP_INFO(robot_system_logger, "Init data joint '%s'", joint.name.c_str());
            auto driver_type =
                config.get_entry<std::string>(joint.parameters["device_name"], "driver").value();
            if (driver_type == "ros2_canopen::Cia402Driver")
            {
                auto data = Cia402Data();
                if (data.init_data(joint, config.dump_device(joint.parameters["device_name"])))
                {
                    robot_motor_data_.push_back(data);
                }
                else
                {
                    robot_motor_data_.push_back(data);
                }
            }
            else
            {
                RCLCPP_ERROR(
                    robot_system_logger, "Driver type '%s' not supported for joint '%s'", driver_type.c_str(),
                    joint.name.c_str());
                return CallbackReturn::ERROR;
            }
        }
       // control_level_.resize(info_.joints.size(), integration_level_t::UNDEFINED);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn LRobotSystemHardware::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {
        executor_ =
            std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
        device_container_ = std::make_shared<ros2_canopen::DeviceContainer>(executor_);
        executor_->add_node(device_container_);

        spin_thread_ = std::make_unique<std::thread>(&LRobotSystemHardware::spin, this);
        init_thread_ = std::make_unique<std::thread>(&LRobotSystemHardware::initDeviceContainer, this);

        if (init_thread_->joinable())
        {
            init_thread_->join();
        }
        else
        {
            RCLCPP_ERROR(robot_system_logger, "Could not join init thread!");
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> LRobotSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (lrobot_control::Cia402Data &data : robot_motor_data_)
        {
            data.export_state_interface(state_interfaces);
        }
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> LRobotSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (lrobot_control::Cia402Data &data : robot_motor_data_)
        {
            data.export_command_interface(command_interfaces);
        }
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
        }
        return command_interfaces;
    }
    hardware_interface::CallbackReturn LRobotSystemHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(rclcpp::get_logger("LRobotSystemHardware"), "Activating ...please wait...");
        // END: This part here is for exemplary purposes - Please do not copy to your production code
        for (auto &data : robot_motor_data_)
        {
            if (!data.driver->init_motor())
            {
                RCLCPP_ERROR(robot_system_logger, "Failed to activate '%s'", data.joint_name.c_str());
                return CallbackReturn::FAILURE;
            }
        }

        // set some default values
        for (auto i = 0u; i < hw_positions_.size(); i++)
        {
            if (std::isnan(hw_positions_[i]))
            {
                hw_positions_[i] = 0;
                hw_velocities_[i] = 0;
                hw_commands_velocities_[i] = 0;
            }
        }
        subscriber_is_active_ = true;

        this->node_ = std::make_shared<rclcpp::Node>("hardware_node");

        std_msgs::msg::Float64MultiArray empty_int16array;
        for (std::size_t i = 0; i < hw_positions_.size(); i++)
        {
            empty_int16array.data.push_back(0.0);
        }
        received_fb_msg_ptr_.set(std::make_shared<std_msgs::msg::Float64MultiArray>(empty_int16array));

        fb_subscriber_ =
            this->node_->create_subscription<std_msgs::msg::Float64MultiArray>(
                DEFAULT_STATE_TOPIC, rclcpp::SystemDefaultsQoS(),
                [this](const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) -> void
                {
                    if (!subscriber_is_active_)
                    {
                        RCLCPP_WARN(
                            this->node_->get_logger(), "Can't accept new commands. subscriber is inactive");
                        return;
                    }
                    received_fb_msg_ptr_.set(std::move(msg));
                });

        // 创建实时Publisher
        cmd_publisher = this->node_->create_publisher<std_msgs::msg::Float64MultiArray>(DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS());
        realtime_cmd_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(cmd_publisher);

        RCLCPP_INFO(rclcpp::get_logger("LRobotSystemHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn LRobotSystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(rclcpp::get_logger("LRobotSystemHardware"), "Deactivating ...please wait...");
        for (auto &data : robot_motor_data_)
        {
            if (!data.driver->halt_motor())
            {
                RCLCPP_ERROR(robot_system_logger, "Failed to deactivate '%s'", data.joint_name.c_str());
                return CallbackReturn::FAILURE;
            }
        }
        subscriber_is_active_ = false;
        fb_subscriber_.reset();
        received_fb_msg_ptr_.set(nullptr);
        // END: This part here is for exemplary purposes - Please do not copy to your production code

        RCLCPP_INFO(rclcpp::get_logger("LRobotSystemHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn LRobotSystemHardware::on_cleanup(
        const rclcpp_lifecycle::State &previous_state)
    {
        clean();
        return CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn LRobotSystemHardware::on_shutdown(
        const rclcpp_lifecycle::State &previous_state)
    {
        clean();
        return CallbackReturn::SUCCESS;
    }
    hardware_interface::return_type lrobot_control::LRobotSystemHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        std::shared_ptr<std_msgs::msg::Float64MultiArray> fb_msg;
        received_fb_msg_ptr_.get(fb_msg);
        rclcpp::spin_some(this->node_);
        for (std::size_t i = 0; i < hw_positions_.size(); i++)
        {
            // Update the joint status: this is a revolute joint without any limit.
            if (i < hw_velocities_.size())
            {
                hw_velocities_[i] = fb_msg->data[i];
                hw_positions_[i] += period.seconds() * hw_velocities_[i];
            }
        }
        for (lrobot_control::Cia402Data &data : robot_motor_data_)
        {
            data.read_state();
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type lrobot_control::LRobotSystemHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        for (lrobot_control::Cia402Data &data : robot_motor_data_)
        {
            data.write_target();
        }
        if (realtime_cmd_publisher_->trylock())
        {
            auto &cmd_msg = realtime_cmd_publisher_->msg_;
            cmd_msg.data.resize(hw_commands_velocities_.size());
            for (auto i = 0u; i < hw_commands_velocities_.size(); i++)
            {
                cmd_msg.data[i] = hw_commands_velocities_[i];
                //RCLCPP_INFO(rclcpp::get_logger("cmd_msg"), "cmd_msg:%f", cmd_msg.data[i]);
            }
            realtime_cmd_publisher_->unlockAndPublish();
        }

        return hardware_interface::return_type::OK;
    }
    
    hardware_interface::return_type LRobotSystemHardware::perform_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces)
    {
        // register interfaces to start per device
        for (auto interface : start_interfaces)
        {
            auto it = std::find_if(
                robot_motor_data_.begin(), robot_motor_data_.end(),
                [interface](Cia402Data &data)
                {
                    return std::find(data.interfaces.begin(), data.interfaces.end(), interface) !=
                           data.interfaces.end();
                });
            if (it != robot_motor_data_.end())
            {
                it->interfaces_to_start.push_back(
                    hardware_interface::CommandInterface(interface).get_interface_name());
            }
        }

        // register interfaces to stop per device
        for (auto interface : stop_interfaces)
        {
            auto it = std::find_if(
                robot_motor_data_.begin(), robot_motor_data_.end(),
                [interface](Cia402Data &data)
                {
                    return std::find(data.interfaces.begin(), data.interfaces.end(), interface) !=
                           data.interfaces.end();
                });
            if (it != robot_motor_data_.end())
            {
                it->interfaces_to_stop.push_back(
                    hardware_interface::CommandInterface(interface).get_interface_name());
            }
        }

        // perform switching
        for (auto &data : robot_motor_data_)
        {
            if (!data.perform_switch())
            {
                return hardware_interface::return_type::ERROR;
            }
        }
        return hardware_interface::return_type::OK;
    }
    void LRobotSystemHardware::initDeviceContainer()
    {
        // Init device container
        device_container_->init(
            this->can_interface_, this->master_config_, this->bus_config_, this->master_bin_);

        // Get all registered drivers.
        auto drivers = device_container_->get_registered_drivers();

        // Iterate over all drivers and allocate them to the correct joint.
        for (auto &data : robot_motor_data_)
        {
            // Find correct driver for joint via node id.
            auto driver = std::find_if(
                drivers.begin(), drivers.end(),
                [&data](const std::pair<int, std::shared_ptr<ros2_canopen::CanopenDriverInterface>> &driver)
                { return driver.first == data.node_id; });

            if (driver == drivers.end())
            {
                RCLCPP_ERROR(
                    device_container_->get_logger(), "Could not find driver for joint '%s' with node id '%d'",
                    data.joint_name.c_str(), data.node_id);
                continue;
            }

            // Allocate driver to joint.
            if (
                device_container_->get_driver_type(driver->first).compare("ros2_canopen::Cia402Driver") == 0)
            {
                data.driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(driver->second);
            }
        }
        RCLCPP_INFO(device_container_->get_logger(), "Initialisation successful.");
    }

    void LRobotSystemHardware::spin()
    {
        executor_->spin();
        executor_->remove_node(device_container_);
        RCLCPP_INFO(device_container_->get_logger(), "Stopped spinning RobotSystem ROS2 executor");
    }

    void LRobotSystemHardware::clean()
    {
        printf("Cancel exectutor...");
        executor_->cancel();
        printf("Join spin thread...");
        spin_thread_->join();

        printf("Reset variables...");
        device_container_.reset();
        executor_.reset();

        init_thread_->join();
        init_thread_.reset();

        executor_.reset();
        spin_thread_.reset();
        robot_motor_data_.clear();
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    lrobot_control::LRobotSystemHardware, hardware_interface::SystemInterface)