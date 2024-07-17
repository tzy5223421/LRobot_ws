#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "lrobot_controller.hpp"
#include <cmath>

namespace
{
  constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
  constexpr auto DEFAULT_VEHICLE_TOPIC = "/vehicle_control_status";
  constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
  constexpr auto DEFAULT_ODOMETRY_TOPIC = "/wheel_odom";
  constexpr auto odom_frame_id = "odom";
  constexpr auto base_frame_id = "base_link";
  constexpr auto DEFAULT_MANUAL_SERVICE = "/set_manual_mode";
  constexpr auto DEFAULT_MOTION_SERVICE = "/set_motion_mode";
} // namespace

enum CommandInterface
{
  VELOCITY,
  INIT,
  INIT_FEEDBACK,
  HALT,
  HALT_FEEDBACK,
  RECOVER,
  RECOVER_FEEDBACK,
  OPERATION_MODE,
  OPERATION_MODE_FEEDBACK,
  ADD_OP
};

namespace lrobot_controller
{
  using namespace std::chrono_literals;
  using controller_interface::interface_configuration_type;
  using controller_interface::InterfaceConfiguration;
  using hardware_interface::HW_IF_EFFORT;
  using hardware_interface::HW_IF_POSITION;
  using hardware_interface::HW_IF_VELOCITY;

  using lifecycle_msgs::msg::State;
  using namespace rclcpp_lifecycle;

  LRobotController::LRobotController() : controller_interface::ControllerInterface() {}

  controller_interface::CallbackReturn LRobotController::on_init()
  {
    RCLCPP_INFO(get_node()->get_logger(), "Loading controller...");
    try
    {
      // Create the parameter listener and get the parameters
      param_listener_ = std::make_shared<ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception &e)
    {

      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  InterfaceConfiguration LRobotController::command_interface_configuration() const
  {
    // return command_interfaces_config_;
    std::vector<std::string> conf_names;
    for (const auto &joint_name : params_.joints)
    {
      conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
    }
    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

  InterfaceConfiguration LRobotController::state_interface_configuration() const
  {
    std::vector<std::string> conf_names;
    for (auto joint_name : params_.joints)
    {
      /* code */
      conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
    }

    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

  controller_interface::CallbackReturn LRobotController::on_configure(
      const rclcpp_lifecycle::State &)
  {
    auto logger = get_node()->get_logger();
    RCLCPP_INFO(logger, "Configuring controller...");

    // update parameters if they have changed
    if (param_listener_->is_old(params_))
    {
      params_ = param_listener_->get_params();
      RCLCPP_INFO(logger, "Parameters were updated");
    }

    if (params_.operation_mode == 0)
    {
      RCLCPP_ERROR(
          get_node()->get_logger(), "'operation_mode' parameter was not correctly specified");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (params_.command_poll_freq <= 0)
    {
      RCLCPP_ERROR(
          get_node()->get_logger(), "'command_poll_freq' parameter was not correctly specified");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }
    const Twist empty_twist;
    received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));
    // Fill last two commands with default constructed commands
    previous_commands_.emplace(empty_twist);
    previous_commands_.emplace(empty_twist);
    // initialize command subscriber
    velocity_command_subscriber_ =
        get_node()->create_subscription<geometry_msgs::msg::Twist>(
            DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
            {
              if (!subscriber_is_active_)
              {
                RCLCPP_WARN(
                    get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
                return;
              }
              received_velocity_msg_ptr_.set(std::move(msg));
            });
    // initialize control status publisher and message
    vehicle_control_publisher_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(DEFAULT_VEHICLE_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_control_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticStatus>>(vehicle_control_publisher_);

    robotdriverMode.Manual = true;
    robotdriverMode.Autonomous = false;
    robotmotionMode.CarLink = true;
    robotmotionMode.Omnidirectional = false;

    auto &message = realtime_control_publisher_->msg_;
    auto keyvalue = diagnostic_msgs::msg::KeyValue();
    keyvalue.key = "RobotDriverMode";
    keyvalue.value = "MANUAL";
    message.values.push_back(keyvalue);
    keyvalue.key = "RobotMotionMode";
    keyvalue.value = "CAR";
    message.values.push_back(keyvalue);
    realtime_control_publisher_->unlockAndPublish();

    motion_mode_ = get_node()->create_service<std_srvs::srv::SetBool>(DEFAULT_MOTION_SERVICE, std::bind(&LRobotController::motion_handle_service, this, std::placeholders::_1, std::placeholders::_2));
    manual_mode_ = get_node()->create_service<std_srvs::srv::SetBool>(DEFAULT_MANUAL_SERVICE, std::bind(&LRobotController::manual_handle_service, this, std::placeholders::_1, std::placeholders::_2));
    // initialize odometry publisher and messasge
    odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
        DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_odometry_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
            odometry_publisher_);

    joint_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SystemDefaultsQoS());
    realtime_joint_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(joint_publisher_);

    canopen_front_wheel_publisher_ = get_node()->create_publisher<canopen_interfaces::msg::COData>("/front_wheel_joint/tpdo", rclcpp::SystemDefaultsQoS());
    realtime_canopen_front_wheel_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<canopen_interfaces::msg::COData>>(canopen_front_wheel_publisher_);

    auto &odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.frame_id = odom_frame_id;
    odometry_message.child_frame_id = base_frame_id;
    get_node()->declare_parameter<std::string>("odom_frame_id", "odom");
    // initialize odom values zeros
    odometry_message.twist =
        geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

    constexpr size_t NUM_DIMENSIONS = 6;
    for (size_t index = 0; index < 6; ++index)
    {
      // 0, 7, 14, 21, 28, 35
      const size_t diagonal_index = NUM_DIMENSIONS * index + index;
      odometry_message.pose.covariance[diagonal_index] = 0.0;
      odometry_message.twist.covariance[diagonal_index] = 0.0;
    }

    odometry_.setWheelParams(params_.wheels_separation, params_.wheel_radius);

    // initialize transform publisher and message
    odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
        DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_odometry_transform_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
            odometry_transform_publisher_);

    // keeping track of odom and base_link transforms only
    auto &odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
    odometry_transform_message.transforms.resize(1);
    odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
    odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

    RCLCPP_INFO(logger, "Configure over...");

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn LRobotController::on_activate(
      const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Active lrobot_controller.");
    is_halted = false;
    subscriber_is_active_ = true;
    for (int i = 0; i < command_interfaces_.size(); i++)
    {
      /* code */
      RCLCPP_INFO(get_node()->get_logger(), "Using %s to init", command_interfaces_[i].get_full_name().c_str());
    }
    for (int i = 0; i < state_interfaces_.size(); i++)
    {
      /* code */
      RCLCPP_INFO(get_node()->get_logger(), "Using %s to init", state_interfaces_[i].get_full_name().c_str());
    }

    timer_ = this->get_node()->create_wall_timer(
        std::chrono::milliseconds(params_.command_poll_freq),
        std::bind(&LRobotController::activate, this));
    RCLCPP_INFO(get_node()->get_logger(), "Subscriber and publisher are now active.");

    return controller_interface::CallbackReturn::SUCCESS;
  }

  void LRobotController::activate()
  {
    for (auto joint_name : params_.joints)
    {
      RCLCPP_INFO(get_node()->get_logger(), "Initialise '%s'", joint_name.c_str());
      // Initialise joint
      ////////////////////////////////////////////////////////////////////////////////////////
      RCLCPP_INFO(get_node()->get_logger(), "Using %s to init", joint_name.c_str());
      init_client_ =
          get_node()->create_client<std_srvs::srv::Trigger>("/" + joint_name + "/position_mode");
      while (!init_client_->wait_for_service(std::chrono::seconds(1)))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(get_node()->get_logger(), "service not available, waiting again...");
        auto trigger_req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = init_client_->async_send_request(trigger_req);
        if (rclcpp::spin_until_future_complete(get_node(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(get_node()->get_logger(), "Init service called successfully");
        }
        else
        {
          RCLCPP_ERROR(get_node()->get_logger(), "Failed to call init service");
        }
      }
    }
    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    timer_->cancel();
  }

  bool LRobotController::motion_handle_service(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (request.get()->data)
    {
      robotmotionMode.Omnidirectional = true;
      robotmotionMode.CarLink = false;
      RCLCPP_INFO(get_node()->get_logger(), "RobotMotionMode is OMNI ");
    }
    else
    {
      robotmotionMode.Omnidirectional = false;
      robotmotionMode.CarLink = true;
      RCLCPP_INFO(get_node()->get_logger(), "RobotMotionMode is CAR ");
    }
    auto &message = realtime_control_publisher_->msg_;
    message.values.clear();
    auto keyvalue = diagnostic_msgs::msg::KeyValue();
    keyvalue.key = "RobotMotionMode";
    if (robotmotionMode.Omnidirectional)
    {
      keyvalue.value = "OMNI";
    }
    else
    {
      keyvalue.value = "CAR";
    }
    message.values.push_back(keyvalue);
    keyvalue.key = "RobotDriveMode";
    if (robotdriverMode.Manual)
    {
      /* code */
      keyvalue.value = "MANUAL";
    }
    else
    {
      keyvalue.value = "AUTONOMOUS";
    }
    message.values.push_back(keyvalue);
    message.name = {"vehicle_control_status"};
    message.message = {"Robot Control Mode"};
    realtime_control_publisher_->unlockAndPublish();
    response->success = true;
    return response->success;
  }

  bool LRobotController::manual_handle_service(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (request.get()->data)
    {
      robotdriverMode.Manual = true;
      robotdriverMode.Autonomous = false;
      RCLCPP_INFO(get_node()->get_logger(), "RobotDriveMode is MANUAL ");
    }
    else
    {
      robotdriverMode.Manual = false;
      robotdriverMode.Autonomous = true;
      RCLCPP_INFO(get_node()->get_logger(), "RobotDriveMode is AUTONOMOUS ");
    }
    auto &message = realtime_control_publisher_->msg_;
    message.values.clear();
    auto keyvalue = diagnostic_msgs::msg::KeyValue();
    keyvalue.key = "RobotMotionMode";
    if (robotmotionMode.Omnidirectional)
    {
      keyvalue.value = "OMNI";
    }
    else
    {
      keyvalue.value = "CAR";
    }
    message.values.push_back(keyvalue);
    keyvalue.key = "RobotDriveMode";
    if (robotdriverMode.Manual)
    {
      keyvalue.value = "MANUAL";
    }
    else
    {
      keyvalue.value = "AUTONOMOUS";
    }
    message.values.push_back(keyvalue);
    message.name = {"vehicle_control_status"};
    message.message = {"Robot Control Mode"};
    realtime_control_publisher_->unlockAndPublish();
    response->success = true;
    return response->success;
  }

  controller_interface::CallbackReturn LRobotController::on_cleanup(
      const rclcpp_lifecycle::State &)
  {
    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }

    received_velocity_msg_ptr_.set(std::make_shared<Twist>());

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn LRobotController::on_error(
      const rclcpp_lifecycle::State &)
  {
    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn LRobotController::on_deactivate(
      const rclcpp_lifecycle::State &)
  {

    subscriber_is_active_ = false;
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    int jn = 0;
    RCLCPP_INFO(get_node()->get_logger(), "deactivate successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn LRobotController::on_shutdown(
      const rclcpp_lifecycle::State &)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type LRobotController::updateCommand(const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    // update cmd
    std::shared_ptr<Twist> last_command_msg;
    received_velocity_msg_ptr_.get(last_command_msg);
    auto logger = get_node()->get_logger();
    if (last_command_msg == nullptr)
    {
      RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
      return controller_interface::return_type::ERROR;
    }

    double front_wheel_vel = 0, back_wheel_vel = 0;
    double front_motor_vel = 0, back_motor_vel = 0;
    double cmd_vel_timeout_ = 0.5;
    const double cmd_dt = period.seconds();
    if (robotdriverMode.Manual)
    {
      /// Omnidirectional inverseKinematics
      if (robotmotionMode.Omnidirectional)
      {
        if ((fabs(last_command_msg.get()->angular.z) * 8) <= 3 && fabs(last_command_msg.get()->linear.y) == 0 && fabs(last_command_msg.get()->linear.x) > 0.0001)
        {
          for (size_t index = 0; index < static_cast<size_t>(params_.wheels_per_side); ++index)
          {
            RCLCPP_INFO(logger, "last_command_msg.get()->angular.z %f", last_command_msg.get()->angular.z);
            RCLCPP_INFO(logger, "last_command_msg.get()->linear.y: %f", last_command_msg.get()->linear.y);

            // auto &joint_message = realtime_joint_publisher_->msg_;
            // joint_message.header.stamp = time;
            // joint_message.name = {"front_motor_joint", "front_wheel_joint", "back_motor_joint", "back_wheel_joint"};
            // joint_message.position = {0, 0, 1.57, 1.57};
            // joint_message.velocity = {0, 0, 1.0, 1.0};
            // realtime_joint_publisher_->unlockAndPublish();

            auto &canopenmsg = realtime_canopen_front_wheel_publisher_->msg_;
            canopenmsg.index = 0x6064;
            canopenmsg.subindex = 0x1;
            canopenmsg.data = 500;
            realtime_canopen_front_wheel_publisher_->unlockAndPublish();

            // RCLCPP_INFO(logger, "front_feedback message: %s", registered_back_motor_handles_[index].velocity.get().get_full_name().c_str());
          }
        }
        /// On-spot rotation
        else if ((fabs(last_command_msg.get()->linear.x) < 0.0001) && (fabs(last_command_msg.get()->linear.y) < 0.0001) && (fabs(last_command_msg.get()->angular.z) != 0))
        {
          /// set motor position,front_motor=-back_motor读取舵机当前角度，设置舵机角度（），等待舵机角度到位后进行平移运动

          // auto &joint_message = realtime_joint_publisher_->msg_;
          // joint_message.header.stamp = time;
          // joint_message.name = {"front_motor_joint", "front_wheel_joint", "back_motor_joint", "back_wheel_joint"};
          // joint_message.position = {0, 0, 1.57, 1.57};
          // joint_message.velocity = {0, 0, 1.0, 1.0};
          // realtime_joint_publisher_->unlockAndPublish();
          // RCLCPP_INFO(logger, "front_feedback message: %s", registered_back_motor_handles_[index].velocity.get().get_full_name().c_str());
          /// caculate vel;
          front_wheel_vel = fabs(last_command_msg.get()->angular.z) / (params_.wheels_separation / 2.0);
          back_wheel_vel = -front_wheel_vel; // 反向运动
          /// config driver
        }
        /// Translational movement(sideway)
        else if ((fabs(last_command_msg.get()->linear.x) < 0.0001) && (fabs(last_command_msg.get()->angular.z) < 0.0001) && (fabs(last_command_msg.get()->linear.y) > 0.0001))
        {
          //
        }
        /// Translational movement(diagnoal)
        else if ((fabs(last_command_msg.get()->linear.x) != 0) && (fabs(last_command_msg.get()->angular.z) < 0.0001) && (fabs(last_command_msg.get()->linear.y) < 0.0001))
        {
          // angular.z为角度
        }
        /*stop car*/
        else if (last_command_msg.get()->linear.x == 0 && last_command_msg.get()->linear.y == 0 && last_command_msg.get()->angular.z == 0)
        {
          front_motor_vel = 0;
          back_motor_vel = 0;
          front_wheel_vel = 0;
          back_wheel_vel = 0;
        }
      }
      //
      /// CarLike inverseKinematics
      if (robotmotionMode.CarLink)
      {
        /* code */
        if ((fabs(last_command_msg.get()->angular.z) * 8) <= 3 && fabs(last_command_msg.get()->linear.y) == 0 && fabs(last_command_msg.get()->linear.x) > 0.0001)
        {
        }
        else if ((fabs(last_command_msg.get()->angular.z)))
        {
          /// angular.z为角度，x,y为线速度分量
        }
      }
    }
    if (robotdriverMode.Autonomous)
    {
      /// Omnidirectional inverseKinematics
      if (robotmotionMode.Omnidirectional)
      {
        /* code */
        if ((fabs(last_command_msg.get()->angular.z) * 8) <= 3 && fabs(last_command_msg.get()->linear.y) == 0 && fabs(last_command_msg.get()->linear.x) > 0.0001)
        {
          /* rviz2 sim*/
          for (size_t index = 0; index < static_cast<size_t>(params_.wheels_per_side); ++index)
          {
            RCLCPP_INFO(logger, "last_command_msg.get()->angular.z %f", last_command_msg.get()->angular.z);
            RCLCPP_INFO(logger, "last_command_msg.get()->linear.y: %f", last_command_msg.get()->linear.y);

            auto &joint_message = realtime_joint_publisher_->msg_;
            joint_message.header.stamp = time;
            joint_message.name = {"front_motor_joint", "front_wheel_joint", "back_motor_joint", "back_wheel_joint"};
            joint_message.position = {0, 0, 1.57, 1.57};
            joint_message.velocity = {0, 0, 1.0, 1.0};
            realtime_joint_publisher_->unlockAndPublish();
            // RCLCPP_INFO(logger, "front_feedback message: %s", registered_back_motor_handles_[index].velocity.get().get_full_name().c_str());
            //  RCLCPP_INFO(logger, "front_feedback message: %f", front_feedback);
            //  RCLCPP_INFO(logger, "back_feedback message: %f", back_feedback);
          }
        }
        /// On-spot rotation
        else if ((fabs(last_command_msg.get()->linear.x) < 0.0001) && (fabs(last_command_msg.get()->linear.y) < 0.0001) && (fabs(last_command_msg.get()->angular.z) != 0))
        {
          /// set motor position,front_motor=-back_motor读取舵机当前角度，设置舵机角度（），等待舵机角度到位后进行平移运动

          // auto &joint_message = realtime_joint_publisher_->msg_;
          // joint_message.header.stamp = time;
          // joint_message.name = {"front_motor_joint", "front_wheel_joint", "back_motor_joint", "back_wheel_joint"};
          // joint_message.position = {0, 0, 1.57, 1.57};
          // joint_message.velocity = {0, 0, 1.0, 1.0};
          // realtime_joint_publisher_->unlockAndPublish();
          // RCLCPP_INFO(logger, "front_feedback message: %s", registered_back_motor_handles_[index].velocity.get().get_full_name().c_str());
          /// caculate vel;
          front_wheel_vel = fabs(last_command_msg.get()->angular.z) / (params_.wheels_separation / 2.0);
          back_wheel_vel = -front_wheel_vel; // 反向运动
          /// config driver
        }
        /// Translational movement(sideway)
        else if ((fabs(last_command_msg.get()->linear.x) < 0.0001) && (fabs(last_command_msg.get()->angular.z) < 0.0001) && (fabs(last_command_msg.get()->linear.y) > 0.0001))
        {
          //
        }
        /// Translational movement(diagnoal)
        else if ((fabs(last_command_msg.get()->linear.x) != 0) && (fabs(last_command_msg.get()->angular.z) < 0.0001) && (fabs(last_command_msg.get()->linear.y) < 0.0001))
        {
          // angular.z为角速度,linear.x，linear.y为车体xy轴速度分量
        }
        /*stop car*/
        else if (last_command_msg.get()->linear.x == 0 && last_command_msg.get()->linear.y == 0 && last_command_msg.get()->angular.z == 0)
        {
          front_motor_vel = 0;
          back_motor_vel = 0;
          front_wheel_vel = 0;
          back_wheel_vel = 0;
        }
      }
      /// CarLike inverseKinematics
      if (robotmotionMode.CarLink)
      {
        /* code */
        if ((fabs(last_command_msg.get()->angular.z) * 8) <= 3 && fabs(last_command_msg.get()->linear.y) == 0 && fabs(last_command_msg.get()->linear.x) > 0.0001)
        {
        }
        else if ((fabs(last_command_msg.get()->angular.z)))
        {
          /// angular.z为角速度，x,y为线速度分量
          double car_angular = std::atan(last_command_msg.get()->linear.x / last_command_msg.get()->linear.y);
          front_motor_vel = car_angular;
          back_motor_vel = car_angular;
          /// 计算车子速度
          double car_linear_vel = std::hypot(last_command_msg.get()->linear.x, last_command_msg.get()->linear.y);
          if (last_command_msg.get()->linear.y < 0)
          {
            /// y负方向平移
            /* code */
            front_motor_vel = 1.57;
            back_motor_vel = 1.57;
            front_wheel_vel = last_command_msg.get()->linear.y;
            back_wheel_vel = last_command_msg.get()->linear.y;
          }
          else
          {
            /// y正方向平移
            front_motor_vel = -1.57;
            back_motor_vel = -1.57;
            front_wheel_vel = last_command_msg.get()->linear.y;
            back_wheel_vel = last_command_msg.get()->linear.y;
          }
        }
      }
    }
    return controller_interface::return_type::OK;
  }

  // controller_interface::return_type LRobotController::updateOdometry(const rclcpp::Time &time, const rclcpp::Duration &period){
  //   return controller_interface::return_type::OK;
  // }

  controller_interface::return_type LRobotController::update(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    return updateCommand(time, period);
  }

  controller_interface::CallbackReturn LRobotController::configure_side(
      const std::string &wheel_kind,
      const std::vector<std::string> &wheel_names,
      std::vector<WheelHandle> &registered_handles)
  {
    auto logger = get_node()->get_logger();
    for (const auto &wheel_name : wheel_names)
    {
      RCLCPP_INFO(logger, "wheel_name '%s' ", wheel_name.c_str());
    }
    if (wheel_names.empty())
    {
      RCLCPP_ERROR(logger, "No '%s' motor names specified", wheel_kind.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    // register handles
    registered_handles.reserve(wheel_names.size());
    for (const auto &wheel_name : wheel_names)
    {
      const auto state_handle = std::find_if(
          state_interfaces_.cbegin(), state_interfaces_.cend(),
          [&wheel_name](const auto &interface)
          {
            return interface.get_prefix_name() == wheel_name &&
                   interface.get_interface_name() == HW_IF_VELOCITY;
          });

      if (state_handle == state_interfaces_.cend())
      {
        RCLCPP_ERROR(logger, "Unable to obtain motor state handle for %s", wheel_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      const auto command_handle = std::find_if(
          command_interfaces_.begin(), command_interfaces_.end(),
          [&wheel_name](const auto &interface)
          {
            return interface.get_prefix_name() == wheel_name &&
                   interface.get_interface_name() == HW_IF_VELOCITY;
          });

      if (command_handle == command_interfaces_.end())
      {
        RCLCPP_ERROR(logger, "Unable to obtain motor command handle for %s", wheel_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      registered_handles.emplace_back(
          WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  bool LRobotController::reset()
  {
    // release the old queue
    std::queue<Twist> empty;
    std::swap(previous_commands_, empty);

    subscriber_is_active_ = false;
    velocity_command_subscriber_.reset();
    received_velocity_msg_ptr_.set(nullptr);

    is_halted = false;
    return true;
  }

  void LRobotController::halt()
  {
    // make wheels stop
    const auto halt_wheels = [](auto &wheel_handles)
    {
      for (const auto &wheel_handle : wheel_handles)
      {
        wheel_handle.velocity.get().set_value(0.0);
      }
    };
  }

} // namespace diff_test_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    lrobot_controller::LRobotController, controller_interface::ControllerInterface)
