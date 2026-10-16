// Copyright 2026 Branimir Caran
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "swerve_drive_controller/swerve_drive_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
constexpr auto DEFAULT_RESET_ODOMETRY_SERVICE = "~/reset_odometry";

// Below this wheel speed [m/s] the heading of the wheel is undefined, so the
// previous steering reference is held instead of snapping to atan2(0, 0) = 0.
constexpr double MIN_WHEEL_SPEED = 1e-4;

constexpr int LOG_THROTTLE_MS = 1000;
}  // namespace

namespace swerve_drive_controller
{
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

SwerveDriveController::SwerveDriveController() : controller_interface::ControllerInterface() {}

std::array<std::string, NUM_WHEELS> SwerveDriveController::drive_joint_names() const
{
  return {
    params_.drive_joint_fl, params_.drive_joint_bl, params_.drive_joint_br,
    params_.drive_joint_fr};
}

std::array<std::string, NUM_WHEELS> SwerveDriveController::steering_joint_names() const
{
  return {
    params_.steering_joint_fl, params_.steering_joint_bl, params_.steering_joint_br,
    params_.steering_joint_fr};
}

const char * SwerveDriveController::drive_feedback_type() const
{
  return params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

const char * SwerveDriveController::steering_command_type() const
{
  return params_.steering_command_interface == "velocity" ? HW_IF_VELOCITY : HW_IF_POSITION;
}

controller_interface::CallbackReturn SwerveDriveController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration SwerveDriveController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : drive_joint_names()) {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : steering_joint_names()) {
    conf_names.push_back(joint_name + "/" + steering_command_type());
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration SwerveDriveController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : drive_joint_names()) {
    conf_names.push_back(joint_name + "/" + drive_feedback_type());
  }
  for (const auto & joint_name : steering_joint_names()) {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type SwerveDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted_) {
      halt();
      is_halted_ = true;
    }
    return controller_interface::return_type::OK;
  }

  // Pick up parameters changed with `ros2 param set`. Non-blocking.
  if (param_listener_->try_update_params(params_)) {
    apply_runtime_parameters();
  }

  const double dt = period.seconds();

  // ---------------------------------------------------------------- command
  BodyTwist command;
  const std::shared_ptr<Twist> last_command_msg = *received_velocity_msg_ptr_.readFromRT();
  if (last_command_msg != nullptr) {
    command.vx = last_command_msg->twist.linear.x;
    command.vy = last_command_msg->twist.linear.y;
    command.wz = last_command_msg->twist.angular.z;

    // Brake if cmd_vel has timed out. A cmd_vel_timeout of 0.0 disables it.
    const double age_of_last_command =
      time.seconds() - rclcpp::Time(last_command_msg->header.stamp).seconds();
    if (params_.cmd_vel_timeout > 0.0 && age_of_last_command > params_.cmd_vel_timeout) {
      command = BodyTwist{};
    }
  }

  // --------------------------------------------------------------- feedback
  std::array<double, NUM_WHEELS> drive_feedback{};
  std::array<double, NUM_WHEELS> steering_position{};
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    drive_feedback[i] = state_interfaces_[drive_state_index_[i]].get_value();
    steering_position[i] = state_interfaces_[steering_state_index_[i]].get_value();
    if (std::isnan(drive_feedback[i]) || std::isnan(steering_position[i])) {
      RCLCPP_ERROR_THROTTLE(
        logger, *get_node()->get_clock(), LOG_THROTTLE_MS,
        "Invalid feedback for wheel %zu (drive %s or steering position is NaN)", i,
        drive_feedback_type());
      return controller_interface::return_type::ERROR;
    }
  }

  // --------------------------------------------------------------- odometry
  if (reset_odometry_requested_.exchange(false)) {
    odometry_.reset_odometry();
  }
  if (!odometry_initialized_) {
    odometry_.init(time);
    if (!params_.open_loop && params_.position_feedback) {
      // Stores the initial wheel positions.
      odometry_.update_from_position(drive_feedback, steering_position, time);
    }
    odometry_initialized_ = true;
  } else if (!params_.open_loop) {
    if (params_.position_feedback) {
      odometry_.update_from_position(drive_feedback, steering_position, time);
    } else {
      odometry_.update_from_velocity(drive_feedback, steering_position, time);
    }
  }

  // ----------------------------------------------------------- body limits
  limiter_linear_x_.limit(command.vx, last_command_.vx, second_to_last_command_.vx, dt);
  limiter_linear_y_.limit(command.vy, last_command_.vy, second_to_last_command_.vy, dt);
  limiter_angular_z_.limit(command.wz, last_command_.wz, second_to_last_command_.wz, dt);
  second_to_last_command_ = last_command_;
  last_command_ = command;

  if (realtime_limited_velocity_publisher_ && realtime_limited_velocity_publisher_->trylock()) {
    auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = time;
    limited_velocity_command.twist.linear.x = command.vx;
    limited_velocity_command.twist.linear.y = command.vy;
    limited_velocity_command.twist.angular.z = command.wz;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }

  // ---------------------------------------------------- inverse kinematics
  std::array<double, NUM_WHEELS> wheel_velocity{};
  std::array<double, NUM_WHEELS> steering_reference{};
  double max_ratio = 0.0;
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    WheelState target = wheel_inverse_kinematics(geometry_[i], command);
    if (target.speed < MIN_WHEEL_SPEED) {
      target = WheelState{previous_steering_reference_[i], 0.0};
    }
    WheelState wheel = unwrap_steering(target, previous_steering_reference_[i]);
    if (has_steering_limits_) {
      wheel = flip_into_limits(wheel, min_steering_angle_, max_steering_angle_);
    }

    steering_reference[i] = wheel.angle;
    wheel_velocity[i] = wheel.speed / geometry_[i].radius;
    if (params_.max_wheel_angular_velocity > 0.0) {
      max_ratio =
        std::max(max_ratio, std::abs(wheel_velocity[i]) / params_.max_wheel_angular_velocity);
    }
  }

  // Scale all wheels by the same factor so the direction of motion is kept.
  if (max_ratio > 1.0) {
    for (auto & velocity : wheel_velocity) {
      velocity /= max_ratio;
    }
    command.vx /= max_ratio;
    command.vy /= max_ratio;
    command.wz /= max_ratio;
  }

  // ------------------------------------------------------------- commands
  const double max_steering_velocity = params_.max_steering_velocity;
  const auto saturate_steering = [max_steering_velocity](double value) {
      return max_steering_velocity > 0.0 ?
             std::clamp(value, -max_steering_velocity, max_steering_velocity) :
             value;
    };
  const double alpha =
    std::exp(-2.0 * M_PI * params_.steering_feedforward_cutoff_frequency * std::max(dt, 0.0));

  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    // Without limits the steering is continuous and the hardware may report
    // the angle wrapped, so only the shortest difference is meaningful.
    const double steering_error = has_steering_limits_ ?
      steering_reference[i] - steering_position[i] :
      std::remainder(steering_reference[i] - steering_position[i], 2.0 * M_PI);

    if (steering_mode_ == SteeringMode::POSITION) {
      command_interfaces_[steering_command_index_[i]].set_value(steering_reference[i]);
    } else {
      double feedforward = 0.0;
      if (params_.steering_velocity_feedforward) {
        // A wheel flip moves the reference by ~pi in one step. That is a jump
        // the P loop has to handle, not a velocity to feed forward.
        const double step = steering_reference[i] - previous_steering_reference_[i];
        const double raw =
          (dt > 0.0 && std::abs(step) < M_PI_2) ? saturate_steering(step / dt) : 0.0;
        steering_feedforward_[i] = alpha * steering_feedforward_[i] + (1.0 - alpha) * raw;
        feedforward = steering_feedforward_[i];
      }
      const double steering_velocity =
        saturate_steering(feedforward + params_.k_delta * steering_error);
      command_interfaces_[steering_command_index_[i]].set_value(steering_velocity);
    }

    if (params_.scale_drive_by_steering_error) {
      wheel_velocity[i] *= std::max(0.0, std::cos(steering_error));
    }
    command_interfaces_[drive_command_index_[i]].set_value(wheel_velocity[i]);

    previous_steering_reference_[i] = steering_reference[i];
  }

  if (params_.open_loop) {
    odometry_.update_open_loop(command, time);
  }

  // -------------------------------------------------------------- publish
  bool should_publish = false;
  try {
    if (previous_publish_timestamp_ + publish_period_ < time) {
      previous_publish_timestamp_ += publish_period_;
      should_publish = true;
    }
  } catch (const std::runtime_error &) {
    // Handle exceptions when the time source changes and initialize publish timestamp
    previous_publish_timestamp_ = time;
    should_publish = true;
  }

  if (should_publish) {
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, odometry_.get_heading());

    if (realtime_odometry_publisher_->trylock()) {
      auto & odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = time;
      odometry_message.pose.pose.position.x = odometry_.get_x();
      odometry_message.pose.pose.position.y = odometry_.get_y();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
      odometry_message.twist.twist.linear.x = odometry_.get_linear_x();
      odometry_message.twist.twist.linear.y = odometry_.get_linear_y();
      odometry_message.twist.twist.angular.z = odometry_.get_angular();
      realtime_odometry_publisher_->unlockAndPublish();
    }

    if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock()) {
      auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = time;
      transform.transform.translation.x = odometry_.get_x();
      transform.transform.translation.y = odometry_.get_y();
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  return controller_interface::return_type::OK;
}

bool SwerveDriveController::apply_runtime_parameters()
{
  auto logger = get_node()->get_logger();

  const Geometry geometry = {{
    {params_.x_w_fl, params_.y_w_fl, params_.r_w_fl},
    {params_.x_w_bl, params_.y_w_bl, params_.r_w_bl},
    {params_.x_w_br, params_.y_w_br, params_.r_w_br},
    {params_.x_w_fr, params_.y_w_fr, params_.r_w_fr},
  }};

  // Same convention as the speed limiters: max is required, min defaults to -max.
  double min_steering_angle = params_.min_steering_angle;
  double max_steering_angle = params_.max_steering_angle;
  if (params_.has_steering_limits) {
    if (std::isnan(max_steering_angle)) {
      RCLCPP_ERROR(logger, "max_steering_angle must be set when has_steering_limits is true");
      return false;
    }
    if (std::isnan(min_steering_angle)) {
      min_steering_angle = -max_steering_angle;
    }
    if (max_steering_angle <= min_steering_angle) {
      RCLCPP_ERROR(
        logger, "max_steering_angle (%f) must be greater than min_steering_angle (%f)",
        max_steering_angle, min_steering_angle);
      return false;
    }
  }

  try {
    const SpeedLimiter limiter_linear_x(
      params_.linear.x.has_velocity_limits, params_.linear.x.has_acceleration_limits,
      params_.linear.x.has_jerk_limits, params_.linear.x.min_velocity,
      params_.linear.x.max_velocity, params_.linear.x.min_acceleration,
      params_.linear.x.max_acceleration, params_.linear.x.min_jerk, params_.linear.x.max_jerk);
    const SpeedLimiter limiter_linear_y(
      params_.linear.y.has_velocity_limits, params_.linear.y.has_acceleration_limits,
      params_.linear.y.has_jerk_limits, params_.linear.y.min_velocity,
      params_.linear.y.max_velocity, params_.linear.y.min_acceleration,
      params_.linear.y.max_acceleration, params_.linear.y.min_jerk, params_.linear.y.max_jerk);
    const SpeedLimiter limiter_angular_z(
      params_.angular.z.has_velocity_limits, params_.angular.z.has_acceleration_limits,
      params_.angular.z.has_jerk_limits, params_.angular.z.min_velocity,
      params_.angular.z.max_velocity, params_.angular.z.min_acceleration,
      params_.angular.z.max_acceleration, params_.angular.z.min_jerk,
      params_.angular.z.max_jerk);

    if (!odometry_.set_wheel_params(geometry)) {
      RCLCPP_ERROR(logger, "Wheel positions are degenerate, odometry cannot be computed");
      return false;
    }

    limiter_linear_x_ = limiter_linear_x;
    limiter_linear_y_ = limiter_linear_y;
    limiter_angular_z_ = limiter_angular_z;
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(logger, "Invalid speed limits: %s", e.what());
    return false;
  }

  geometry_ = geometry;
  has_steering_limits_ = params_.has_steering_limits;
  min_steering_angle_ = min_steering_angle;
  max_steering_angle_ = max_steering_angle;
  return true;
}

controller_interface::CallbackReturn SwerveDriveController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  steering_mode_ = params_.steering_command_interface == "velocity" ? SteeringMode::VELOCITY :
    SteeringMode::POSITION;

  if (!apply_runtime_parameters()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (has_steering_limits_ && max_steering_angle_ - min_steering_angle_ < M_PI) {
    RCLCPP_WARN(
      logger,
      "Steering range [%f, %f] is shorter than pi: some wheel headings are not reachable "
      "and will be clamped.",
      min_steering_angle_, max_steering_angle_);
  }

  odometry_.set_velocity_rolling_window_size(
    static_cast<std::size_t>(params_.velocity_rolling_window_size));

  if (!reset()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.publish_limited_velocity) {
    limited_velocity_publisher_ =
      get_node()->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  // initialize command subscriber
  if (params_.use_stamped_vel) {
    velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Twist> msg) -> void
      {
        if (!subscriber_is_active_) {
          RCLCPP_WARN(
            get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
          RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        received_velocity_msg_ptr_.writeFromNonRT(msg);
      });
  } else {
    velocity_command_unstamped_subscriber_ =
      get_node()->create_subscription<geometry_msgs::msg::Twist>(
      DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
      {
        if (!subscriber_is_active_) {
          RCLCPP_WARN(
            get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        auto twist_stamped = std::make_shared<Twist>();
        twist_stamped->twist = *msg;
        twist_stamped->header.stamp = get_node()->get_clock()->now();
        received_velocity_msg_ptr_.writeFromNonRT(twist_stamped);
      });
  }

  reset_odometry_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    DEFAULT_RESET_ODOMETRY_SERVICE,
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      reset_odometry_requested_ = true;
      response->success = true;
      response->message = "Odometry reset";
      RCLCPP_INFO(get_node()->get_logger(), "Odometry reset");
    });

  // initialize odometry publisher and message
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
    odometry_publisher_);

  // Append the tf prefix if there is one
  std::string tf_prefix = "";
  if (params_.tf_frame_prefix_enable) {
    if (params_.tf_frame_prefix != "") {
      tf_prefix = params_.tf_frame_prefix;
    } else {
      tf_prefix = std::string(get_node()->get_namespace());
    }

    // Make sure prefix does not start with '/' and always ends with '/'
    if (tf_prefix.back() != '/') {
      tf_prefix = tf_prefix + "/";
    }
    if (tf_prefix.front() == '/') {
      tf_prefix.erase(0, 1);
    }
  }

  const auto odom_frame_id = tf_prefix + params_.odom_frame_id;
  const auto base_frame_id = tf_prefix + params_.base_frame_id;

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_frame_id;
  odometry_message.child_frame_id = base_frame_id;

  // limit the publication on the topics /odom and /tf
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / params_.publish_rate);

  // initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr std::size_t NUM_DIMENSIONS = 6;
  for (std::size_t index = 0; index < NUM_DIMENSIONS; ++index) {
    // 0, 7, 14, 21, 28, 35
    const std::size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
    odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  const auto find_command = [this](const std::string & joint, const std::string & type) -> long {
      for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
        if (
          command_interfaces_[i].get_prefix_name() == joint &&
          command_interfaces_[i].get_interface_name() == type)
        {
          return static_cast<long>(i);
        }
      }
      return -1;
    };
  const auto find_state = [this](const std::string & joint, const std::string & type) -> long {
      for (std::size_t i = 0; i < state_interfaces_.size(); ++i) {
        if (
          state_interfaces_[i].get_prefix_name() == joint &&
          state_interfaces_[i].get_interface_name() == type)
        {
          return static_cast<long>(i);
        }
      }
      return -1;
    };

  const auto drive_joints = drive_joint_names();
  const auto steering_joints = steering_joint_names();
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    const long drive_command = find_command(drive_joints[i], HW_IF_VELOCITY);
    const long drive_state = find_state(drive_joints[i], drive_feedback_type());
    const long steering_command = find_command(steering_joints[i], steering_command_type());
    const long steering_state = find_state(steering_joints[i], HW_IF_POSITION);
    if (drive_command < 0 || drive_state < 0 || steering_command < 0 || steering_state < 0) {
      RCLCPP_ERROR(
        logger, "Unable to obtain interfaces for drive joint '%s' / steering joint '%s'",
        drive_joints[i].c_str(), steering_joints[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    drive_command_index_[i] = static_cast<std::size_t>(drive_command);
    drive_state_index_[i] = static_cast<std::size_t>(drive_state);
    steering_command_index_[i] = static_cast<std::size_t>(steering_command);
    steering_state_index_[i] = static_cast<std::size_t>(steering_state);
  }

  // Start from the current steering angles so the wheels do not snap to zero.
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    const double steering_position = state_interfaces_[steering_state_index_[i]].get_value();
    if (std::isnan(steering_position)) {
      previous_steering_reference_[i] = 0.0;
    } else if (has_steering_limits_) {
      previous_steering_reference_[i] =
        std::clamp(steering_position, min_steering_angle_, max_steering_angle_);
    } else {
      previous_steering_reference_[i] = steering_position;
    }
    steering_feedforward_[i] = 0.0;
  }
  last_command_ = BodyTwist{};
  second_to_last_command_ = BodyTwist{};
  odometry_initialized_ = false;

  is_halted_ = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(logger, "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  if (!is_halted_) {
    halt();
    is_halted_ = true;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (!reset()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_error(
  const rclcpp_lifecycle::State &)
{
  if (!reset()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool SwerveDriveController::reset()
{
  odometry_.reset_odometry();
  odometry_initialized_ = false;
  reset_odometry_requested_ = false;

  last_command_ = BodyTwist{};
  second_to_last_command_ = BodyTwist{};
  previous_steering_reference_.fill(0.0);
  steering_feedforward_.fill(0.0);

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();
  reset_odometry_service_.reset();

  received_velocity_msg_ptr_.writeFromNonRT(nullptr);
  is_halted_ = false;
  return true;
}

void SwerveDriveController::halt()
{
  // Interfaces are only resolved once the controller has been activated.
  if (command_interfaces_.size() < 2 * NUM_WHEELS) {
    return;
  }
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    command_interfaces_[drive_command_index_[i]].set_value(0.0);
    // In position mode the steering keeps its last reference.
    if (steering_mode_ == SteeringMode::VELOCITY) {
      command_interfaces_[steering_command_index_[i]].set_value(0.0);
    }
  }
}

}  // namespace swerve_drive_controller

PLUGINLIB_EXPORT_CLASS(
  swerve_drive_controller::SwerveDriveController, controller_interface::ControllerInterface)
