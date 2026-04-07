// SPDX-FileCopyrightText: 2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_controllers/fanuc_force_sensor_broadcaster.hpp"
#include "fanuc_client/fanuc_client.hpp"
#include "fanuc_robot_driver/constants.hpp"

namespace fanuc_controllers
{
constexpr auto kFRForceSensorBroadcaster = "FR_Force_Sensor_Broadcaster";

namespace
{
double ReadInterfaceValue(const hardware_interface::LoanedStateInterface& interface, const rclcpp::Logger& logger,
                          const rclcpp::Clock::SharedPtr& clock)
{
  if (const auto value = interface.get_optional<double>())
  {
    return *value;
  }

  RCLCPP_WARN_THROTTLE(logger, *clock, 5000, "Failed to read interface '%s'", interface.get_name().c_str());
  return 0.0;
}

fanuc_client::FanucClient* getClientInstance()
{
  return fanuc_client::FanucClient::get_instance();
}

void CfgForceSensor(const std::shared_ptr<fanuc_msgs::srv::CfgForceSensor::Request>& request,
                    const std::shared_ptr<fanuc_msgs::srv::CfgForceSensor::Response>& response)
{
  try
  {
    getClientInstance()->configureForceSensor(request->do_reset, request->fs_type);
    response->result = 0;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRForceSensorBroadcaster), "Config force sensor failed: %s", e.what());
    response->result = 1;
  }
}
}  // namespace

controller_interface::CallbackReturn FanucForceSensorBroadcaster::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FanucForceSensorBroadcaster::command_interface_configuration() const
{
  return command_interface_configuration_;
}

controller_interface::InterfaceConfiguration FanucForceSensorBroadcaster::state_interface_configuration() const
{
  return state_interface_configuration_;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FanucForceSensorBroadcaster::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  // Setup command interfaces
  command_interface_configuration_.type = controller_interface::interface_configuration_type::NONE;
  command_interface_configuration_.names.clear();

  // Setup state interfaces
  state_interface_configuration_.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interface_configuration_.names.clear();

  using fanuc_robot_driver::kForceInterfaceName;
  using fanuc_robot_driver::kForceSensorType;
  using fanuc_robot_driver::kForceXType;
  using fanuc_robot_driver::kForceYType;
  using fanuc_robot_driver::kForceZType;
  using fanuc_robot_driver::kMomentXType;
  using fanuc_robot_driver::kMomentYType;
  using fanuc_robot_driver::kMomentZType;

  size_t state_interface_index = 0;
  index_force_sensor_[0] = state_interface_index++;
  state_interface_configuration_.names.push_back(std::string(kForceInterfaceName) + "/" + kForceXType);
  index_force_sensor_[1] = state_interface_index++;
  state_interface_configuration_.names.push_back(std::string(kForceInterfaceName) + "/" + kForceYType);
  index_force_sensor_[2] = state_interface_index++;
  state_interface_configuration_.names.push_back(std::string(kForceInterfaceName) + "/" + kForceZType);
  index_force_sensor_[3] = state_interface_index++;
  state_interface_configuration_.names.push_back(std::string(kForceInterfaceName) + "/" + kMomentXType);
  index_force_sensor_[4] = state_interface_index++;
  state_interface_configuration_.names.push_back(std::string(kForceInterfaceName) + "/" + kMomentYType);
  index_force_sensor_[5] = state_interface_index++;
  state_interface_configuration_.names.push_back(std::string(kForceInterfaceName) + "/" + kMomentZType);
  index_force_sensor_[6] = state_interface_index++;
  state_interface_configuration_.names.push_back(std::string(kForceInterfaceName) + "/" + kForceSensorType);

  force_sensor_publisher_ =
      get_node()->create_publisher<fanuc_msgs::msg::ForceSensor>("~/force_sensor", rclcpp::QoS(1).reliable());

  rt_force_sensor_publisher_ =
      std::make_unique<RealtimePublisher<fanuc_msgs::msg::ForceSensor>>(force_sensor_publisher_);

  cfg_force_sensor_service_ =
      get_node()->create_service<fanuc_msgs::srv::CfgForceSensor>("~/cfg_force_sensor", &CfgForceSensor);

  return ControllerInterface::on_configure(previous_state);
}

controller_interface::CallbackReturn FanucForceSensorBroadcaster::on_activate(const rclcpp_lifecycle::State& state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FanucForceSensorBroadcaster::update(const rclcpp::Time& time,
                                                                      const rclcpp::Duration& period)
{
  auto logger = get_node()->get_logger();
  auto clock = get_node()->get_clock();

  // Publish all state interface data
  if (rt_force_sensor_publisher_->trylock())
  {
    force_sensor_msg_.force_x = ReadInterfaceValue(state_interfaces_[index_force_sensor_[0]], logger, clock);
    force_sensor_msg_.force_y = ReadInterfaceValue(state_interfaces_[index_force_sensor_[1]], logger, clock);
    force_sensor_msg_.force_z = ReadInterfaceValue(state_interfaces_[index_force_sensor_[2]], logger, clock);
    force_sensor_msg_.moment_x = ReadInterfaceValue(state_interfaces_[index_force_sensor_[3]], logger, clock);
    force_sensor_msg_.moment_y = ReadInterfaceValue(state_interfaces_[index_force_sensor_[4]], logger, clock);
    force_sensor_msg_.moment_z = ReadInterfaceValue(state_interfaces_[index_force_sensor_[5]], logger, clock);
    force_sensor_msg_.fs_type = ReadInterfaceValue(state_interfaces_[index_force_sensor_[6]], logger, clock);

    rt_force_sensor_publisher_->msg_ = force_sensor_msg_;
    rt_force_sensor_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FanucForceSensorBroadcaster::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  // Reset all publishers
  rt_force_sensor_publisher_.reset();

  force_sensor_publisher_.reset();

  // Reset all services
  cfg_force_sensor_service_.reset();

  return ControllerInterface::on_deactivate(previous_state);
}
}  // namespace fanuc_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fanuc_controllers::FanucForceSensorBroadcaster, controller_interface::ControllerInterface)
