// Implementation of PublishSignalColorAction

#include "manymove_color_signal/bt_nodes.hpp"

#include <utility>

namespace manymove_color_signal
{

PublishSignalColorAction::PublishSignalColorAction(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  // Retrieve ROS node from blackboard
  if (!config.blackboard) {
    throw BT::RuntimeError("PublishSignalColorAction: missing blackboard");
  }
  if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("PublishSignalColorAction: 'node' not found in blackboard");
  }

  // Read (or default) the topic name from ports
  if (!getInput<std::string>("topic", topic_)) {
    topic_ = "/signal_column";
  }

  // Attempt to read key names once; will re-read in tick if ports change dynamically
  (void)getInput<std::string>("green_key", green_key_);
  (void)getInput<std::string>("yellow_key", yellow_key_);
  (void)getInput<std::string>("red_key", red_key_);

  // Create publisher once
  pub_ = node_->create_publisher<signal_column_msgs::msg::SignalColor>(topic_, rclcpp::QoS(10));
}

BT::NodeStatus PublishSignalColorAction::tick()
{
  // (Re)read key names from ports in case they were changed via remapping/params
  std::string gk, yk, rk;
  if (getInput<std::string>("green_key", gk) && !gk.empty()) {
    green_key_ = gk;
  }
  if (getInput<std::string>("yellow_key", yk) && !yk.empty()) {
    yellow_key_ = yk;
  }
  if (getInput<std::string>("red_key", rk) && !rk.empty()) {
    red_key_ = rk;
  }

  bool green = false;
  bool yellow = false;
  bool red = false;

  // Read bools from blackboard by key names; missing keys default to false
  if (!green_key_.empty()) {
    (void)config().blackboard->get(green_key_, green);
  }
  if (!yellow_key_.empty()) {
    (void)config().blackboard->get(yellow_key_, yellow);
  }
  if (!red_key_.empty()) {
    (void)config().blackboard->get(red_key_, red);
  }

  signal_column_msgs::msg::SignalColor msg;
  msg.green_color = green;
  msg.yellow_color = yellow;
  msg.red_color = red;

  pub_->publish(msg);

  RCLCPP_DEBUG(
    node_->get_logger(),
    "PublishSignalColorAction: topic=%s gk=%s yk=%s rk=%s -> g=%d y=%d r=%d",
    topic_.c_str(), green_key_.c_str(), yellow_key_.c_str(), red_key_.c_str(),
    static_cast<int>(green), static_cast<int>(yellow), static_cast<int>(red));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace manymove_color_signal
