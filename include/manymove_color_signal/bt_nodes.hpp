// Copyright 2025

#ifndef MANYMOVE_COLOR_SIGNAL__BT_NODES_HPP_
#define MANYMOVE_COLOR_SIGNAL__BT_NODES_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <signal_column_msgs/msg/signal_color.hpp>

namespace manymove_color_signal
{

class PublishSignalColorAction : public BT::SyncActionNode
{
public:
  explicit PublishSignalColorAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/signal_column", "Topic to publish SignalColor"),
      // Blackboard key names holding bools
      BT::InputPort<std::string>("green_key", "", "Blackboard key for green lamp (bool)"),
      BT::InputPort<std::string>("yellow_key", "", "Blackboard key for yellow lamp (bool)"),
      BT::InputPort<std::string>("red_key", "", "Blackboard key for red lamp (bool)"),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<signal_column_msgs::msg::SignalColor>::SharedPtr pub_;
  std::string topic_;
  // cache last-resolved key names to avoid re-fetching from ports every tick unnecessarily
  std::string green_key_;
  std::string yellow_key_;
  std::string red_key_;
};

// Register helper for BehaviorTree factory
inline void registerManymoveColorSignalNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<PublishSignalColorAction>("PublishSignalColorAction");
}

// Tiny XML builder, mirroring manymove_cpp_trees helpers style
inline std::string buildPublishSignalColorXML(
  const std::string & name,
  const std::string & green_key,
  const std::string & yellow_key,
  const std::string & red_key,
  const std::string & topic = "/signal_column")
{
  std::string xml;
  xml += "<PublishSignalColorAction";
  xml += " name=\"" + name + "\"";
  xml += " topic=\"" + topic + "\"";
  xml += std::string(" green_key=\"") + green_key + "\"";
  xml += std::string(" yellow_key=\"") + yellow_key + "\"";
  xml += std::string(" red_key=\"") + red_key + "\"";
  xml += " />\n";
  return xml;
}

}  // namespace manymove_color_signal

#endif  // MANYMOVE_COLOR_SIGNAL__BT_NODES_HPP_
