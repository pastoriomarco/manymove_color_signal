// Simple demo publisher for signal_column_msgs/SignalColor
// Publishes to /signal_column and cycles through colors.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "signal_column_msgs/msg/signal_color.hpp"

using namespace std::chrono_literals;

class SignalColumnDemoNode : public rclcpp::Node
{
public:
  SignalColumnDemoNode()
  : rclcpp::Node("signal_column_demo")
  {
    publisher_ = this->create_publisher<signal_column_msgs::msg::SignalColor>(
      "/signal_column", rclcpp::QoS(10));

    // Timer to publish a new color message periodically
    timer_ = this->create_wall_timer(1000ms, std::bind(&SignalColumnDemoNode::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "SignalColumnDemoNode started. Publishing on /signal_column");
  }

private:
  void on_timer()
  {
    signal_column_msgs::msg::SignalColor msg;

    // Cycle: green -> yellow -> red -> all off
    switch (state_ % 4) {
      case 0:
        msg.green_color = true;
        msg.yellow_color = false;
        msg.red_color = false;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Color: GREEN");
        break;
      case 1:
        msg.green_color = false;
        msg.yellow_color = true;
        msg.red_color = false;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Color: YELLOW");
        break;
      case 2:
        msg.green_color = false;
        msg.yellow_color = false;
        msg.red_color = true;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Color: RED");
        break;
      default:
        msg.green_color = false;
        msg.yellow_color = false;
        msg.red_color = false;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Color: OFF");
        break;
    }

    publisher_->publish(msg);
    state_++;
  }

  rclcpp::Publisher<signal_column_msgs::msg::SignalColor>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t state_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Keep the main structure similar to bt_client_ur.cpp: create node, use executor, spin.
  auto node = std::make_shared<SignalColumnDemoNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Spin until shutdown; timers handle publishing.
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

