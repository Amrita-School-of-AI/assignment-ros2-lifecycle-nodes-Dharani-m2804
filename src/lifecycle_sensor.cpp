#include <chrono>
#include <memory>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class LifecycleSensor : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleSensor()
  : rclcpp_lifecycle::LifecycleNode("lifecycle_sensor")
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Sensor Node created");
  }

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

protected:

  // Called when transitioning from UNCONFIGURED -> INACTIVE
  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "sensor_data", 10);

    RCLCPP_INFO(get_logger(), "Node configured");
    return CallbackReturn::SUCCESS;
  }

  // Called when transitioning from INACTIVE -> ACTIVE
  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    publisher_->on_activate();

    timer_ = this->create_wall_timer(
      1s,
      std::bind(&LifecycleSensor::publish_data, this));

    RCLCPP_INFO(get_logger(), "Node activated");
    return CallbackReturn::SUCCESS;
  }

  // Called when transitioning from ACTIVE -> INACTIVE
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    if (timer_)
    {
      timer_->cancel();
    }

    publisher_->on_deactivate();

    RCLCPP_INFO(get_logger(), "Node deactivated");
    return CallbackReturn::SUCCESS;
  }

  // Called when transitioning from INACTIVE -> UNCONFIGURED
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    timer_.reset();
    publisher_.reset();

    RCLCPP_INFO(get_logger(), "Node cleaned up");
    return CallbackReturn::SUCCESS;
  }

  // Called during shutdown
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Node shutting down");
    return CallbackReturn::SUCCESS;
  }

private:

  void publish_data()
  {
    if (!publisher_->is_activated())
    {
      RCLCPP_WARN(get_logger(), "Publisher not active");
      return;
    }

    auto message = std_msgs::msg::Float64();
    message.data = static_cast<double>(rand() % 100);

    publisher_->publish(message);

    RCLCPP_INFO(get_logger(), "Publishing: %.2f", message.data);
  }

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LifecycleSensor>();
  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
