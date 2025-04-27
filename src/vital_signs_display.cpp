#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <memory>
#include <vector>
#include <string>
#include <type_traits>

class VitalAggregator : public rclcpp::Node
{
public:
  VitalAggregator()
  : Node("vital_aggregator")
  {
    // /vital/A, B, C の各トピックを購読し、受信時にログ出力
    for (const auto &id : {"A","B","C"}) {
      const std::string base = "/vital/" + std::string(id) + "/";
      create_logger<std_msgs::msg::Int32>(base + "hr");
      create_logger<std_msgs::msg::Float32>(base + "spo2");
      create_logger<std_msgs::msg::Int32>(base + "sys_bp");
      create_logger<std_msgs::msg::Int32>(base + "dia_bp");
    }
  }

private:
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;

  template<typename MsgT>
  void create_logger(const std::string &topic)
  {
    auto sub = this->create_subscription<MsgT>(
      topic, 10,
      [this, topic](typename MsgT::SharedPtr msg) {
        double value = 0.0;
        if constexpr (std::is_same_v<MsgT, std_msgs::msg::Int32>) {
          value = static_cast<double>(msg->data);
        } else if constexpr (std::is_same_v<MsgT, std_msgs::msg::Float32>) {
          value = static_cast<double>(msg->data);
        }
        RCLCPP_INFO(this->get_logger(), "Received %s: %.2f", topic.c_str(), value);
      }
    );
    subs_.push_back(sub);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VitalAggregator>());
  rclcpp::shutdown();
  return 0;
}
