// vital_listener.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>
#include <string>

class VitalListener : public rclcpp::Node
{
public:
  VitalListener()
  : Node("vital_listener")
  {
    // 受信したい基盤IDのリスト
    const std::vector<int> ids = {10, 11, 12};

    for (int id : ids) {
      const std::string prefix = "vital/ID:" + std::to_string(id) + "/";

      // Heart Rate (Int32)
      hr_subs_.push_back(
        this->create_subscription<std_msgs::msg::Int32>(
          prefix + "hr", 10,
          [this, id](const std_msgs::msg::Int32::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(),
                        "[ID:%d] Heart Rate: %d", id, msg->data);
          }));

      // SpO2 (Float32)
      spo2_subs_.push_back(
        this->create_subscription<std_msgs::msg::Float32>(
          prefix + "spo2", 10,
          [this, id](const std_msgs::msg::Float32::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(),
                        "[ID:%d] SpO2: %.1f", id, msg->data);
          }));

      // Systolic BP (Int32)
      sysbp_subs_.push_back(
        this->create_subscription<std_msgs::msg::Int32>(
          prefix + "sys_bp", 10,
          [this, id](const std_msgs::msg::Int32::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(),
                        "[ID:%d] Systolic BP: %d", id, msg->data);
          }));

      // Diastolic BP (Int32)
      diabp_subs_.push_back(
        this->create_subscription<std_msgs::msg::Int32>(
          prefix + "dia_bp", 10,
          [this, id](const std_msgs::msg::Int32::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(),
                        "[ID:%d] Diastolic BP: %d", id, msg->data);
          }));
    }
  }

private:
  // Subscription をベクタで保持
  std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr>  hr_subs_, sysbp_subs_, diabp_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> spo2_subs_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VitalListener>());
  rclcpp::shutdown();
  return 0;
}
