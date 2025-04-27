#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <memory>
#include <map>
#include <string>
#include <vector>
#include <chrono>
#include <mutex>

using namespace std::chrono_literals;

class VitalAggregator : public rclcpp::Node
{
public:
  VitalAggregator()
  : Node("vital_aggregator")
  {
    for (auto id : {"A","B","C"}) {
      // hr
      subs_.push_back(this->create_subscription<std_msgs::msg::Int32>(
        "/vital/" + std::string(id) + "/hr", 10,
        [this, id](const std_msgs::msg::Int32::SharedPtr msg){
          push_value(id + std::string("_hr"), msg->data);
        }
      ));
      // spo2
      subs_.push_back(this->create_subscription<std_msgs::msg::Float32>(
        "/vital/" + std::string(id) + "/spo2", 10,
        [this, id](const std_msgs::msg::Float32::SharedPtr msg){
          push_value(id + std::string("_spo2"), msg->data);
        }
      ));
      // sys_bp
      subs_.push_back(this->create_subscription<std_msgs::msg::Int32>(
        "/vital/" + std::string(id) + "/sys_bp", 10,
        [this, id](const std_msgs::msg::Int32::SharedPtr msg){
          push_value(id + std::string("_sys_bp"), msg->data);
        }
      ));
      // dia_bp
      subs_.push_back(this->create_subscription<std_msgs::msg::Int32>(
        "/vital/" + std::string(id) + "/dia_bp", 10,
        [this, id](const std_msgs::msg::Int32::SharedPtr msg){
          push_value(id + std::string("_dia_bp"), msg->data);
        }
      ));
    }

    timer_ = this->create_wall_timer(1s, std::bind(&VitalAggregator::print_averages, this));
  }

private:
  struct Stats {
    double sum = 0.0;
    size_t count = 0;
  };

  void push_value(const std::string &key, double value) {
    std::lock_guard<std::mutex> lk(mtx_);
    stats_[key].sum   += value;
    stats_[key].count += 1;
  }

  void print_averages() {
    std::lock_guard<std::mutex> lk(mtx_);
    RCLCPP_INFO(get_logger(), "---- Vital Averages ----");
    for (auto &key : {"A_hr","A_spo2","A_sys_bp","A_dia_bp",
                      "B_hr","B_spo2","B_sys_bp","B_dia_bp",
                      "C_hr","C_spo2","C_sys_bp","C_dia_bp"})
    {
      auto &st = stats_[key];
      if (st.count > 0) {
        double avg = st.sum / static_cast<double>(st.count);
        RCLCPP_INFO(get_logger(), "  %s: %.2f (%zu samples)", key, avg, st.count);
      } else {
        RCLCPP_WARN(get_logger(), "  %s: no data", key);
      }
      st.sum = 0.0;
      st.count = 0;
    }
    RCLCPP_INFO(get_logger(), "------------------------");
  }

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::string, Stats> stats_;
  std::mutex mtx_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VitalAggregator>());
  rclcpp::shutdown();
  return 0;
}
