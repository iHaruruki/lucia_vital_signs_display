#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <numeric>

class VitalAggregator : public rclcpp::Node
{
public:
  VitalAggregator()
  : Node("vital_aggregator")
  {
    // ボードIDのリスト
    const std::vector<std::string> ids{"A","B","C"};
    for (const auto &id : ids) {
      add_subscription<std_msgs::msg::Int32>(id, "hr",     20.0,  200.0);
      add_subscription<std_msgs::msg::Float32>(id, "spo2",   80.0,  100.0);
      add_subscription<std_msgs::msg::Int32>(id, "sys_bp", 100.0, 200.0);
      add_subscription<std_msgs::msg::Int32>(id, "dia_bp",  30.0,  90.0);
    }
  }

private:
  // 生データをためるバッファ
  struct Buffer {
    std::vector<double> hr;
    std::vector<double> spo2;
    std::vector<double> sys_bp;
    std::vector<double> dia_bp;
  };
  // 計算済み平均値を保持
  std::map<std::string, std::map<std::string, double>> computed_avg_;
  // トピックごとのバッファ
  std::map<std::string, Buffer> buffers_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;

  template<typename MsgT>
  void add_subscription(
    const std::string &id,
    const std::string &key,
    double valid_min,
    double valid_max)
  {
    const std::string topic = "/vital/" + id + "/" + key;
    auto cb = [this, id, key, valid_min, valid_max](typename MsgT::SharedPtr msg) {
      double value = static_cast<double>(msg->data);

      // 範囲チェック
      if (value <= valid_min || value >= valid_max) {
        RCLCPP_WARN(
          this->get_logger(),
          "Ignored invalid %s/%s: %.2f (valid: %.1f–%.1f)",
          id.c_str(), key.c_str(), value, valid_min, valid_max);
        return;
      }

      // 対応バッファに追加
      auto &buf = buffers_[id];
      std::vector<double>* vec_ptr = nullptr;
      if      (key == "hr")      vec_ptr = &buf.hr;
      else if (key == "spo2")    vec_ptr = &buf.spo2;
      else if (key == "sys_bp")  vec_ptr = &buf.sys_bp;
      else if (key == "dia_bp")  vec_ptr = &buf.dia_bp;

      vec_ptr->push_back(value);
      RCLCPP_INFO(
        this->get_logger(),
        "Buffered %s/%s: %.2f (%zu/15)",
        id.c_str(), key.c_str(), value, vec_ptr->size());

      // 15 個たまったら平均を計算
      if (vec_ptr->size() == 15) {
        double sum = std::accumulate(vec_ptr->begin(), vec_ptr->end(), 0.0);
        double avg = sum / vec_ptr->size();
        // 計算値を保持＆バッファクリア
        computed_avg_[id][key] = avg;
        vec_ptr->clear();

        // ４つすべて揃ったら一括表示して終了
        if (computed_avg_[id].size() == 4) {
          RCLCPP_INFO(this->get_logger(), "--------Average--------");
          RCLCPP_INFO(this->get_logger(), "hr:     %.2f", computed_avg_[id]["hr"]);
          RCLCPP_INFO(this->get_logger(), "spo2:   %.2f", computed_avg_[id]["spo2"]);
          RCLCPP_INFO(this->get_logger(), "sys_bp: %.2f", computed_avg_[id]["sys_bp"]);
          RCLCPP_INFO(this->get_logger(), "dia_bp: %.2f", computed_avg_[id]["dia_bp"]);
          RCLCPP_INFO(this->get_logger(), "-----------------------");
          rclcpp::shutdown();
        }
      }
    };

    auto sub = this->create_subscription<MsgT>(topic, 10, cb);
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
