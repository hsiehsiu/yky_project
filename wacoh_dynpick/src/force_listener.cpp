#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <cmath>

using json = nlohmann::json;

class ForceListenerNode : public rclcpp::Node
{
public:
    ForceListenerNode()
        : Node("force_listener_node"),
          last_fx_(0.0), last_fy_(0.0), last_fz_(0.0),
          force_threshold_(0.1),  // [優化] 只有變化超過此閾值才更新檔案
          write_count_(0),
          decimation_(10)  // [優化] 每 N 次 callback 才允許寫入一次
    {
        // 訂閱 WrenchStamped topic
        subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/force",
            10,
            std::bind(&ForceListenerNode::force_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Force listener node started (optimized).");
    }

private:
    void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        double fx = msg->wrench.force.x;
        double fy = msg->wrench.force.y;
        double fz = msg->wrench.force.z;

        write_count_++;

        // [優化] 只有在達到 decimation 間隔 且 數值有顯著變化時才寫入
        if (write_count_ % decimation_ != 0) {
            return;
        }

        // 檢查是否有足夠的變化
        double dx = std::abs(fx - last_fx_);
        double dy = std::abs(fy - last_fy_);
        double dz = std::abs(fz - last_fz_);

        if (dx < force_threshold_ && dy < force_threshold_ && dz < force_threshold_) {
            return;  // 變化太小，跳過寫入
        }

        // 更新上次數值
        last_fx_ = fx;
        last_fy_ = fy;
        last_fz_ = fz;

        // 寫入 force.json
        json data;
        data["force"]["x"] = fx;
        data["force"]["y"] = fy;
        data["force"]["z"] = fz;

        std::ofstream file("/home/hsiu/tmrdriver_ws/resource/json/force.json");
        if (file.is_open()) {
            file << data.dump(4);
            file.close();
        }

        // [優化] 移除每次都打印的 log，改為只在啟動時打印一次
    }

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
    
    // [優化] 用於減少寫入頻率的變數
    double last_fx_, last_fy_, last_fz_;
    double force_threshold_;
    int write_count_;
    int decimation_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceListenerNode>());
    rclcpp::shutdown();
    return 0;
}
