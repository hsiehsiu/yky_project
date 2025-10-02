#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <fstream>
#include <iomanip>
#include <cmath>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// 四元數轉 Euler (度)
std::array<double, 3> quaternion_to_euler(double x, double y, double z, double w)
{
    double rx, ry, rz;

    // Roll
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    rx = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        ry = std::copysign(M_PI / 2, sinp);
    else
        ry = std::asin(sinp);

    // Yaw
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    rz = std::atan2(siny_cosp, cosy_cosp);

    return {rx * 180.0 / M_PI, ry * 180.0 / M_PI, rz * 180.0 / M_PI};
}

class FeedbackSubscriber : public rclcpp::Node
{
public:
    FeedbackSubscriber() : Node("feedback_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tool_pose", 10,
            std::bind(&FeedbackSubscriber::callback, this, std::placeholders::_1));

        json_path_ = "/home/hudenxiao/tmdriver_ws/src/tmr_ros2/json/feedback_pose.json";
        RCLCPP_INFO(this->get_logger(), "Subscriber 啟動，座標會存到 %s", json_path_.c_str());
    }

private:
    void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double x = msg->pose.position.x * 1000;
        double y = msg->pose.position.y * 1000;
        double z = msg->pose.position.z * 1000;

        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;

        auto euler = quaternion_to_euler(qx, qy, qz, qw);

        json j;
        j["tool_pose"]["x"] = x;
        j["tool_pose"]["y"] = y;
        j["tool_pose"]["z"] = z;
        j["tool_pose"]["rx"] = euler[0];
        j["tool_pose"]["ry"] = euler[1];
        j["tool_pose"]["rz"] = euler[2];

        std::ofstream file(json_path_);
        if (file.is_open())
        {
            file << std::setw(4) << j << std::endl;
            RCLCPP_INFO(this->get_logger(),
                        "座標已更新：x=%.3f y=%.3f z=%.3f rx=%.2f ry=%.2f rz=%.2f",
                        x, y, z, euler[0], euler[1], euler[2]);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    std::string json_path_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeedbackSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
