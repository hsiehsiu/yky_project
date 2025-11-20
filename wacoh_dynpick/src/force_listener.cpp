#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <fstream>
#include <nlohmann/json.hpp>   // 使用 JSON (需要在 CMakeLists.txt 中加入)

using json = nlohmann::json;

class ForceListenerNode : public rclcpp::Node
{
public:
    ForceListenerNode()
        : Node("force_listener_node")
    {
        // 訂閱 WrenchStamped topic
        subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/force",    // ← 記得改你的 topic
            10,
            std::bind(&ForceListenerNode::force_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Force listener node started.");
    }

private:
    void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        double fx = msg->wrench.force.x;
        double fy = msg->wrench.force.y;
        double fz = msg->wrench.force.z;

        // 設定 JSON 格式
        json data;
        data["force"]["x"] = fx;
        data["force"]["y"] = fy;
        data["force"]["z"] = fz;

        // 寫入 force.json
        std::ofstream file("/home/hsiu/tmrdriver_ws/resource/json/force.json");
        file << data.dump(4); 
        file.close();

        RCLCPP_INFO(this->get_logger(),
                    "Force updated → x: %.3f, y: %.3f, z: %.3f",
                    fx, fy, fz);
    }

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceListenerNode>());
    rclcpp::shutdown();
    return 0;
}
