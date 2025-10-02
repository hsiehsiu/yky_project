#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/send_script.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>
#include <fstream>
#include <sstream>
#include <nlohmann/json.hpp> // JSON 解析
using json = nlohmann::json;
using namespace std::chrono_literals;

// ---- Service 呼叫 ----
bool send_cmd(std::string cmd,
              std::shared_ptr<rclcpp::Node> node,
              rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client)
{
    auto request = std::make_shared<tm_msgs::srv::SendScript::Request>();
    request->id = "demo";
    request->script = cmd;

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),
                                "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                           "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        if (result.get()->ok) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "OK");
        } else {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "not OK");
        }
    } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),
                            "Failed to call service");
    }

    return true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("send_movement");

    // 建立 SendScript Client
    auto client = node->create_client<tm_msgs::srv::SendScript>("send_movement");

    // 讀取 JSON 指令
    std::ifstream file("/home/hudenxiao/tmdriver_ws/src/tmr_ros2/json/move.json");
    if (!file.is_open()) {
        std::cerr << "無法開啟 move.json" << std::endl;
        return 1;
    }

    json data;
    file >> data;

    // ---- 判斷是單一步驟還是多步驟 ----
    if (data.contains("swap_sequence")) {
        auto seq = data["swap_sequence"];
        for (auto &step : seq) {
            double x = step.value("x", 0.0);
            double y = step.value("y", 0.0);
            double z = step.value("z", 0.0);
            double rx = step.value("rx", 0.0);
            double ry = step.value("ry", 0.0);
            double rz = step.value("rz", 0.0);

            std::ostringstream oss;
            oss << "PTP(\"CPP\","
                << x << "," << y << "," << z << ","
                << rx << "," << ry << "," << rz
                << ",35,200,0,false)";
            std::string cmd = oss.str();

            RCLCPP_INFO_STREAM(node->get_logger(), 
                "執行 swap_sequence 動作: " << step.value("action", "move"));
            RCLCPP_INFO_STREAM(node->get_logger(), "Send Command: " << cmd);

            send_cmd(cmd, node, client);

            // 稍微停頓，避免動作重疊
            rclcpp::sleep_for(500ms);
        }
    } else {
        double x = data.value("x", 0.0);
        double y = data.value("y", 0.0);
        double z = data.value("z", 0.0);
        double rx = data.value("rx", 0.0);
        double ry = data.value("ry", 0.0);
        double rz = data.value("rz", 0.0);

        std::ostringstream oss;
        oss << "PTP(\"CPP\","
            << x << "," << y << "," << z << ","
            << rx << "," << ry << "," << rz
            << ",35,200,0,false)";
        std::string cmd = oss.str();

        RCLCPP_INFO_STREAM(node->get_logger(), "Send Command: " << cmd);

        send_cmd(cmd, node, client);
    }

    rclcpp::shutdown();
    return 0;
}
