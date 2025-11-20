#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/send_script.hpp"
#include "std_msgs/msg/bool.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <fstream>
#include <sstream>
#include <nlohmann/json.hpp> // JSON 解析

using json = nlohmann::json;
using namespace std::chrono_literals;

// ---- Service 呼叫 ----
bool send_cmd(const std::string &cmd,
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
                           "Service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        if (result.get()->ok) {
            RCLCPP_INFO_STREAM(node->get_logger(), "Service call OK");
        } else {
            RCLCPP_WARN_STREAM(node->get_logger(), "Service call returned NOT OK");
        }
    } else {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to call service");
        return false;
    }

    return true;
}

// ---- 執行 sequence 的函式 ----
void execute_sequence(std::shared_ptr<rclcpp::Node> node,
                      rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client,
                      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_pub,
                      const json &seq)
{
    for (auto &step : seq) {

        // -------- MOVE 動作 --------
        if (step.contains("mode") && step["mode"] == "absolute") {
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

            RCLCPP_INFO_STREAM(node->get_logger(), "Move: " << oss.str());
            send_cmd(oss.str(), node, client);
        }

        // -------- 吸盤 ON --------
        else if (step.contains("action") && step["action"] == "suck") {
            std_msgs::msg::Bool msg;
            msg.data = true; // 吸盤開
            suction_pub->publish(msg);
            RCLCPP_INFO_STREAM(node->get_logger(), "Suction ON");
        }

        // -------- 吸盤 OFF --------
        else if (step.contains("action") && step["action"] == "release") {
            std_msgs::msg::Bool msg;
            msg.data = false; // 吸盤關
            suction_pub->publish(msg);
            RCLCPP_INFO_STREAM(node->get_logger(), "Suction OFF");
        }

        else {
            RCLCPP_WARN_STREAM(node->get_logger(), "未知 step: " << step.dump());
        }

        // 稍微停頓，避免動作重疊
        rclcpp::sleep_for(1s);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("send_movement");

    // 建立 SendScript Client
    auto client = node->create_client<tm_msgs::srv::SendScript>("send_script");

    // 建立吸盤 Publisher
    auto suction_pub = node->create_publisher<std_msgs::msg::Bool>("/sucker_command", 10);

    // ---- 讀取 JSON 檔案 (用 stringstream) ----
    std::ifstream file("/home/hsiu/tmrdriver_ws/resource/json/move.json");
    if (!file.is_open()) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "無法開啟 move.json");
        return 1;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();

    json data;
    try {
        data = json::parse(buffer.str());
    } catch (const json::parse_error &e) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "JSON 解析失敗: " << e.what());
        return 1;
    }

    // ---- swap_sequence ----
    if (data.contains("swap_sequence")) {
        RCLCPP_INFO_STREAM(node->get_logger(), "開始執行 swap_sequence...");
        execute_sequence(node, client, suction_pub, data["swap_sequence"]);
        RCLCPP_INFO_STREAM(node->get_logger(), "swap_sequence 執行完畢。");
    }

    // ---- 單一步驟 ----
    else if (data.contains("x") && data.contains("y") && data.contains("z")) {
        json single_step;
        single_step["mode"] = "absolute";
        single_step["x"] = data["x"];
        single_step["y"] = data["y"];
        single_step["z"] = data["z"];
        single_step["rx"] = data.value("rx", 0.0);
        single_step["ry"] = data.value("ry", 0.0);
        single_step["rz"] = data.value("rz", 0.0);

        RCLCPP_INFO_STREAM(node->get_logger(), "執行單一步驟...");
        execute_sequence(node, client, suction_pub, json::array({single_step}));
        RCLCPP_INFO_STREAM(node->get_logger(), "單一步驟執行完畢。");
    }

    // ---- assembly_sequence ----
    if (data.contains("assembly_sequence")) {
        RCLCPP_INFO_STREAM(node->get_logger(), "開始執行 assembly_sequence...");
        execute_sequence(node, client, suction_pub, data["assembly_sequence"]);
        RCLCPP_INFO_STREAM(node->get_logger(), "assembly_sequence 執行完畢。");
    }

    rclcpp::shutdown();
    return 0;
}
