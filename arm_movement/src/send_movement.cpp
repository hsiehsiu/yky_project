#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/send_script.hpp"
#include "std_msgs/msg/bool.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>
#include <fstream>
#include <sstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;

// -------- Service 呼叫 --------
bool send_cmd(const std::string &cmd,
              std::shared_ptr<rclcpp::Node> node,
              rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client)
{
    auto request = std::make_shared<tm_msgs::srv::SendScript::Request>();
    request->id = "demo";
    request->script = cmd;

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Service interrupted");
            return false;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for send_script service...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        return result.get()->ok;
    }
    return false;
}

// -------- 讀 force.json --------
double read_force_z(const std::string &filename)
{
    double fz = 0.0;
    std::ifstream file(filename);
    if (file.is_open())
    {
        try
        {
            json j;
            file >> j;
            fz = j["force"].value("z", 0.0);
        }
        catch (std::exception &e)
        {
            std::cerr << "讀 force.json 發生錯誤: " << e.what() << std::endl;
        }
    }
    else
    {
        std::cerr << "Cannot open force.json" << std::endl;
    }
    return fz;
}

// -------- 安全提起動作 --------
void move_safe_height(std::shared_ptr<rclcpp::Node> node,
                      rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client)
{
    std::string cmd = "PTP(\"CPP\",433.72,78.29,440,-180.15,0,135,35,200,0,false)";
    RCLCPP_WARN(node->get_logger(), "[KILLSWITCH] Moving to safe height!");
    send_cmd(cmd, node, client);
}

// -------- 執行動作序列（動作後檢查 force，force 太低直接中止） --------
bool execute_sequence(std::shared_ptr<rclcpp::Node> node,
                      rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client,
                      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_pub,
                      const json &seq,
                      const std::string &force_file)
{
    const double MIN_FORCE = -20.0;

    // 判斷是不是單一步驟（沒有包含 array）
    if (!seq.is_array()) {
        json tmp = json::array();
        tmp.push_back(seq);
        return execute_sequence(node, client, suction_pub, tmp, force_file);
    }

    for (auto &step : seq)
    {
        // -------- MOVE --------
        if (step.contains("mode") && step["mode"] == "absolute") {
            double x = step.value("x", 0.0);
            double y = step.value("y", 0.0);
            double z = step.value("z", 0.0);
            double rx = step.value("rx", 0.0);
            double ry = step.value("ry", 0.0);
            double rz = step.value("rz", 0.0);

            std::ostringstream oss;
            oss << "PTP(\"CPP\"," << x << "," << y << "," << z << ","
                << rx << "," << ry << "," << rz
                << ",35,200,0,false)";
            RCLCPP_INFO(node->get_logger(), "Move: %s", oss.str().c_str());
            send_cmd(oss.str(), node, client);
            rclcpp::sleep_for(std::chrono::seconds(2));
        }

        // -------- 吸盤 ON --------
        else if (step.contains("action") && step["action"] == "suck") {
            std_msgs::msg::Bool msg;
            msg.data = true;
            suction_pub->publish(msg);
        }

        // -------- 吸盤 OFF --------
        else if (step.contains("action") && step["action"] == "release") {
            std_msgs::msg::Bool msg;
            msg.data = false;
            suction_pub->publish(msg);
        }

        rclcpp::sleep_for(std::chrono::seconds(5));

        // ---- 動作後檢查 force ----
        double fz = read_force_z(force_file);
        if (fz < MIN_FORCE) {
            RCLCPP_ERROR(node->get_logger(),
                         "[KILLSWITCH] FORCE TOO LOW AFTER STEP (%.2f) — Moving to safe height and abort sequence",
                         fz);
            move_safe_height(node, client);
            return false; // 直接中止 sequence
        }
    }

    return true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("send_movement");

    auto client = node->create_client<tm_msgs::srv::SendScript>("send_script");
    auto suction_pub = node->create_publisher<std_msgs::msg::Bool>("/sucker_command", 10);

    std::string move_file = "/home/hsiu/tmrdriver_ws/resource/json/move.json";
    std::string force_file = "/home/hsiu/tmrdriver_ws/resource/json/force.json";

    // ---- 讀 JSON ----
    std::ifstream file(move_file);
    if (!file.is_open()) {
        RCLCPP_ERROR(node->get_logger(), "Cannot open move.json");
        return 1;
    }
    json data = json::parse(file);

    // ---- 判斷 sequence 型態並執行 ----
    if (data.contains("swap_sequence")) {
        if (!execute_sequence(node, client, suction_pub, data["swap_sequence"], force_file)) {
            RCLCPP_ERROR(node->get_logger(), "Sequence interrupted by KILLSWITCH");
            return 0;
        }
    }
    else if (data.contains("assembly_sequence")) {
        if (!execute_sequence(node, client, suction_pub, data["assembly_sequence"], force_file)) {
            RCLCPP_ERROR(node->get_logger(), "Sequence interrupted by KILLSWITCH");
            return 0;
        }
    }
    else {
        // 單一步驟直接執行
        if (!execute_sequence(node, client, suction_pub, data, force_file)) {
            RCLCPP_ERROR(node->get_logger(), "Single step interrupted by KILLSWITCH");
            return 0;
        }
    }

    rclcpp::shutdown();
    return 0;
}
