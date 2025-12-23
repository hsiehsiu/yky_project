#!/usr/bin/env python3
import os
import json
import requests
import subprocess
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from ament_index_python.packages import get_package_share_directory

# ------------------ OpenRouter API ------------------
API_KEY = ""
URL = "https://openrouter.ai/api/v1/chat/completions"

headers = {
    "Authorization": f"Bearer {API_KEY}",
    "Content-Type": "application/json",
    "HTTP-Referer": "http://localhost",
    "X-Title": "TerminalChatLLM"
}

ROW_OFFSETS = {
    "OFFSET_Q": {"x": -1.0, "y": 0.0,  "z": -158.0}, 
    "OFFSET_A": {"x": -0.5, "y": 0.0,  "z": -161.5},
    "OFFSET_Z": {"x": 0.0, "y": 0.0, "z": -162.0},
    "OFFSET_DEFAULT": {"x": 0.0, "y": 0.0, "z": 0.0}
}

# 根據按鍵名稱取得對應行數的修正值
def get_row_offset(key_name):
    """根據按鍵在 QWERTY 鍵盤的位置回傳對應的修正值"""
    key = key_name.lower()
    row_q = ['q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p']
    row_a = ['a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l']
    row_z = ['z', 'x', 'c', 'v', 'b', 'n', 'm']
    
    if key in row_q:
        return ROW_OFFSETS["OFFSET_Q"]
    elif key in row_a:
        return ROW_OFFSETS["OFFSET_A"]
    elif key in row_z:
        return ROW_OFFSETS["OFFSET_Z"]
    else:
        return ROW_OFFSETS["OFFSET_DEFAULT"]


class LLMControlNode(Node):
    def __init__(self):
        super().__init__('llm_control_node')
        
        # 建立 Publisher
        self.sucker_pub = self.publisher = self.create_publisher(Bool, '/sucker_command', 10)
        self.move_pub = self.create_publisher(String, '/llm_move_command', 10)
        
        self.get_logger().info('LLM Control Node 已啟動')
        
        # 檔案路徑
        self.keycaps_file = "/home/hsiu/tmrdriver_ws/resource/json/keyboard_layout.json"
        self.feedback_file = "/home/hsiu/tmrdriver_ws/resource/json/feedback_pose.json"
        self.move_file = "/home/hsiu/tmrdriver_ws/resource/json/move.json"
        self.last_move_file = "/home/hsiu/tmrdriver_ws/resource/json/last_move.json"
        self.prompt_file = os.path.join(get_package_share_directory('llm'), 'resource', 'prompt.txt')
        
        # 載入 system prompt
        if not os.path.exists(self.prompt_file):
            self.get_logger().error(f"找不到 {self.prompt_file}，請先建立 system prompt 檔案")
            return
        with open(self.prompt_file, "r", encoding="utf-8") as f:
            self.system_prompt = f.read()
        
        self.messages = [{"role": "system", "content": self.system_prompt}]
        
        # 在背景執行聊天迴圈
        self.chat_thread = threading.Thread(target=self.chat_loop, daemon=True)
        self.chat_thread.start()

    def ask_openrouter(self, messages, model="openai/gpt-3.5-turbo"):
        payload = {"model": model, "messages": messages, "temperature": 0.7}
        response = requests.post(URL, headers=headers, json=payload)
        if response.status_code != 200:
            self.get_logger().error(f"錯誤 {response.status_code}: {response.text}")
            return '{"error": "API 請求失敗"}'
        return response.json()["choices"][0]["message"]["content"]

    def get_keycap_position(self, key_name):
        if not os.path.exists(self.keycaps_file):
            self.get_logger().error(f"找不到 {self.keycaps_file}，請先建立鍵帽座標檔")
            return None
        with open(self.keycaps_file, "r", encoding="utf-8") as f:
            keycaps = json.load(f)
        return keycaps.get(key_name, None)

    def get_feedback_pose(self):
        if not os.path.exists(self.feedback_file):
            self.get_logger().error(f"找不到 {self.feedback_file}，請先建立手臂狀態 JSON")
            return None
        with open(self.feedback_file, "r", encoding="utf-8") as f:
            pose = json.load(f)
        return pose

    def publish_sucker_command(self, suck: bool):
        """發布吸盤控制命令"""
        msg = Bool()
        msg.data = suck
        self.sucker_pub.publish(msg)
        action_name = "吸取" if suck else "釋放"
        self.get_logger().info(f"發布吸盤指令: {action_name} → {suck}")

    def publish_move_command(self, json_data: dict):
        """發布移動命令到 /llm_move_command"""
        msg = String()
        msg.data = json.dumps(json_data)
        self.move_pub.publish(msg)
        self.get_logger().info(f"發布移動命令")

    def execute_movement(self, json_data: dict):
        """執行移動命令（寫入檔案並呼叫 send_movement）"""
        # 儲存上一次位置
        with open(self.last_move_file, 'w', encoding="utf-8") as f:
            json.dump(json_data, f, indent=2)

        # 寫入 move.json
        with open(self.move_file, 'w', encoding="utf-8") as f:
            json.dump(json_data, f, indent=2)

        # 發布到 topic
        self.publish_move_command(json_data)

        # 呼叫 send_movement 節點執行
        subprocess.run(["ros2", "run", "arm_movement", "send_movement"])

    def chat_loop(self):
        """聊天迴圈 - 在背景執行"""
        print("\n聊天啟動，輸入 'q' 離開。", flush=True)
        
        while rclpy.ok():
            try:
                user_input = input("我：")
                if user_input.lower() in ["exit", "quit", "q"]:
                    self.get_logger().info("使用者要求離開")
                    rclpy.shutdown()
                    break

                self.messages.append({"role": "user", "content": user_input})
                
                reply = self.ask_openrouter(self.messages)
                print(f"機器人：{reply}")
                self.messages.append({"role": "assistant", "content": reply})

                try:
                    json_data = json.loads(reply)
                except json.JSONDecodeError:
                    print("無法解析，請重新輸入。")
                    continue

                if "error" in json_data:
                    print("無法理解您的指令，請重新輸入！")
                    continue

                # 處理各種命令類型
                json_data = self.process_command(json_data)
                if json_data is None:
                    continue

                # 執行移動
                self.execute_movement(json_data)

            except Exception as e:
                self.get_logger().error(f"發生錯誤：{e}")

    def process_command(self, json_data: dict):
        """處理 LLM 回傳的命令"""
        
        # 回到上次位置
        if "return_last" in json_data and json_data["return_last"] is True:
            if not os.path.exists(self.last_move_file):
                print("目前沒有上一次的位置記錄。")
                return None
            with open(self.last_move_file, "r", encoding="utf-8") as f:
                json_data = json.load(f)
            print("回到上一次的座標。")
            return json_data

        # --- swap 功能 ---
        elif "swap" in json_data:
            return self.handle_swap(json_data)

        # --- action 功能（吸盤控制） ---
        elif "action" in json_data:
            return self.handle_action(json_data)

        # --- keycap 功能 ---
        elif "keycap" in json_data:
            return self.handle_keycap(json_data)

        # --- 相對 / 絕對移動 ---
        elif "mode" in json_data:
            return self.handle_mode(json_data)

        # --- spell 功能 ---
        elif "spell" in json_data:
            return self.handle_spell(json_data)

        # --- movetokeycap 功能 ---
        elif "movetokeycap" in json_data:
            return self.handle_movetokeycap(json_data)

        else:
            print("模型輸出不包含可執行的指令欄位。")
            return None

    def handle_action(self, json_data: dict):
        """處理吸盤控制命令"""
        action = json_data["action"]
        if action not in ["suck", "release", "pick", "place"]:
            print(f"不支援的動作：{action}")
            return None

        if action in ["suck", "pick"]:
            self.publish_sucker_command(True)
        elif action in ["release", "place"]:
            self.publish_sucker_command(False)

        return None  # action 不需要後續的 execute_movement

    def handle_swap(self, json_data: dict):
        """處理交換鍵帽命令"""
        k1, k2 = json_data["swap"]
        pos1 = self.get_keycap_position(k1)
        pos2 = self.get_keycap_position(k2)
        if not pos1 or not pos2:
            print(f"找不到鍵帽 {k1} 或 {k2} 的座標")
            return None

        feedback_pose = self.get_feedback_pose()
        if feedback_pose:
            rx, ry, rz = feedback_pose["tool_pose"]["rx"], feedback_pose["tool_pose"]["ry"], feedback_pose["tool_pose"]["rz"]
        else:
            rx, ry, rz = -173.0, -12.27, 129.31

        offset1 = get_row_offset(k1)
        offset2 = get_row_offset(k2)

        base1_x, base1_y, base1_z = pos1["x"] + 3, pos1["y"] - 20, pos1["z"]
        corrected1_x = base1_x + offset1["x"]
        corrected1_y = base1_y + offset1["y"]
        pick1_z = base1_z + offset1["z"]

        base2_x, base2_y, base2_z = pos2["x"] + 3, pos2["y"] - 20, pos2["z"]
        corrected2_x = base2_x + offset2["x"]
        corrected2_y = base2_y + offset2["y"]
        pick2_z = base2_z + offset2["z"]

        idle_pos = {"x": 409, "y": -30.663, "z": 440.0}
        swap_instructions = [
            {"mode": "absolute", "x": base1_x, "y": base1_y, "z": base1_z, "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": corrected1_x, "y": corrected1_y, "z": pick1_z, "rx": rx, "ry": ry, "rz": rz},
            {"action": "suck"},
            {"mode": "absolute", "x": corrected1_x, "y": corrected1_y, "z": 440, "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"], "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"] - 155, "rx": rx, "ry": ry, "rz": rz},
            {"action": "release"},
            {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": 440, "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": base2_x, "y": base2_y, "z": base2_z, "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": corrected2_x, "y": corrected2_y, "z": pick2_z, "rx": rx, "ry": ry, "rz": rz},
            {"action": "suck"},
            {"mode": "absolute", "x": corrected2_x, "y": corrected2_y, "z": 440, "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": base1_x, "y": base1_y, "z": base1_z, "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": corrected1_x, "y": corrected1_y, "z": pick1_z, "rx": rx, "ry": ry, "rz": rz},
            {"action": "release"},
            {"mode": "absolute", "x": corrected1_x, "y": corrected1_y, "z": 440, "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"], "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"] - 155, "rx": rx, "ry": ry, "rz": rz},
            {"action": "suck"},
            {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"], "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": base2_x, "y": base2_y, "z": base2_z, "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": corrected2_x, "y": corrected2_y, "z": pick2_z, "rx": rx, "ry": ry, "rz": rz},
            {"action": "release"},
            {"mode": "absolute", "x": 433.72, "y": 78.29, "z": 440, "rx": rx, "ry": ry, "rz": rz}
        ]
        return {"swap_sequence": swap_instructions}

    def handle_keycap(self, json_data: dict):
        """處理組裝鍵帽命令"""
        key_name = json_data["keycap"]

        idle_pos = {"x": 409.0, "y": -30.663, "z": 440.0}
        pos_pick = self.get_keycap_position(key_name)
        if not pos_pick:
            print(f"找不到鍵帽 {key_name} 的座標")
            return None

        keyboard_layout_file = "/home/hsiu/tmrdriver_ws/resource/json/keyboard_layout.json"
        if not os.path.exists(keyboard_layout_file):
            print(f"找不到 {keyboard_layout_file}")
            return None

        with open(keyboard_layout_file, "r", encoding="utf-8") as f:
            keyboard_layout = json.load(f)

        if key_name not in keyboard_layout:
            print(f"keyboard_layout 中找不到 {key_name} 的座標")
            return None

        pos_place = keyboard_layout[key_name]
        offset = get_row_offset(key_name)

        base_x = pos_place["x"] + 3
        base_y = pos_place["y"] - 20
        base_z = pos_place["z"]
        corrected_x = base_x + offset["x"]
        corrected_y = base_y + offset["y"]
        pick_z = base_z + offset["z"]

        feedback_pose = self.get_feedback_pose()
        if feedback_pose:
            rx = feedback_pose["tool_pose"]["rx"]
            ry = feedback_pose["tool_pose"]["ry"]
            rz = feedback_pose["tool_pose"]["rz"]
        else:
            rx, ry, rz = 180.0, 0.0, 133.0

        sequence = [
            {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"], "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"] - 160, "rx": rx, "ry": ry, "rz": rz},
            {"action": "suck"},
            {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"], "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": base_x, "y": base_y, "z": base_z, "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": corrected_x, "y": corrected_y, "z": pick_z, "rx": rx, "ry": ry, "rz": rz},
            {"action": "release"},
            {"mode": "absolute", "x": corrected_x, "y": corrected_y, "z": 440.0, "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": 433.72, "y": 78.29, "z": 440.0, "rx": rx, "ry": ry, "rz": rz}
        ]
        return {"assembly_sequence": sequence}

    def handle_mode(self, json_data: dict):
        """處理相對/絕對移動命令"""
        feedback_pose = self.get_feedback_pose()
        if not feedback_pose:
            print("找不到手臂當前座標，無法執行")
            return None

        abs_pose = [
            feedback_pose["tool_pose"]["x"],
            feedback_pose["tool_pose"]["y"],
            feedback_pose["tool_pose"]["z"],
            feedback_pose["tool_pose"]["rx"],
            feedback_pose["tool_pose"]["ry"],
            feedback_pose["tool_pose"]["rz"]
        ]

        if json_data["mode"] == "relative":
            abs_pose[0] += json_data.get("x", 0.0)
            abs_pose[1] += json_data.get("y", 0.0)
            abs_pose[2] += json_data.get("z", 0.0)
            abs_pose[3] += json_data.get("rx", 0.0)
            abs_pose[4] += json_data.get("ry", 0.0)
            abs_pose[5] += json_data.get("rz", 0.0)

            return {
                "mode": "absolute",
                "x": abs_pose[0], "y": abs_pose[1], "z": abs_pose[2],
                "rx": abs_pose[3], "ry": abs_pose[4], "rz": abs_pose[5]
            }
        return json_data

    def handle_spell(self, json_data: dict):
        """處理拼字命令"""
        action_list = json_data["spell"]
        
        place_base = {"x": 407.8, "y": -30.663, "z": 440.0}
        y_step = 19.0
        spell_sequence = []
        
        chars = [item["char"] for item in action_list]
        if len(chars) != len(set(chars)):
            print("包含重複字母，請輸入別的單字")
            return None

        feedback_pose = self.get_feedback_pose()
        if feedback_pose:
            rx = feedback_pose["tool_pose"]["rx"]
            ry = feedback_pose["tool_pose"]["ry"]
            rz = feedback_pose["tool_pose"]["rz"]
        else:
            rx, ry, rz = 180.0, 0.0, 133.0

        for idx, step in enumerate(action_list):
            char_target = step["char"].lower()
            offset_key = step["correction"]
            
            pos = self.get_keycap_position(char_target)
            if not pos:
                print(f"找不到鍵帽 {char_target} 的座標")
                continue

            correction = ROW_OFFSETS.get(offset_key, {"x": 0.0, "y": 0.0, "z": 0.0})

            base_x = pos["x"] + 3
            base_y = pos["y"] - 20
            base_z = pos["z"]

            corrected_x = base_x + correction["x"]
            corrected_y = base_y + correction["y"]
            pick_z = base_z + correction["z"]

            place_x = place_base["x"]
            place_y = place_base["y"] + idx * y_step
            place_z = place_base["z"]

            spell_sequence.extend([
                {"mode": "absolute", "x": base_x, "y": base_y, "z": base_z, "rx": rx, "ry": ry, "rz": rz},
                {"mode": "absolute", "x": corrected_x, "y": corrected_y, "z": pick_z, "rx": rx, "ry": ry, "rz": rz},
                {"action": "suck"},
                {"mode": "absolute", "x": corrected_x, "y": corrected_y, "z": 440, "rx": rx, "ry": ry, "rz": rz},
                {"mode": "absolute", "x": place_x, "y": place_y, "z": place_z, "rx": rx, "ry": ry, "rz": rz},
                {"mode": "absolute", "x": place_x, "y": place_y, "z": pick_z, "rx": rx, "ry": ry, "rz": rz},
                {"action": "release"},
                {"mode": "absolute", "x": place_x, "y": place_y, "z": 440, "rx": rx, "ry": ry, "rz": rz},
                {"mode": "absolute", "x": 433.72, "y": 78.29, "z": 440, "rx": rx, "ry": ry, "rz": rz}
            ])

        return {"spell_sequence": spell_sequence}

    def handle_movetokeycap(self, json_data: dict):
        """處理移動到鍵帽命令"""
        key_name = json_data["movetokeycap"]
        pos = self.get_keycap_position(key_name)
        if not pos:
            print(f"找不到鍵帽 {key_name} 的座標")
            return None

        feedback_pose = self.get_feedback_pose()
        if feedback_pose:
            rx, ry, rz = feedback_pose["tool_pose"]["rx"], feedback_pose["tool_pose"]["ry"], feedback_pose["tool_pose"]["rz"]
        else:
            rx, ry, rz = 180.0, 0, 133

        offset = get_row_offset(key_name)

        base_x = pos["x"] + 3
        base_y = pos["y"] - 20
        base_z = pos["z"]
        corrected_x = base_x + offset["x"]
        corrected_y = base_y + offset["y"]
        pick_z = base_z + offset["z"]

        move_sequence = [
            {"mode": "absolute", "x": base_x, "y": base_y, "z": base_z, "rx": rx, "ry": ry, "rz": rz},
            {"mode": "absolute", "x": corrected_x, "y": corrected_y, "z": pick_z, "rx": rx, "ry": ry, "rz": rz}
        ]
        return {"movetokeycap_sequence": move_sequence}


def main(args=None):
    rclpy.init(args=args)
    
    node = LLMControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()