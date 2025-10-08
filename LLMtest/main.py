import os
import json
import base64
import requests
import subprocess

# ------------------ OpenRouter API ------------------
API_KEY = "my API key"
URL = "https://openrouter.ai/api/v1/chat/completions"

headers = {
    "Authorization": f"Bearer {API_KEY}",
    "Content-Type": "application/json",
    "HTTP-Referer": "http://localhost",
    "X-Title": "TerminalChatLLM"
}

def ask_openrouter(messages, model="openai/gpt-3.5-turbo"):
    payload = {"model": model, "messages": messages, "temperature": 0.7}
    response = requests.post(URL, headers=headers, json=payload)
    if response.status_code != 200:
        print(f"錯誤 {response.status_code}: {response.text}")
        return "[錯誤] 請確認 API Key 或模型名稱是否正確"
    return response.json()["choices"][0]["message"]["content"]

# ------------------ JSON 讀取 ------------------
def get_keycap_position(key_name, keycaps_file):
    if not os.path.exists(keycaps_file):
        print(f"找不到 {keycaps_file}，請先建立鍵帽座標檔")
        return None
    with open(keycaps_file, "r", encoding="utf-8") as f:
        keycaps = json.load(f)
    return keycaps.get(key_name, None)

def get_feedback_pose(feedback_file):
    if not os.path.exists(feedback_file):
        print(f"找不到 {feedback_file}，請先建立手臂狀態 JSON")
        return None
    with open(feedback_file, "r", encoding="utf-8") as f:
        pose = json.load(f)
    return pose

# ------------------ 主程式 ------------------
def main():
    print("聊天啟動，輸入 'exit' 離開。", flush=True)

    keycaps_file = "/home/hudenxiao/tmdriver_ws/src/tmr_ros2/json/keycap_coordinate.json"
    feedback_file = "/home/hudenxiao/tmdriver_ws/src/tmr_ros2/json/feedback_pose.json"
    move_file = "/home/hudenxiao/tmdriver_ws/src/tmr_ros2/json/move.json"
    prompt_file = "/home/hudenxiao/tmdriver_ws/LLMtest/prompt.txt"   # <-- 外部 prompt 檔案

    #prompt
    if not os.path.exists(prompt_file):
        print(f"找不到 {prompt_file}，請先建立 system prompt 檔案")
        return
    with open(prompt_file, "r", encoding="utf-8") as f:
        system_prompt = f.read()

    messages = [{
        "role": "system",
        "content": system_prompt
    }]

    while True:
        user_input = input("我：")
        if user_input.lower() in ["exit", "quit"]:
            break

        messages.append({"role": "user", "content": user_input})
        try:
            reply = ask_openrouter(messages)
            print(f"機器人：{reply}")
            messages.append({"role": "assistant", "content": reply})

            json_data = json.loads(reply)

            # --- swap 功能 ---
            if "swap" in json_data:
                k1, k2 = json_data["swap"]
                pos1 = get_keycap_position(k1, keycaps_file)
                pos2 = get_keycap_position(k2, keycaps_file)
                if not pos1 or not pos2:
                    print(f"找不到鍵帽 {k1} 或 {k2} 的座標，請檢查 {keycaps_file}")
                    continue

                feedback_pose = get_feedback_pose(feedback_file)
                if feedback_pose:
                    rx, ry, rz = feedback_pose["tool_pose"]["rx"], feedback_pose["tool_pose"]["ry"], feedback_pose["tool_pose"]["rz"]
                else:
                    rx, ry, rz = -173.0, -12.27, 129.31

                # 閒置區
                idle_pos = {"x": 500.0, "y": -60.0, "z": 400.0}

                # 交換流程
                swap_instructions = [
                    {"mode": "absolute", "x": pos1["x"], "y": pos1["y"], "z": pos1["z"], "rx": rx, "ry": ry, "rz": rz, "action": "pick"},
                    {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"], "rx": rx, "ry": ry, "rz": rz, "action": "place"},
                    {"mode": "absolute", "x": pos2["x"], "y": pos2["y"], "z": pos2["z"], "rx": rx, "ry": ry, "rz": rz, "action": "pick"},
                    {"mode": "absolute", "x": pos1["x"], "y": pos1["y"], "z": pos1["z"], "rx": rx, "ry": ry, "rz": rz, "action": "place"},
                    {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"], "rx": rx, "ry": ry, "rz": rz, "action": "pick"},
                    {"mode": "absolute", "x": pos2["x"], "y": pos2["y"], "z": pos2["z"], "rx": rx, "ry": ry, "rz": rz, "action": "place"}
                ]

                json_data = {"swap_sequence": swap_instructions}
                print(f"執行鍵帽 {k1} 與 {k2} 的交換動作（使用閒置區暫存）")

            # --- keycap 功能 ---
            elif "keycap" in json_data:
                key_name = json_data["keycap"]
                pos = get_keycap_position(key_name, keycaps_file)
                if pos:
                    feedback_pose = get_feedback_pose(feedback_file)
                    if feedback_pose:
                        rx, ry, rz = feedback_pose["tool_pose"]["rx"], feedback_pose["tool_pose"]["ry"], feedback_pose["tool_pose"]["rz"]
                    else:
                        rx, ry, rz = 180.0, 0, 133

                    json_data = {
                        "mode": "absolute",
                        "x": pos["x"],
                        "y": pos["y"],
                        "z": pos["z"],
                        "rx": rx,
                        "ry": ry,
                        "rz": rz,
                        "action": "place"
                    }
                    print(f"移動到鍵帽 {key_name}")
                else:
                    print(f"找不到鍵帽 {key_name} 的座標")
                    continue

            # --- 相對 / 絕對移動 ---
            elif "mode" in json_data:
                feedback_pose = get_feedback_pose(feedback_file)
                if not feedback_pose:
                    print("找不到手臂當前座標，無法執行")
                    continue

                abs_pose = [
                    feedback_pose["tool_pose"]["x"],
                    feedback_pose["tool_pose"]["y"],
                    feedback_pose["tool_pose"]["z"],
                    feedback_pose["tool_pose"]["rx"],
                    feedback_pose["tool_pose"]["ry"],
                    feedback_pose["tool_pose"]["rz"]
                ]

                if json_data["mode"] == "relative":
                    dx = json_data.get("x", 0.0)
                    dy = json_data.get("y", 0.0)
                    dz = json_data.get("z", 0.0)
                    drx = json_data.get("rx", 0.0)
                    dry = json_data.get("ry", 0.0)
                    drz = json_data.get("rz", 0.0)

                    abs_pose[0] += dx
                    abs_pose[1] += dy
                    abs_pose[2] += dz
                    abs_pose[3] += drx
                    abs_pose[4] += dry
                    abs_pose[5] += drz

                    json_data = {
                        "mode": "absolute",
                        "x": abs_pose[0],
                        "y": abs_pose[1],
                        "z": abs_pose[2],
                        "rx": abs_pose[3],
                        "ry": abs_pose[4],
                        "rz": abs_pose[5]
                    }

                    print(f"已轉換為絕對座標：{json_data}")

            # 儲存 JSON
            with open(move_file, 'w', encoding="utf-8") as f:
                json.dump(json_data, f, indent=2)
            print(f"已將指令儲存至 {move_file}")

            # 執行 ROS2
            subprocess.run(["ros2", "run", "arm_movement", "send_movement"])

        except Exception as e:
            print(f"發生錯誤：{e}")

if __name__ == "__main__":
    main()
