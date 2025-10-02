import os
import json
import base64
import requests
import subprocess
import time

# ------------------ OpenRouter API ------------------
API_KEY = "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"
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

def ask_vlm_with_image(image_path, user_prompt, model="qwen/qwen-vl-plus"):
    with open(image_path, "rb") as f:
        base64_img = base64.b64encode(f.read()).decode("utf-8")
    messages = [{
        "role": "user",
        "content": [
            {"type": "text", "text": user_prompt},
            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_img}"}}
        ]
    }]
    payload = {"model": model, "messages": messages, "temperature": 0.7}
    response = requests.post(URL, headers=headers, json=payload)
    if response.status_code != 200:
        print(f"錯誤 {response.status_code}: {response.text}")
        return "[錯誤] 圖像任務處理失敗"
    return response.json()["choices"][0]["message"]["content"]

def safe_print(text):
    try:
        print(text)
    except UnicodeEncodeError:
        print(text.encode("utf-8", errors="replace").decode("utf-8", errors="replace"))

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
    print("聊天啟動，輸入 'exit' 離開，輸入 'pic' 進行圖片辨識。", flush=True)

    # JSON 檔案路徑（請改成你自己的路徑）
    keycaps_file = "/home/hudenxiao/tmdriver_ws/LLMtest/expected.json"
    feedback_file = "/home/hudenxiao/tmdriver_ws/src/tmr_ros2/json/feedback_pose.json"
    move_file = "/home/hudenxiao/tmdriver_ws/src/tmr_ros2/json/move.json"

    messages = [{
        "role": "system",
        "content": (
            "你是一位用中文回答的ROS2手臂控制智慧助理。"
            "你的任務是根據使用者輸入，輸出一個 JSON 格式的指令，並且只輸出 JSON，不要多餘的文字。\n\n"
            "JSON 必須符合以下格式：\n\n"
            "1. 相對移動:\n"
            "{ \"mode\": \"relative\", \"x\": 數值, \"y\": 數值, \"z\": 數值, \"rx\": 數值, \"ry\": 數值, \"rz\": 數值 }\n\n"
            "2. 絕對座標:\n"
            "{ \"mode\": \"absolute\", \"x\": 數值, \"y\": 數值, \"z\": 數值, \"rx\": 數值, \"ry\": 數值, \"rz\": 數值 }\n\n"
            "3. 指定鍵帽:\n"
            "{ \"keycap\": \"鍵帽名稱\" }\n\n"
            "### 範例：\n"
            "輸入：請讓機器手臂向 x 軸方向移動 20 毫米\n"
            "輸出：{ \"mode\": \"relative\", \"x\": 20.0, \"y\": 0.0, \"z\": 0.0, \"rx\": 0.0, \"ry\": 0.0, \"rz\": 0.0 }\n\n"
            "輸入：請讓機器手臂移動到 Enter 鍵\n"
            "輸出：{ \"keycap\": \"Enter\" }"
        )
    }]

    while True:
        user_input = input("我：")

        if user_input.lower() in ["exit", "quit"]:
            break
        elif user_input == "pic":
            image_path = input("請輸入圖片路徑（如：keyboard.jpg）：")
            if not os.path.exists(image_path):
                print("圖片路徑錯誤")
                continue
            user_prompt = input("請輸入任務描述：")
            result = ask_vlm_with_image(image_path, user_prompt)
            safe_print(f"機器人（圖片分析）：{result}")
            continue

        messages.append({"role": "user", "content": user_input})
        try:
            reply = ask_openrouter(messages)
            safe_print(f"機器人：{reply}")
            messages.append({"role": "assistant", "content": reply})

            json_data = json.loads(reply)

            # ------------------ 處理鍵帽座標 ------------------
            if "keycap" in json_data:
                key_name = json_data["keycap"]
                pos = get_keycap_position(key_name, keycaps_file)
                if pos:
                    feedback_pose = get_feedback_pose(feedback_file)
                    if feedback_pose is None:
                        print("無法讀取手臂狀態，使用預設旋轉角度")
                        rx, ry, rz = -173.0, -12.27, 129.31
                    else:
                        rx, ry, rz = feedback_pose["rx"], feedback_pose["ry"], feedback_pose["rz"]

                    json_data = {
                        "mode": "absolute",
                        "x": pos["x"],
                        "y": pos["y"],
                        "z": pos["z"],
                        "rx": rx,
                        "ry": ry,
                        "rz": rz
                    }
                    print(f"移動到鍵帽 {key_name}，保持旋轉角度 {[rx, ry, rz]}")
                else:
                    print(f"找不到鍵帽 {key_name} 的座標，請檢查 {keycaps_file}")
                    continue

            # 儲存 JSON
            with open(move_file, 'w', encoding="utf-8") as f:
                json.dump(json_data, f, indent=2)
            print(f"已將指令儲存至 {move_file}")

            # 執行 ROS2 
            subprocess.run(["ros2", "run", "demo", "demo_send_script"])

        except Exception as e:
            print(f"發生錯誤：{e}")

if __name__ == "__main__":
    main()
