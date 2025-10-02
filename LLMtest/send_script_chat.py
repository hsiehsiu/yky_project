import json
import subprocess
import requests
import os
import time

# ------------------- OpenRouter API -------------------
API_KEY = "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"
URL = "https://openrouter.ai/api/v1/chat/completions"
HEADERS = {
    "Authorization": f"Bearer {API_KEY}",
    "Content-Type": "application/json"
}

def ask_openrouter(messages, model="openai/gpt-3.5-turbo"):
    payload = {"model": model, "messages": messages, "temperature": 0.7}
    r = requests.post(URL, headers=HEADERS, json=payload)
    if r.status_code != 200:
        print(f"錯誤 {r.status_code}: {r.text}")
        return None
    return r.json()["choices"][0]["message"]["content"]

def safe_print(text):
    try:
        print(text)
    except UnicodeEncodeError:
        print(text.encode("utf-8", errors="replace").decode("utf-8", errors="replace"))

# ------------------- JSON 路徑 -------------------
FEEDBACK_JSON = "/home/hudenxiao/tmdriver_ws/src/tmr_ros2/json/feedback_pose.json"
MOVE_JSON = "/home/hudenxiao/tmdriver_ws/src/tmr_ros2/json/move.json"

# ------------------- System Prompt -------------------
SYSTEM_PROMPT = {
    "role": "system",
    "content": (
        "你是一位用中文回答的ROS2手臂控制智慧助理，會根據使用者輸入的自然語言，輸出一段 JSON 格式的手臂移動指令。\n"
        "JSON 必須包含以下欄位：\n"
        "{ \"mode\": \"relative\" 或 \"absolute\", \"x\": 0.0, \"y\": 0.0, \"z\": 0.0, \"rx\": 0.0, \"ry\": 0.0, \"rz\": 0.0 }\n"
        "請直接回傳 JSON，不需要多餘文字說明。\n\n"
        "範例:\n"
        "1. 輸入：請讓機器手臂沿 x 軸移動 20 毫米\n"
        "   回傳：\n"
        "{\n  \"mode\": \"relative\",\n  \"x\": 20.0,\n  \"y\": 0.0,\n  \"z\": 0.0,\n  \"rx\": 0.0,\n  \"ry\": 0.0,\n  \"rz\": 0.0\n}\n\n"
        "2. 輸入：請讓機器手臂沿 y 軸移動 -10 毫米\n"
        "   回傳：\n"
        "{\n  \"mode\": \"relative\",\n  \"x\": 0.0,\n  \"y\": -10.0,\n  \"z\": 0.0,\n  \"rx\": 0.0,\n  \"ry\": 0.0,\n  \"rz\": 0.0\n}\n\n"
        "3. 輸入：請將手臂移動到絕對座標 (450, 100, 300)\n"
        "   回傳：\n"
        "{\n  \"mode\": \"absolute\",\n  \"x\": 450.0,\n  \"y\": 100.0,\n  \"z\": 300.0,\n  \"rx\": 0.0,\n  \"ry\": 0.0,\n  \"rz\": 0.0\n}"
    )
}

def main():
    print("聊天啟動，輸入 'exit' 離開")
    messages = [SYSTEM_PROMPT]

    try:
        while True:
            user_input = input("我：")
            if user_input.lower() in ["exit", "quit"]:
                break
            elif user_input.strip() == "":
                continue

            # 讀取最新手臂座標 JSON
            try:
                with open(FEEDBACK_JSON, "r") as f:
                    feedback = json.load(f)
                    abs_pose = [
                        feedback["tool_pose"]["x"],
                        feedback["tool_pose"]["y"],
                        feedback["tool_pose"]["z"],
                        feedback["tool_pose"]["rx"],
                        feedback["tool_pose"]["ry"],
                        feedback["tool_pose"]["rz"]
                    ]
            except Exception as e:
                print(f"讀取 feedback JSON 失敗: {e}")
                continue

            messages.append({"role": "user", "content": user_input})

            # 呼叫 LLM
            reply = ask_openrouter(messages)
            if reply is None:
                continue
            safe_print(f"機器人：{reply}")
            messages.append({"role": "assistant", "content": reply})

            # 解析 JSON (相對座標)
            rel_json = json.loads(reply)
            dx = rel_json.get("x", 0.0)
            dy = rel_json.get("y", 0.0)
            dz = rel_json.get("z", 0.0)
            drx = rel_json.get("rx", 0.0)
            dry = rel_json.get("ry", 0.0)
            drz = rel_json.get("rz", 0.0)

            # 累加成絕對座標
            abs_pose[0] += dx
            abs_pose[1] += dy
            abs_pose[2] += dz
            abs_pose[3] += drx
            abs_pose[4] += dry
            abs_pose[5] += drz

            abs_json = {
                "mode": "absolute",
                "x": abs_pose[0],
                "y": abs_pose[1],
                "z": abs_pose[2],
                "rx": abs_pose[3],
                "ry": abs_pose[4],
                "rz": abs_pose[5]
            }

            # 寫入 move.json
            with open(MOVE_JSON, "w") as f:
                json.dump(abs_json, f, indent=2)
            print(f"絕對座標 JSON 已寫入 {MOVE_JSON}")

            # 執行 ROS2 控制程式
            subprocess.run(["ros2", "run", "demo", "demo_send_script"])

    except KeyboardInterrupt:
        print("程式中止")

if __name__ == "__main__":
    main()
