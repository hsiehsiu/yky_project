#!/usr/bin/env python3
import os
import json
import requests
import subprocess
import time

# ------------------ OpenRouter API ------------------
API_KEY = "APIKEY"
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
        return '{"error": "API 請求失敗"}'
    return response.json()["choices"][0]["message"]["content"]

# ------------------ 啟動 ROS launch ------------------
def start_ros_launch():
    """
    啟動main.launch.py
    """
    print("啟動中...")

    main_process = subprocess.Popen(
        ["ros2", "launch", "bringup", "main.launch.py"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    time.sleep(2)

    print("ROS2 系統已啟動完成。")
    return main_process

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
    print("初始化 ROS2 系統 ...")
    main_process = start_ros_launch()

    try:
        print("聊天啟動，輸入 'q' 離開。", flush=True)

        keycaps_file = "/home/hsiu/tmrdriver_ws/resource/json/keycap_coordinate.json"
        feedback_file = "/home/hsiu/tmrdriver_ws/resource/json/feedback_pose.json"
        move_file = "/home/hsiu/tmrdriver_ws/resource/json/move.json"
        last_move_file = "/home/hsiu/tmrdriver_ws/resource/json/last_move.json"
        prompt_file = "/home/hsiu/tmrdriver_ws/resource/json/prompt.txt"

        if not os.path.exists(prompt_file):
            print(f"找不到 {prompt_file}，請先建立 system prompt 檔案")
            return
        with open(prompt_file, "r", encoding="utf-8") as f:
            system_prompt = f.read()

        messages = [{"role": "system", "content": system_prompt}]

        while True:
            user_input = input("我：")
            if user_input.lower() in ["exit", "quit","q"]:
                break

            messages.append({"role": "user", "content": user_input})
            try:
                reply = ask_openrouter(messages)
                print(f"機器人：{reply}")
                messages.append({"role": "assistant", "content": reply})

                try:
                    json_data = json.loads(reply)
                except json.JSONDecodeError:
                    print("無法解析，請重新輸入。")
                    continue

                if "error" in json_data:
                    print("無法理解您的指令，請重新輸入！")
                    continue

                # 回到上次位置
                if "return_last" in json_data and json_data["return_last"] is True:
                    if not os.path.exists(last_move_file):
                        print("目前沒有上一次的位置記錄。")
                        continue
                    with open(last_move_file, "r", encoding="utf-8") as f:
                        json_data = json.load(f)
                    print("回到上一次的座標。")

                # --- swap 功能 ---
                elif "swap" in json_data:
                    k1, k2 = json_data["swap"]
                    pos1 = get_keycap_position(k1, keycaps_file)
                    pos2 = get_keycap_position(k2, keycaps_file)
                    if not pos1 or not pos2:
                        print(f"找不到鍵帽 {k1} 或 {k2} 的座標")
                        continue

                    feedback_pose = get_feedback_pose(feedback_file)
                    if feedback_pose:
                        rx, ry, rz = feedback_pose["tool_pose"]["rx"], feedback_pose["tool_pose"]["ry"], feedback_pose["tool_pose"]["rz"]
                    else:
                        rx, ry, rz = -173.0, -12.27, 129.31

                    idle_pos = {"x": 500.0, "y": -60.0, "z": 400.0}
                    swap_instructions = [
                        {"mode": "absolute", "x": pos1["x"], "y": pos1["y"], "z": pos1["z"], "rx": rx, "ry": ry, "rz": rz, "action": "pick"},
                        {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"], "rx": rx, "ry": ry, "rz": rz, "action": "place"},
                        {"mode": "absolute", "x": pos2["x"], "y": pos2["y"], "z": pos2["z"], "rx": rx, "ry": ry, "rz": rz, "action": "pick"},
                        {"mode": "absolute", "x": pos1["x"], "y": pos1["y"], "z": pos1["z"], "rx": rx, "ry": ry, "rz": rz, "action": "place"},
                        {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"], "rx": rx, "ry": ry, "rz": rz, "action": "pick"},
                        {"mode": "absolute", "x": pos2["x"], "y": pos2["y"], "z": pos2["z"], "rx": rx, "ry": ry, "rz": rz, "action": "place"}
                    ]
                    json_data = {"swap_sequence": swap_instructions}

                # --- action 功能（吸盤控制） ---
                elif "action" in json_data:
                    action = json_data["action"]

                    if action not in ["suck", "release", "pick", "place"]:
                        print(f"不支援的動作：{action}")
                        continue                   

                # --- keycap 功能（鍵帽組裝流程 with force feedback） ---
                elif "keycap" in json_data:
                    key_name = json_data["keycap"]

                    # 讀取鍵帽座標（安全高度）
                    pos_pick = get_keycap_position(key_name, keycaps_file)
                    if not pos_pick:
                        print(f"找不到鍵帽 {key_name} 的座標")
                        continue

                    # 讀取鍵盤目標座標（安全高度）
                    keyboard_layout_file = "/home/hsiu/tmrdriver_ws/resource/json/keyboard_layout.json"
                    if not os.path.exists(keyboard_layout_file):
                        print(f"找不到 {keyboard_layout_file}")
                        continue

                    with open(keyboard_layout_file, "r", encoding="utf-8") as f:
                        keyboard_layout = json.load(f)

                    if key_name not in keyboard_layout:
                        print(f"keyboard_layout 中找不到 {key_name} 的座標")
                        continue

                    pos_place = keyboard_layout[key_name]

                    # 吸盤高度與插入高度偏移
                    pick_offset = -20      # 從安全高度向下 20mm 去吸鍵帽
                    place_offset_start = -10  # 插入前靠近鍵盤 10mm
                    press_step = 1.0         # 每次下壓 1mm
                    force_threshold = 2.5    # 觸發判定力值 (可調)

                    # 姿態
                    feedback_pose = get_feedback_pose(feedback_file)
                    if feedback_pose:
                        rx = feedback_pose["tool_pose"]["rx"]
                        ry = feedback_pose["tool_pose"]["ry"]
                        rz = feedback_pose["tool_pose"]["rz"]
                    else:
                        rx, ry, rz = 180.0, 0.0, 133.0

                    sequence = []

                    # ------------------ 拾取鍵帽 ------------------
                    # 1. 移動到鍵帽安全高度
                    sequence.append({
                        "mode": "absolute",
                        "x": pos_pick["x"], "y": pos_pick["y"], "z": pos_pick["z"],
                        "rx": rx, "ry": ry, "rz": rz
                    })

                    # 2. 向下到鍵帽上方準備吸附
                    sequence.append({
                        "mode": "absolute",
                        "x": pos_pick["x"], "y": pos_pick["y"], "z": pos_pick["z"] + pick_offset,
                        "rx": rx, "ry": ry, "rz": rz
                    })

                    # 3. 開啟吸盤
                    sequence.append({   
                        "action": "suck"
                    })

                    # 4. 回到安全高度
                    sequence.append({
                        "mode": "absolute",
                        "x": pos_pick["x"], "y": pos_pick["y"], "z": 480.0,
                        "rx": rx, "ry": ry, "rz": rz
                    })

                    # ------------------ 移動到鍵盤放置點 ------------------
                    sequence.append({
                        "mode": "absolute",
                        "x": pos_place["x"], "y": pos_place["y"], "z": pos_place["z"],
                        "rx": rx, "ry": ry, "rz": rz
                    })

                    # 接近鍵盤位置
                    z_current = pos_place["z"] + place_offset_start
                    sequence.append({
                        "mode": "absolute",
                        "x": pos_place["x"], "y": pos_place["y"], "z": z_current,
                        "rx": rx, "ry": ry, "rz": rz
                    })

                    # ------------------ 力回饋插入 ------------------
                    force_file = "/home/hsiu/tmrdriver_ws/resource/json/force.json"

                    while True:
                        # 每次壓 1mm
                        z_next = z_current - press_step
                        z_current = z_next

                        sequence.append({
                            "mode": "absolute",
                            "x": pos_place["x"], "y": pos_place["y"], "z": z_current,
                            "rx": rx, "ry": ry, "rz": rz
                        })

                        # 讀取 force.json
                        if os.path.exists(force_file):
                            try:
                                with open(force_file, "r", encoding="utf-8") as f:
                                    force_data = json.load(f)
                                # 嘗試支援不同 key 名（fz 或 f_z）
                                fz = force_data.get("z", force_data.get("f_z", 0))
                            except Exception as e:
                                print(f"讀取 force.json 發生錯誤: {e}")
                                fz = 0
                        else:
                            fz = 0

                        # 觸發插入完成
                        if fz > force_threshold:
                            print(f"> 力回饋達閾值 (fz = {fz}N)，鍵帽插入成功")
                            break

                        # 防止一直往下撞（安全檢查）
                        if z_current - 125 < pos_place["z"] - 125 - 100 - 30: #125：吸盤長度，100：吸盤與鍵盤表面距離，30：下壓程度
                            print(" 插入深度過大，中止！")
                            break

                        # 小等待，讓 force.json 有時間更新（可依實際情況調整）
                        time.sleep(0.1)

                    # ------------------ 回到安全高度 ------------------
                    sequence.append({
                        "mode": "absolute",
                        "x": pos_place["x"], "y": pos_place["y"], "z": 480.0,
                        "rx": rx, "ry": ry, "rz": rz
                    })

                    json_data = {"assembly_sequence": sequence}

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
                        abs_pose[0] += json_data.get("x", 0.0)
                        abs_pose[1] += json_data.get("y", 0.0)
                        abs_pose[2] += json_data.get("z", 0.0)
                        abs_pose[3] += json_data.get("rx", 0.0)
                        abs_pose[4] += json_data.get("ry", 0.0)
                        abs_pose[5] += json_data.get("rz", 0.0)

                        json_data = {
                            "mode": "absolute",
                            "x": abs_pose[0],
                            "y": abs_pose[1],
                            "z": abs_pose[2],
                            "rx": abs_pose[3],
                            "ry": abs_pose[4],
                            "rz": abs_pose[5]
                        }
                # --- movetokeycap 功能 ---
                elif "movetokeycap" in json_data:
                    key_name = json_data["movetokeycap"]
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
                    else:
                        print(f"找不到鍵帽 {key_name} 的座標")
                        continue    

                else:
                    print("模型輸出不包含可執行的指令欄位。")
                    continue

                # 儲存上一次位置
                with open(last_move_file, 'w', encoding="utf-8") as f:
                    json.dump(json_data, f, indent=2)

                # 寫入 move.json（send_movement 會讀取並執行）
                with open(move_file, 'w', encoding="utf-8") as f:
                    json.dump(json_data, f, indent=2)

                # 呼叫 C++ 節點執行（確保 send_movement 能處理 assembly_sequence/action）
                subprocess.run(["ros2", "run", "arm_movement", "send_movement"])
                subprocess.run(["ros2", "run", "arduino", "sucker"])

            except Exception as e:
                print(f"發生錯誤：{e}")

    finally:
        print("關閉 ROS2 系統 ...")
        main_process.terminate()
        time.sleep(1)
        print("ROS2 已停止。")


if __name__ == "__main__":
    main()
