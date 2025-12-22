
#!/usr/bin/env python3
import os
import json
import requests
import subprocess
import time

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
    "OFFSET_Z": {"x": 0.0, "y": 0.0, "z": -160.0},
    "OFFSET_DEFAULT": {"x": 0.0, "y": 0.0, "z": 0.0}
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
    subprocess.run(["ros2", "run", "arm_movement", "initial_position"])
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

        keycaps_file = "/home/hsiu/tmrdriver_ws/resource/json/keyboard_layout.json"
        feedback_file = "/home/hsiu/tmrdriver_ws/resource/json//feedback_pose.json"
        move_file = "/home/hsiu/tmrdriver_ws/resource/json//move.json"
        last_move_file = "/home/hsiu/tmrdriver_ws/resource/json//last_move.json"
        prompt_file = "/home/hsiu/tmrdriver_ws/LLMtest/prompt.txt"

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

                    idle_pos = {"x": 409, "y": -30.663, "z": 440.0}
                    swap_instructions = [
                        # 拔起第一個鍵帽
                        {"mode": "absolute", "x": pos1["x"]+3, "y": pos1["y"]-20, "z": pos1["z"], "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": pos1["x"]+3, "y": pos1["y"]-20, "z": pos1["z"] - 160, "rx": rx, "ry": ry, "rz": rz},
                        {"action": "suck"},
                        {"mode": "absolute", "x": pos1["x"]+3, "y": pos1["y"]-20, "z": 440, "rx": rx, "ry": ry, "rz": rz},

                        # 移動到暫放區並回到原點
                        {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"], "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"] - 155, "rx": rx, "ry": ry, "rz": rz},
                        time.sleep(0.7),
                        {"action": "release"},
                        {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": 440, "rx": rx, "ry": ry, "rz": rz},

                        # 拔起第二個鍵帽回到原點
                        {"mode": "absolute", "x": pos2["x"]+3, "y": pos2["y"]-20, "z": pos2["z"], "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": pos2["x"]+3, "y": pos2["y"]-20, "z": pos2["z"]-160, "rx": rx, "ry": ry, "rz": rz},
                        {"action": "suck"},
                        {"mode": "absolute", "x": pos2["x"]+3, "y": pos2["y"]-20, "z": 440, "rx": rx, "ry": ry, "rz": rz},

                        # 放到第一個鍵帽原本的空位
                        {"mode": "absolute", "x": pos1["x"]+3, "y": pos1["y"]-20, "z": pos1["z"], "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": pos1["x"]+3, "y": pos1["y"]-20, "z": pos1["z"] - 160, "rx": rx, "ry": ry, "rz": rz},
                        {"action": "release"},
                        {"mode": "absolute", "x": pos1["x"]+3, "y": pos1["y"]-20, "z": 440, "rx": rx, "ry": ry, "rz": rz},

                        # 在暫放區把第一個鍵帽吸起
                        {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"], "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"] - 155, "rx": rx, "ry": ry, "rz": rz},
                        {"action": "suck"},
                        {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"], "rx": rx, "ry": ry, "rz": rz},
                        

                        # 放到第二個鍵帽原本的空位，完成互換並回到原點
                        {"mode": "absolute", "x": pos2["x"]+2, "y": pos2["y"]-20, "z": pos2["z"], "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": pos2["x"]+2, "y": pos2["y"]-20, "z": pos2["z"]-160, "rx": rx, "ry": ry, "rz": rz},
                        {"action": "release"},
                        {"mode": "absolute", "x": 433.72, "y": 78.29, "z": 440, "rx": rx, "ry": ry, "rz": rz}
                    ]
                    json_data = {"swap_sequence": swap_instructions}

                # --- action 功能（吸盤控制） ---
                elif "action" in json_data:
                    action = json_data["action"]
                    if action not in ["suck", "release", "pick", "place"]:
                        print(f"不支援的動作：{action}")
                        continue

                # 決定要 publish 的 Bool 值
                    if action in ["suck", "pick"]:
                        bool_value = True
                    elif action in ["release", "place"]:
                        bool_value = False

                # 呼叫 topic
                    cmd = [
                    "ros2", "topic", "pub", "--once",
                    "/sucker_command", "std_msgs/msg/Bool",
                    f'{{data: {str(bool_value).lower()}}}'
                    ]

                    print(f"執行吸盤指令: {action} → publish {bool_value}")
                    try:
                        subprocess.run(cmd, check=True)
                    except subprocess.CalledProcessError as e:
                        print(f"吸盤指令執行失敗：{e}")

                    continue


                # --- keycap 功能 ---
                elif "keycap" in json_data:
                    key_name = json_data["keycap"]

                    # 讀取鍵帽座標（安全高度）
                    idle_pos = {"x": 409.0, "y": -30.663, "z": 440.0}
                    pos_pick = get_keycap_position(key_name, keycaps_file)
                    if not pos_pick:
                        print(f"找不到鍵帽 {key_name} 的座標")
                        continue

                    # 讀取鍵盤目標座標（安全高度）
                    keyboard_layout_file = "/home/hsiu/tmrdriver_ws/resource/json//keyboard_layout.json"
                    if not os.path.exists(keyboard_layout_file):
                        print(f"找不到 {keyboard_layout_file}")
                        continue

                    with open(keyboard_layout_file, "r", encoding="utf-8") as f:
                        keyboard_layout = json.load(f)

                    if key_name not in keyboard_layout:
                        print(f"keyboard_layout 中找不到 {key_name} 的座標")
                        continue

                    pos_place = keyboard_layout[key_name]

                    # 吸盤高度偏移
                    pick_offset = -160     # 從安全高度向下吸鍵帽（可調）

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
                    # 1. 移動到欲組裝的鍵帽安全高度
                    sequence.append({
                        "mode": "absolute",
                        "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"],
                        "rx": rx, "ry": ry, "rz": rz
                    })

                    # 2. 向下到鍵帽上方準備吸附
                    sequence.append({
                        "mode": "absolute",
                        "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"] + pick_offset,
                        "rx": rx, "ry": ry, "rz": rz
                    })

                    # 3. 開啟吸盤
                    sequence.append({   
                        "action": "suck"
                    })

                    # 4. 回到安全高度
                    sequence.append({
                        "mode": "absolute",
                        "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"],
                        "rx": rx, "ry": ry, "rz": rz
                    })

                    

                    #------------------ 移動到鍵盤放置點 ------------------
                    sequence.append({
                        "mode": "absolute",
                        "x": pos_place["x"]+3, "y": pos_place["y"]-20, "z": pos_place["z"],
                        "rx": rx, "ry": ry, "rz": rz
                    })

                    # 往下移動到鍵盤正上方
                    sequence.append({
                        "mode": "absolute",
                        "x": pos_place["x"]+3, "y": pos_place["y"]-20, "z": pos_place["z"] - 160,
                        "rx": rx, "ry": ry, "rz": rz
                    })

                    sequence.append({   
                        "action": "release"
                    })

                    # 4. 回到安全高度
                    sequence.append({
                        "mode": "absolute",
                        "x": 433.72, "y": 78.29, "z": 440.0,
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
                # --- spell 功能 (依照新邏輯：先到上方->修正下降->吸取->安全高度->放置) ---
                elif "spell" in json_data:
                    action_list = json_data["spell"]
                    
                    place_base = {"x": 407.8, "y": -30.663, "z": 440.0}
                    y_step = 19.0  # 安裝位置的 Y 間距
                    spell_sequence = []
                    
                    # 檢查重複字母
                    chars = [item["char"] for item in action_list]
                    if len(chars) != len(set(chars)):
                        print("包含重複字母，請輸入別的單字")
                        continue

                    # 取得目前姿態，若無則用預設
                    feedback_pose = get_feedback_pose(feedback_file)
                    if feedback_pose:
                        rx = feedback_pose["tool_pose"]["rx"]
                        ry = feedback_pose["tool_pose"]["ry"]
                        rz = feedback_pose["tool_pose"]["rz"]
                    else:
                        rx, ry, rz = 180.0, 0.0, 133.0

                    for idx, step in enumerate(action_list):
                        char_target = step["char"].lower()
                        offset_key = step["correction"]
                        
                        pos = get_keycap_position(char_target, keycaps_file)
                        if not pos:
                            print(f"找不到鍵帽 {char_target} 的座標")
                            continue

                        # 取得對應行數的修正值
                        correction = ROW_OFFSETS.get(offset_key, {"x": 0.0, "y": 0.0, "z": 0.0})

                        # 基礎座標 (無修正)
                        base_x = pos["x"] + 3
                        base_y = pos["y"] - 20
                        base_z = pos["z"]

                        # 修正座標 (有 Offset)
                        corrected_x = base_x + correction["x"]
                        corrected_y = base_y + correction["y"]
                        # 下降高度：Z 軸向下修正
                        pick_z = base_z + correction["z"]

                        # 計算放置座標 (每次 Y 增加 19)
                        place_x = place_base["x"]
                        place_y = place_base["y"] + idx * y_step
                        place_z = place_base["z"]

                        # --- 1. 移動到要組裝的鍵帽上面 (先不要添加修正項) ---
                        spell_sequence.append({
                            "mode": "absolute",
                            "x": base_x, "y": base_y, "z": base_z,
                            "rx": rx, "ry": ry, "rz": rz
                        })
                        
                        # --- 2. 向下移動並修正 offset ---
                        spell_sequence.append({
                            "mode": "absolute",
                            "x": corrected_x, "y": corrected_y, "z": pick_z,
                            "rx": rx, "ry": ry, "rz": rz
                        })
                        
                        # --- 3. 吸取鍵帽 ---
                        spell_sequence.append({"action": "suck"})

                        # --- 4. 移動到安全高度 ---
                        # (這裡使用 corrected_x/y 垂直上升，避免撞到隔壁)
                        spell_sequence.append({
                            "mode": "absolute",
                            "x": corrected_x, "y": corrected_y, "z": 440,
                            "rx": rx, "ry": ry, "rz": rz
                        })

                        # --- 5. 移動到指定安裝位置上方 ---
                        spell_sequence.append({
                            "mode": "absolute",
                            "x": place_x, "y": place_y, "z": place_z,
                            "rx": rx, "ry": ry, "rz": rz
                        })

                        # --- 6. 下降到放置高度 ---
                        spell_sequence.append({
                            "mode": "absolute",
                            "x": place_x, "y": place_y, "z": pick_z,
                            "rx": rx, "ry": ry, "rz": rz
                        })

                        # --- 7. 放下鍵帽 ---
                        spell_sequence.append({"action": "release"})

                        # --- 8. 上升回安全高度 ---
                        spell_sequence.append({
                            "mode": "absolute",
                            "x": place_x, "y": place_y, "z": 440,
                            "rx": rx, "ry": ry, "rz": rz
                        })

                        # --- 9. 回原點 ---
                        spell_sequence.append({
                            "mode": "absolute",
                            "x": 433.72, "y": 78.29, "z": 440,
                            "rx": rx, "ry": ry, "rz": rz
                        })

                    json_data = {"spell_sequence": spell_sequence}
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
                            "x": pos["x"] +5,
                            "y": pos["y"]-20,
                            "z": pos["z"],
                            "rx": rx,
                            "ry": ry,
                            "rz": rz,
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

                # 呼叫 C++ 節點執行（確保 send_movement 處理 sequence/action）
                subprocess.run(["ros2", "run", "arm_movement", "send_movement"])
                
                

            except Exception as e:
                print(f"發生錯誤：{e}")

    finally:
        print("關閉 ROS2 系統 ...")
        main_process.terminate()
        time.sleep(1)
        print("ROS2 已停止。")


if __name__ == "__main__":
    main()