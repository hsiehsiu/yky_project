#!/usr/bin/env python3
import os
import json
import subprocess
import time
import ollama  # 改用 ollama 函式庫

# ------------------ 本地 LLM 設定 ------------------
# 根據您的顯卡 (RTX 5070 8GB) 與 ROS2 共存需求，使用 Llama 3.2 (3B)
OLLAMA_MODEL = "llama3.2"
offsetx = 3
offsety = -20
offsetx_q = 0
offsety_q = 0
offsetx_a = 0
offsety_a = 0
offsetx_z = 0
offsety_z = 0
def ask_ollama(messages, model=OLLAMA_MODEL):
    try:
        response = ollama.chat(
            model=model,
            messages=messages,
            options={
                'temperature': 0.7,  # 維持原本的創造力參數
            },
            keep_alive=0  # 重要：回答完立刻釋放 VRAM，避免卡到 ROS2
        )
        return response['message']['content']
    except Exception as e:
        print(f"Ollama 呼叫錯誤: {e}")
        # 回傳一個錯誤的 JSON 格式讓主程式能捕捉並重試
        return '{"error": "本地模型執行失敗"}'

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
        print(f"聊天啟動 (本地模型: {OLLAMA_MODEL})，輸入 'q' 離開。", flush=True)

        # 您的路徑設定維持不變
        keycaps_file = "/home/hsiu/tmrdriver_ws/resource/json/keycap_coordinate.json"
        feedback_file = "/home/hsiu/tmrdriver_ws/resource/json/feedback_pose.json"
        move_file = "/home/hsiu/tmrdriver_ws/resource/json/move.json"
        last_move_file = "/home/hsiu/tmrdriver_ws/resource/json/last_move.json"
        prompt_file = "/home/hsiu/tmrdriver_ws/LLMtest/prompt.txt"

        if not os.path.exists(prompt_file):
            print(f"找不到 {prompt_file}，請先建立 system prompt 檔案")
            return
        with open(prompt_file, "r", encoding="utf-8") as f:
            system_prompt = f.read()

        messages = [{"role": "system", "content": system_prompt}]

        while True:
            user_input = input("我：")
            if user_input.lower() in ["exit", "quit", "q"]:
                break

            messages.append({"role": "user", "content": user_input})
            try:
                # 改用本地 Ollama 呼叫
                reply = ask_ollama(messages)
                print(f"機器人：{reply}")
                messages.append({"role": "assistant", "content": reply})

                try:
                    # 嘗試解析 JSON (Llama 3.2 通常能很好地遵循 JSON 格式)
                    # 如果發現模型一直回傳廢話，可以在 ask_ollama 裡加上 format='json' 參數
                    json_data = json.loads(reply)
                except json.JSONDecodeError:
                    print("無法解析 JSON，請重新輸入。")
                    continue

                if "error" in json_data:
                    print("無法理解您的指令，請重新輸入！")
                    continue

                # ==========================================
                # 以下邏輯完全維持您原本的寫法
                # ==========================================

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

                    idle_pos = {"x": 575.0, "y": -30, "z": 440.0}
                    # Swap 指令集 (維持原樣)
                    swap_instructions = [
                        {"mode": "absolute", "x": pos1["x"]+offsetx, "y": pos1["y"]+offsety, "z": pos1["z"], "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": pos1["x"]+offsetx, "y": pos1["y"]+offsety, "z": pos1["z"] - 170, "rx": rx, "ry": ry, "rz": rz},
                        {"action": "suck"},
                        {"mode": "absolute", "x": pos1["x"]+offsetx, "y": pos1["y"]+offsety, "z": 440, "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"], "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"] - 179, "rx": rx, "ry": ry, "rz": rz},
                        {"action": "release"},
                        {"mode": "absolute", "x": 433.72, "y": 78.29, "z": 440, "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": pos2["x"]+offsetx, "y": pos2["y"]+offsety, "z": pos2["z"], "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": pos2["x"]+offsetx, "y": pos2["y"]+offsety, "z": pos2["z"]-170, "rx": rx, "ry": ry, "rz": rz},
                        {"action": "suck"},
                        {"mode": "absolute", "x": 433.72, "y": 78.29, "z": 440, "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": pos1["x"]+offsetx, "y": pos1["y"]+offsety, "z": pos1["z"], "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": pos1["x"]+offsetx, "y": pos1["y"]+offsety, "z": pos1["z"] - 170, "rx": rx, "ry": ry, "rz": rz},
                        {"action": "release"},
                        {"mode": "absolute", "x": pos1["x"]+offsetx, "y": pos1["y"]+offsety, "z": 440, "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"], "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"] - 179, "rx": rx, "ry": ry, "rz": rz},
                        {"action": "suck"},
                        {"mode": "absolute", "x": 433.72, "y": 78.29, "z": 440, "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": pos2["x"]+offsetx, "y": pos2["y"]+offsety, "z": pos2["z"], "rx": rx, "ry": ry, "rz": rz},
                        {"mode": "absolute", "x": pos2["x"]+offsetx, "y": pos2["y"]+offsety, "z": pos2["z"]-170, "rx": rx, "ry": ry, "rz": rz},
                        {"action": "release"},
                        {"mode": "absolute", "x": 433.72, "y": 78.29, "z": 440, "rx": rx, "ry": ry, "rz": rz}
                    ]
                    json_data = {"swap_sequence": swap_instructions}

                # --- action 功能 ---
                elif "action" in json_data:
                    action = json_data["action"]
                    if action not in ["suck", "release", "pick", "place"]:
                        print(f"不支援的動作：{action}")
                        continue
                    if action in ["suck", "pick"]:
                        bool_value = True
                    elif action in ["release", "place"]:
                        bool_value = False
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
                    idle_pos = {"x": 575.0, "y": -30.0, "z": 440.0}
                    pos_pick = get_keycap_position(key_name, keycaps_file)
                    if not pos_pick:
                        print(f"找不到鍵帽 {key_name} 的座標")
                        continue
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
                    pick_offset = -179
                    feedback_pose = get_feedback_pose(feedback_file)
                    if feedback_pose:
                        rx, ry, rz = feedback_pose["tool_pose"]["rx"], feedback_pose["tool_pose"]["ry"], feedback_pose["tool_pose"]["rz"]
                    else:
                        rx, ry, rz = 180.0, 0.0, 133.0
                    sequence = []
                    sequence.append({"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"], "rx": rx, "ry": ry, "rz": rz})
                    sequence.append({"mode": "absolute", "x": idle_pos["x"], "y": idle_pos["y"], "z": idle_pos["z"] + pick_offset, "rx": rx, "ry": ry, "rz": rz})
                    sequence.append({"action": "suck"})
                    sequence.append({"mode": "absolute", "x": 433.72, "y": 78.29, "z": 440.0, "rx": rx, "ry": ry, "rz": rz})
                    sequence.append({"mode": "absolute", "x": pos_place["x"]+3, "y": pos_place["y"]-20, "z": pos_place["z"], "rx": rx, "ry": ry, "rz": rz})
                    sequence.append({"mode": "absolute", "x": pos_place["x"]+3, "y": pos_place["y"]-20, "z": pos_place["z"] - 170, "rx": rx, "ry": ry, "rz": rz})
                    sequence.append({"action": "release"})
                    sequence.append({"mode": "absolute", "x": 433.72, "y": 78.29, "z": 440.0, "rx": rx, "ry": ry, "rz": rz})
                    json_data = {"assembly_sequence": sequence}
                    
                # --- spell 功能 ---
                elif "spell" in json_data:
                    word = json_data["spell"].upper()
                    place_base = {"x": 550.0, "y": 30.0, "z": 440.0}
                    x_step = 35.0
                    if len(word)!= len(set(word)):
                        print("包含重複字母，請輸入別的單字")
                        continue
                    spell_sequence = []
                    # 姿態讀取
                    feedback_pose = get_feedback_pose(feedback_file)
                    if feedback_pose:
                         rx, ry, rz = feedback_pose["tool_pose"]["rx"], feedback_pose["tool_pose"]["ry"], feedback_pose["tool_pose"]["rz"]
                    else:
                         rx, ry, rz = 180.0, 0, 133
                    for idx, ch in enumerate(word):
                        pos = get_keycap_position(ch, keycaps_file)
                        if not pos:
                            print(f"找不到鍵帽 {ch} 的座標")
                            continue
                        place_x = place_base["x"]
                        place_y = place_base["y"] + idx * x_step
                        place_z = place_base["z"]
                        spell_sequence.append({"mode": "absolute", "x": pos["x"] + 3, "y": pos["y"] - 20, "z": pos["z"], "rx": rx, "ry": ry, "rz": rz})
                        spell_sequence.append({"mode": "absolute", "x": pos["x"] + 3, "y": pos["y"] - 20, "z": pos["z"] - 170, "rx": rx, "ry": ry, "rz": rz})
                        spell_sequence.append({"action": "suck"})
                        spell_sequence.append({"mode": "absolute", "x": pos["x"] + 3, "y": pos["y"] - 20, "z": 440, "rx": rx, "ry": ry, "rz": rz})
                        spell_sequence.append({"mode": "absolute", "x": place_x, "y": place_y, "z": place_z, "rx": rx, "ry": ry, "rz": rz})
                        spell_sequence.append({"mode": "absolute", "x": place_x, "y": place_y, "z": place_z - 179, "rx": rx, "ry": ry, "rz": rz})
                        spell_sequence.append({"action": "release"})
                        spell_sequence.append({"mode": "absolute", "x": place_x, "y": place_y, "z": 440, "rx": rx, "ry": ry, "rz": rz})
                        spell_sequence.append({"mode": "absolute", "x": 433.72, "y": 78.29, "z": 440, "rx": rx, "ry": ry, "rz": rz})
                    json_data = {"spell_sequence": spell_sequence}

                # --- mode 功能 ---
                elif "mode" in json_data:
                    feedback_pose = get_feedback_pose(feedback_file)
                    if not feedback_pose:
                        print("找不到手臂當前座標，無法執行")
                        continue
                    abs_pose = [feedback_pose["tool_pose"]["x"], feedback_pose["tool_pose"]["y"], feedback_pose["tool_pose"]["z"], feedback_pose["tool_pose"]["rx"], feedback_pose["tool_pose"]["ry"], feedback_pose["tool_pose"]["rz"]]
                    if json_data["mode"] == "relative":
                        abs_pose[0] += json_data.get("x", 0.0)
                        abs_pose[1] += json_data.get("y", 0.0)
                        abs_pose[2] += json_data.get("z", 0.0)
                        abs_pose[3] += json_data.get("rx", 0.0)
                        abs_pose[4] += json_data.get("ry", 0.0)
                        abs_pose[5] += json_data.get("rz", 0.0)
                        json_data = {"mode": "absolute", "x": abs_pose[0], "y": abs_pose[1], "z": abs_pose[2], "rx": abs_pose[3], "ry": abs_pose[4], "rz": abs_pose[5]}

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
                        json_data = {"mode": "absolute", "x": pos["x"] +5, "y": pos["y"]-20, "z": pos["z"], "rx": rx, "ry": ry, "rz": rz}
                    else:
                        print(f"找不到鍵帽 {key_name} 的座標")
                        continue    

                else:
                    print("模型輸出不包含可執行的指令欄位。")
                    continue
            
                # 儲存與執行
                with open(last_move_file, 'w', encoding="utf-8") as f:
                    json.dump(json_data, f, indent=2)
                with open(move_file, 'w', encoding="utf-8") as f:
                    json.dump(json_data, f, indent=2)

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