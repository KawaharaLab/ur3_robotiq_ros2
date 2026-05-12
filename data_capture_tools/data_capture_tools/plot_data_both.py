# /// script
# dependencies = [
#   "pandas",
#   "matplotlib",
# ]
# ///

import pandas as pd
import matplotlib.pyplot as plt
import os
import argparse

def plot_for_slide(csv_dir):
    # ファイル名設定
    files = {
        'gripper': 'robotiq_2f_gripper_finger_distance_mm.csv',
        'right': 'force_torque_right_calibrated_smooth.csv',
        'left': 'force_torque_left_calibrated_smooth.csv'
    }
    
    data = {'gripper': None, 'right': None, 'left': None}
    start_times = []

    # --- データの読み込みとチェック ---
    for key, filename in files.items():
        path = os.path.join(csv_dir, filename)
        if os.path.exists(path):
            try:
                df = pd.read_csv(path)
                data[key] = df
                start_times.append(df['stamp_ns'].min())
            except Exception as e:
                print(f"Warning: Could not read {filename}: {e}")
        else:
            print(f"Notice: {filename} not found, skipping.")

    if not start_times:
        print("Error: No valid CSV files found to plot.")
        return

    # タイムスタンプ同期 (存在するデータの最小値を基準にする)
    global_start = min(start_times)
    for key in data:
        if data[key] is not None:
            data[key]['time_s'] = (data[key]['stamp_ns'] - global_start) / 1e9

    # プロット設定
    plt.rcParams['font.size'] = 14
    fig, ax1 = plt.subplots(figsize=(12, 8))
    ax2 = ax1.twinx()

    # --- 左軸: グリッパ距離 ---
    if data['gripper'] is not None:
        color_gripper = '#555555' 
        ax1.plot(data['gripper']['time_s'], 
                 data['gripper']['robotiq_2f_gripper_finger_distance_mm.data'], 
                 color=color_gripper, label='Gripper Dist', 
                 linewidth=1.5, alpha=0.4, linestyle=':')
        ax1.set_ylabel('Gripper Distance [mm]', color=color_gripper, fontsize=16, fontweight='bold')
        ax1.tick_params(axis='y', labelcolor=color_gripper)
    else:
        ax1.set_yticks([]) # データがない場合は目盛りを消す

    # --- 右軸: 6軸センサ (Force) ---
    ax2.set_ylabel('Force [N]', color='black', fontsize=16, fontweight='bold')
    
    # RIGHT (実線 / 暖色系)
    if data['right'] is not None:
        ax2.plot(data['right']['time_s'], data['right']['force_torque_right.wrench.force.x'], 
                 label='R-Force X', color='#e377c2', linewidth=2) 
        ax2.plot(data['right']['time_s'], data['right']['force_torque_right.wrench.force.y'], 
                 label='R-Force Y', color='#ff7f0e', linewidth=2) 
        ax2.plot(data['right']['time_s'], data['right']['force_torque_right.wrench.force.z'], 
                 label='R-Force Z', color='#d62728', linewidth=2.5) 

    # LEFT (破線 / 寒色系)
    if data['left'] is not None:
        ax2.plot(data['left']['time_s'], data['left']['force_torque_left.wrench.force.x'], 
                 label='L-Force X', color='#9467bd', linestyle='--', linewidth=2) 
        ax2.plot(data['left']['time_s'], data['left']['force_torque_left.wrench.force.y'], 
                 label='L-Force Y', color='#1f77b4', linestyle='--', linewidth=2) 
        ax2.plot(data['left']['time_s'], data['left']['force_torque_left.wrench.force.z'], 
                 label='L-Force Z', color='#17becf', linestyle='--', linewidth=2.5) 

    ax1.set_xlabel('Time [s]', fontsize=16, fontweight='bold')
    ax2.tick_params(axis='y', labelsize=14)
    ax1.grid(True, which='major', linestyle='-', alpha=0.2)
    
    # --- 凡例の動的構成 ---
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    if lines1 + lines2:
        ax1.legend(lines1 + lines2, labels1 + labels2, 
                   loc='upper center', bbox_to_anchor=(0.5, -0.12),
                   ncol=3, fontsize=10, frameon=True)

    plt.tight_layout()

    # --- 保存処理 ---
    save_dir = "/home/tsumura/my_robotiq_ws/image"
    os.makedirs(save_dir, exist_ok=True)

    path_parts = os.path.abspath(csv_dir).split(os.sep)
    # パス構造に応じてファイル名を生成 (インデックスは環境に合わせて調整してください)
    try:
        task_name = path_parts[-4] if len(path_parts) >= 4 else "unknown_task"
        session_id = path_parts[-2] if len(path_parts) >= 2 else "unknown_session"
        file_name = f"dual_plot_{task_name}_{session_id}.png"
    except IndexError:
        file_name = "slide_plot_result.png"

    save_path = os.path.join(save_dir, file_name)
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Plot saved: {save_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', type=str, required=True)
    args = parser.parse_args()
    plot_for_slide(args.dir)