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
    # ファイル設定
    gripper_file = 'robotiq_2f_gripper_finger_distance_mm.csv'
    ft_file = 'force_torque_left_calibrated_smooth.csv'
    
    gripper_path = os.path.join(csv_dir, gripper_file)
    ft_path = os.path.join(csv_dir, ft_file)

    if not os.path.exists(gripper_path) or not os.path.exists(ft_path):
        print(f"Error: CSV files not found in {csv_dir}")
        return

    # データ読み込み
    df_gripper = pd.read_csv(gripper_path)
    df_ft = pd.read_csv(ft_path)

    # タイムスタンプ同期
    start_time = min(df_gripper['stamp_ns'].min(), df_ft['stamp_ns'].min())
    df_gripper['time_s'] = (df_gripper['stamp_ns'] - start_time) / 1e9
    df_ft['time_s'] = (df_ft['stamp_ns'] - start_time) / 1e9

    # プロット設定 (スライド用・文字さらに拡大版)
    plt.rcParams['font.size'] = 18  # 全体のベースフォントサイズをさらに拡大
    fig, ax1 = plt.subplots(figsize=(10, 6))

    # --- 左軸: グリッパ距離 ---
    color_gripper = '#1f77b4' # 鮮やかな青
    # 【変更】 軸ラベルを 18 → 22 に拡大
    ax1.set_xlabel('Time [s]', fontsize=22, fontweight='bold', labelpad=10)  
    ax1.set_ylabel('Gripper Distance [mm]', color=color_gripper, fontsize=22, fontweight='bold', labelpad=10)  
    
    ax1.plot(df_gripper['time_s'], df_gripper['robotiq_2f_gripper_finger_distance_mm.data'], 
             color=color_gripper, label='Gripper Distance', linewidth=3)
    
    # 【変更】 数字（目盛り）を 16 → 18 に拡大
    ax1.tick_params(axis='y', labelcolor=color_gripper, labelsize=18)  
    ax1.tick_params(axis='x', labelsize=18)  
    
    # 【軸の固定】 データの範囲に合わせて調整してください
    #ax1.set_xlim(0, 5.0)    
    #ax1.set_ylim(40, 110)   

    # --- 右軸: 6軸センサ ---
    ax2 = ax1.twinx()
    # 【変更】 軸ラベルを 18 → 22 に拡大
    ax2.set_ylabel('Force [N]', color='black', fontsize=22, fontweight='bold', labelpad=15)  
    
    # 主要な力(Z/Y)を太く、その他を細くプロット
    ax2.plot(df_ft['time_s'], df_ft['force_torque_left.wrench.force.y'], 
             label='Force Y', color='#2ca02c', alpha=0.7, linestyle='--', linewidth=2) # 緑
    ax2.plot(df_ft['time_s'], df_ft['force_torque_left.wrench.force.z'], 
             label='Force Z', color='#d62728', linewidth=3) # 赤
    
    # 【変更】 数字（目盛り）を 16 → 18 に拡大
    ax2.tick_params(axis='y', labelsize=18)  
    
    # 【軸の固定】 データの範囲に合わせて調整してください
    #ax2.set_ylim(-15, 5)    

    # デザイン仕上げ
    ax1.grid(True, which='major', linestyle='-', alpha=0.3)
    
    # 凡例 (右下に配置、サイズを 14 → 16 に拡大)
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='lower right', fontsize=15, frameon=False)

    # ラベルがはみ出さないように余白を自動調整
    plt.tight_layout()

    # --- 保存処理 ---
    save_dir = "/home/tsumura/my_robotiq_ws/image"
    os.makedirs(save_dir, exist_ok=True)

    path_parts = os.path.abspath(csv_dir).split(os.sep)
    if len(path_parts) >= 3:
        task_name = path_parts[-3]
        session_id = path_parts[-2]
        file_name = f"cork_f2.png"
    else:
        file_name = f"slide_plot_result.png"

    save_path = os.path.join(save_dir, file_name)
    plt.savefig(save_path, dpi=300) # 高解像度保存
    print(f"Slide-ready plot saved: {save_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', type=str, required=True)
    args = parser.parse_args()
    plot_for_slide(args.dir)