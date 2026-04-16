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
import sys

def plot_robot_data(csv_dir):
    # ファイル名の定義
    gripper_file = 'robotiq_2f_gripper_finger_distance_mm.csv'
    ft_file = 'force_torque_right.csv'
    
    gripper_path = os.path.join(csv_dir, gripper_file)
    ft_path = os.path.join(csv_dir, ft_file)

    # フォルダとファイルの存在確認
    if not os.path.exists(csv_dir):
        print(f"エラー: フォルダ '{csv_dir}' が見つかりません。")
        return
    
    if not os.path.exists(gripper_path) or not os.path.exists(ft_path):
        print(f"エラー: '{csv_dir}' 内に必要なCSVファイルが不足しています。")
        print(f"確認対象: {gripper_file}, {ft_file}")
        return

    # データの読み込み
    print(f"データを読み込んでいます: {csv_dir}")
    df_gripper = pd.read_csv(gripper_path)
    df_ft = pd.read_csv(ft_path)

    # タイムスタンプ（stamp_ns）を相対秒（0秒開始）に変換
    start_time = min(df_gripper['stamp_ns'].min(), df_ft['stamp_ns'].min())
    df_gripper['time_s'] = (df_gripper['stamp_ns'] - start_time) / 1e9
    df_ft['time_s'] = (df_ft['stamp_ns'] - start_time) / 1e9

    # プロットの作成
    fig, ax1 = plt.subplots(figsize=(12, 7))

    # 左軸: グリッパの距離
    color_gripper = 'tab:blue'
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Gripper Distance [mm]', color=color_gripper, fontsize=12)
    ax1.plot(df_gripper['time_s'], df_gripper['robotiq_2f_gripper_finger_distance_mm.data'], 
             color=color_gripper, label='Gripper Distance', marker='o', markersize=4, linewidth=1.5)
    ax1.tick_params(axis='y', labelcolor=color_gripper)
    ax1.grid(True, which='both', linestyle='--', alpha=0.5)

    # 右軸: 6軸センサ（力）
    ax2 = ax1.twinx()
    ax2.set_ylabel('Force [N]', color='black', fontsize=12)
    
    # 各成分をプロット
    ax2.plot(df_ft['time_s'], df_ft['force_torque_right.wrench.force.x'], label='Force X', alpha=0.5, linestyle=':')
    ax2.plot(df_ft['time_s'], df_ft['force_torque_right.wrench.force.y'], label='Force Y', alpha=0.5, linestyle='--')
    ax2.plot(df_ft['time_s'], df_ft['force_torque_right.wrench.force.z'], label='Force Z (Vertical)', color='tab:red', linewidth=2.5)
    
    ax2.tick_params(axis='y', labelcolor='black')

    # タイトル
    plt.title(f'Robot Data Analysis\nSource: {os.path.abspath(csv_dir)}', fontsize=12)
    
    # 凡例
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right', frameon=True)

    plt.tight_layout()
    
# 1. 保存先ディレクトリの指定
    save_dir = "/home/tsumura/my_robotiq_ws/image"
    os.makedirs(save_dir, exist_ok=True)

    # 2. パスを分解してファイル名を生成
    # /home/tsumura/my_robotiq_ws/data/colorball_test_0303/20260304_214234/csv
    path_parts = os.path.abspath(csv_dir).split(os.sep)
    
    if len(path_parts) >= 3:
        # 末尾から3番目(task)と2番目(session)を取得
        task_name = path_parts[-3]
        session_id = path_parts[-2]
        file_name = f"plot_{task_name}_{session_id}.png"
    else:
        file_name = f"plot_{os.path.basename(os.path.normpath(csv_dir))}.png"

    # 3. フルパスを結合して保存
    save_path = os.path.join(save_dir, file_name)
    
    plt.savefig(save_path, dpi=300)
    print(f"プロットを保存しました: {save_path}")
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Robot CSV data plotter')
    parser.add_argument('--dir', type=str, default='csv', help='CSVファイルが格納されているフォルダへのパス (デフォルト: csv)')
    
    args = parser.parse_args()
    plot_robot_data(args.dir)