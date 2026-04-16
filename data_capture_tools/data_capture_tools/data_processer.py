import os
import torch
import pandas as pd
import numpy as np
from pathlib import Path
from tqdm import tqdm
from PIL import Image
import argparse

# ==========================================
# 1. パス・パラメータ設定
# ==========================================
# ★ここを「物体名フォルダ」ではなく、その上の「data」フォルダに設定してください
RAW_DATA_ROOT = Path("/home/tsumura/my_robotiq_ws/data")
PROCESSED_ROOT = Path("/home/tsumura/my_robotiq_ws/processed_data")

IMG_NUM = 5           
SENSOR_STEPS = 50     
IMG_SIZE = (128, 128) 

LABEL_MAP = {
    "mochi-mochi": 0, "muni-muni": 1, "puni-puni": 2, "sara-sara": 3,
    "tsuru-tsuru": 4, "beta-beta": 5, "kachi-kachi": 6, "fuwa-fuwa": 7
}

GRIPPER_COL = 'robotiq_2f_gripper_finger_distance_mm.data'
FT_COLS = [
    'force_torque_left.wrench.force.x', 'force_torque_left.wrench.force.y', 'force_torque_left.wrench.force.z',
    'force_torque_left.wrench.torque.x', 'force_torque_left.wrench.torque.y', 'force_torque_left.wrench.torque.z'
]

# ==========================================
# 2. 処理ロジック
# ==========================================

def find_stable_time(gripper_csv_path, threshold=0.01):
    df = pd.read_csv(gripper_csv_path)
    df['diff'] = df[GRIPPER_COL].diff().abs()
    stable_points = df[(df.index > 5) & (df['diff'] < threshold)]
    if stable_points.empty:
        return int(df['stamp_ns'].iloc[0])
    return int(stable_points['stamp_ns'].iloc[0])

def extract_and_save(trial_path, output_path):
    try:
        # --- ラベル取得 ---
        label_file = trial_path / "label.txt"
        with open(label_file, 'r', encoding='utf-8') as f:
            content = f.read().strip()
            label_name = content.split(',')[1].strip() if ',' in content else content
        
        label_id = LABEL_MAP.get(label_name, -1)
        if label_id == -1: return False

        # --- 起点時刻特定 ---
        t_stable = find_stable_time(trial_path / "csv/robotiq_2f_gripper_finger_distance_mm.csv")

        # --- 6軸センサ ---
        ft_df = pd.read_csv(trial_path / "csv/force_torque_left.csv")
        start_idx = (ft_df['stamp_ns'] - t_stable).abs().argmin()
        ft_seq = ft_df.iloc[start_idx : start_idx + SENSOR_STEPS][FT_COLS].values
        
        if len(ft_seq) < SENSOR_STEPS:
            pad_val = ft_seq[-1] if len(ft_seq) > 0 else np.zeros(6)
            padding = np.tile(pad_val, (SENSOR_STEPS - len(ft_seq), 1))
            ft_seq = np.vstack([ft_seq, padding])

        # --- GelSight画像 ---
        img_dir = trial_path / "images/gelsight_left_image_raw_compressed"
        all_imgs = sorted(list(img_dir.glob("*.png")))
        if not all_imgs: return False
        
        img_stamps = np.array([int(f.stem) for f in all_imgs])
        img_start_idx = (np.abs(img_stamps - t_stable)).argmin()
        
        selected_paths = all_imgs[img_start_idx : img_start_idx + IMG_NUM]
        if len(selected_paths) < IMG_NUM:
            selected_paths += [selected_paths[-1]] * (IMG_NUM - len(selected_paths))
        
        img_tensors = [np.array(Image.open(p).convert('RGB').resize(IMG_SIZE)) for p in selected_paths]
        images_tensor = torch.tensor(np.stack(img_tensors), dtype=torch.float32).permute(0, 3, 1, 2) / 255.0

        # --- 保存 ---
        torch.save({
            "sensor": torch.tensor(ft_seq, dtype=torch.float32), 
            "images": images_tensor,                             
            "label": torch.tensor(label_id, dtype=torch.long)
        }, output_path)
        return True
    except Exception:
        return False

# ==========================================
# 3. メインループ（全物体巡回対応）
# ==========================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--object", type=str, help="特定の物体のみ実行したい場合に指定")
    args = parser.parse_args()

    # RAW_DATA_ROOT直下のフォルダを全て取得（物体フォルダ一覧）
    if args.object:
        object_dirs = [RAW_DATA_ROOT / args.object]
    else:
        # .successなどを持たない、純粋な「フォルダ」のみを物体フォルダとして抽出
        object_dirs = [d for d in RAW_DATA_ROOT.iterdir() if d.is_dir()]

    for obj_dir in object_dirs:
        # その物体フォルダの中に、時刻フォルダ（試行データ）があるか探す
        # かつ、.successがあり、かつ.failedがないものだけをリストアップ
        trial_dirs = [
            d for d in obj_dir.iterdir() 
            if d.is_dir() and (d / ".success").exists() and not (d / ".failed").exists()
        ]
        
        if not trial_dirs:
            continue

        print(f"\n物体: {obj_dir.name} ({len(trial_dirs)}件の有効な試行)")
        
        for trial_dir in tqdm(trial_dirs):
            out_file = PROCESSED_ROOT / obj_dir.name / trial_dir.name / "data.pt"
            if out_file.exists(): continue
            
            out_file.parent.mkdir(parents=True, exist_ok=True)
            success = extract_and_save(trial_dir, out_file)
            if not success:
                # 失敗した空フォルダを削除
                if out_file.parent.exists() and not any(out_file.parent.iterdir()):
                    out_file.parent.rmdir()

if __name__ == "__main__":
    main()