# /// script
# dependencies = [
#   "pandas",
#   "numpy",
# ]
# ///

import pandas as pd
from pathlib import Path
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ft_calibrator")

def calibrate_csv(csv_path: Path, offset_count: int = 20, min_required_samples: int = 50):
    """
    CSVを読み込み、補正後のデータを別ファイル名で保存する。
    """
    try:
        # 1. 読み込み
        df = pd.read_csv(csv_path)
        
        # 2. 欠損・極端に短いデータのガード
        sample_count = len(df)
        if sample_count < min_required_samples or sample_count < offset_count:
            logger.warning(f"  [Skip] Insufficient data in {csv_path.name} ({sample_count} samples)")
            return False

        # 3. 補正計算
        target_cols = [c for c in df.columns if "wrench.force" in c or "wrench.torque" in c]
        if not target_cols:
            return False

        # 各軸ごとの平均を算出し、差し引く
        offsets = df[target_cols].head(offset_count).mean()
        df[target_cols] = df[target_cols] - offsets

        # 4. 別名で保存 (例: force_torque_left.csv -> force_torque_left_calibrated.csv)
        # .with_stem は Python 3.9+ で使用可能。古い場合は .with_name を使用。
        new_filename = csv_path.stem + "_calibrated.csv"
        output_path = csv_path.with_name(new_filename)
        
        df.to_csv(output_path, index=False)
        logger.info(f"  Saved calibrated data to: {new_filename}")
        return True

    except Exception as e:
        logger.error(f"  [Error] {csv_path.name}: {e}")
        return False

def main():
    ROOT_DIR = Path("/mnt/nvme1/tsumura/dicomo_data")
    # 探索対象のオリジナルファイル名
    target_files = ["force_torque_left.csv", "force_torque_right.csv"]
    
    csv_dirs = list(ROOT_DIR.glob("**/trial_*/csv"))
    
    for csv_dir in csv_dirs:
        for target in target_files:
            csv_path = csv_dir / target
            if not csv_path.exists():
                continue
            
            # --- チェックを外して常に実行するように変更 ---
            # これにより、条件を変えて実行するたびに _calibrated.csv が更新されます
            logger.info(f"Processing: {csv_dir.parents[1].name}/{csv_dir.parent.name}/{target}")
            calibrate_csv(csv_path)

if __name__ == "__main__":
    main()