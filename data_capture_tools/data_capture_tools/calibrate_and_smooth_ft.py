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
logger = logging.getLogger("ft_processor")

def process_ft_csv(csv_path: Path, offset_count: int = 10, window_size: int = 5):
    """
    1. ゼロ点補正 (Calibration)
    2. 移動平均による平滑化 (Smoothing)
    3. 右手のみX軸を-1倍 (Invert X for right sensor)
    """
    try:
        df = pd.read_csv(csv_path)
        
        # データ数チェック
        if len(df) < max(offset_count, window_size, 50):
            logger.warning(f"  [Skip] Insufficient data: {csv_path.name}")
            return False

        # FT関連のカラムを特定
        target_cols = [c for c in df.columns if "wrench.force" in c or "wrench.torque" in c]
        if not target_cols:
            return False

        # --- 1. ゼロ点補正 (各軸独立) ---
        offsets = df[target_cols].head(offset_count).mean()
        df[target_cols] = df[target_cols] - offsets

        # --- 2. 右手(right)の場合のみX軸を-1倍 ---
        # カラム名が 'force_torque_right.wrench.force.x' のような形式を想定
        if "right" in csv_path.name:
            x_cols = [c for c in target_cols if c.endswith("force.x") or c.endswith("torque.x")]
            for col in x_cols:
                df[col] = df[col] * -1.0
            logger.info(f"  [Invert] Inverted X-axis for {csv_path.name}")

        # --- 3. 平滑化 (移動平均) ---
        # min_periods=1 とすることで、データの端でNaNになるのを防ぎます
        df[target_cols] = df[target_cols].rolling(window=window_size, min_periods=1, center=True).mean()

        # 4. 保存
        output_path = csv_path.with_name(csv_path.stem + "_calibrated_smooth.csv")
        df.to_csv(output_path, index=False)
        logger.info(f"  [Success] Saved processed data to: {output_path.name}")
        return True

    except Exception as e:
        logger.error(f"  [Error] {csv_path.name}: {e}")
        return False

def main():
    ROOT_DIR = Path("/mnt/nvme1/tsumura/dicomo_data")
    target_files = ["force_torque_left.csv", "force_torque_right.csv"]
    
    # 探索
    csv_dirs = list(ROOT_DIR.glob("**/trial_*/csv"))
    
    for csv_dir in csv_dirs:
        for target in target_files:
            csv_path = csv_dir / target
            if csv_path.exists():
                # 常に上書き実行
                process_ft_csv(csv_path, offset_count=10, window_size=5)

if __name__ == "__main__":
    main()