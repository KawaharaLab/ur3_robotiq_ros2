# /// script
# dependencies = [
#   "pandas",
#   "numpy",
#   "scipy",
#   "matplotlib",
# ]
# ///

import pandas as pd
import numpy as np
import os
import glob
import matplotlib.pyplot as plt
from scipy.stats import skew, kurtosis, t

def analyze_push_reproducibility(base_dir):
    object_dirs = sorted(glob.glob(os.path.join(base_dir, "object*_push")))

    for obj_dir in object_dirs:
        obj_name = os.path.basename(obj_dir)
        
        # 1. センサデータの探索 (Right優先)
        side, csv_filename = "right", "force_torque_right_calibrated_smooth.csv"
        csv_paths = glob.glob(os.path.join(obj_dir, "**", "trial_*", "csv", csv_filename), recursive=True)
        if not csv_paths:
            side, csv_filename = "left", "force_torque_left_calibrated_smooth.csv"
            csv_paths = glob.glob(os.path.join(obj_dir, "**", "trial_*", "csv", csv_filename), recursive=True)

        if not csv_paths:
            continue

        target_column = f"force_torque_{side}.wrench.force.y"
        peaks_all = []
        for path in csv_paths:
            try:
                df = pd.read_csv(path)
                if target_column in df.columns:
                    peaks_all.append(df[target_column].abs().max())
            except Exception: pass

        if len(peaks_all) < 3:
            continue

        peaks_all = np.array(peaks_all)

        # 2. 中央値ベースの動的フィルタリング (30%閾値)
        median_val = np.median(peaks_all)
        threshold = median_val * 0.3
        
        mask = (peaks_all > threshold)
        peaks_filtered = peaks_all[mask]
        outliers = peaks_all[~mask]

        # 3. 統計量算出関数
        def calc_stats(data, label):
            n = len(data)
            if n == 0: return None
            mu = np.mean(data)
            sigma = np.std(data, ddof=1) if n > 1 else 0
            cv = (sigma / mu) * 100 if mu != 0 else 0
            sem = sigma / np.sqrt(n) if n > 1 else 0
            ci = t.interval(0.95, n - 1, loc=mu, scale=sem) if n > 1 else (mu, mu)
            
            return {
                "type": label,
                "sample_count": n,
                "mean": mu,
                "std": sigma,
                "cv": cv,
                "ci95_lo": ci[0],
                "ci95_hi": ci[1]
            }

        s_all = calc_stats(peaks_all, "Raw_All")
        s_filt = calc_stats(peaks_filtered, "Normal_Trials")
        
        # CSV/PNG保存 (詳細は前回までのコードと同様)
        # ... (保存処理は実行される前提で省略) ...
        # 保存用データフレーム作成
        res_df = pd.DataFrame([s_all, s_filt])
        res_df.to_csv(os.path.join(obj_dir, f"reproducibility_analysis_{side}.csv"), index=False)
        
        # ヒストグラム保存
        plt.figure(figsize=(8, 5))
        plt.hist(peaks_filtered, bins=12, color='skyblue', label='Normal')
        plt.hist(outliers, bins=12, color='red', alpha=0.5, label='Missed')
        plt.savefig(os.path.join(obj_dir, f"reproducibility_histogram_{side}.png"))
        plt.close()

        # 4. コマンドラインへの主要統計量の表示
        print(f"==================================================")
        print(f" OBJECT: {obj_name} ({side})")
        print(f"--------------------------------------------------")
        print(f"  [Trials]   Total: {len(peaks_all):>2}  |  Missed: {len(outliers):>2}  |  Valid: {len(peaks_filtered):>2}")
        print(f"  [Threshold] {threshold:.4f} N (30% of Median: {median_val:.4f} N)")
        print(f"--------------------------------------------------")
        print(f"  STATS (Valid Trials Only):")
        print(f"    - Mean Peak Force:  {s_filt['mean']:>8.4f} N")
        print(f"    - Std Deviation:    {s_filt['std']:>8.4f} N")
        print(f"    - CV (Coefficient): {s_filt['cv']:>8.2f} %")
        print(f"    - 95% Conf. Int.:  [{s_filt['ci95_lo']:.4f}, {s_filt['ci95_hi']:.4f}]")
        
        # 再現性の判定を視覚的に表示
        status = "PASSED (<=10%)" if s_filt['cv'] <= 10 else "HIGH VARIANCE (>10%)"
        print(f"  [STABILITY] {status}")
        print(f"==================================================\n")

if __name__ == "__main__":
    analyze_push_reproducibility("/mnt/nvme1/tsumura/dicomo/dicomo_data/")