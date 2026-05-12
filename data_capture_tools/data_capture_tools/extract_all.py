import csv
import json
import logging
import datetime
from pathlib import Path
from collections import defaultdict
from typing import Dict, List, Optional, Any

import cv2
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py import message_to_ordereddict

# 同一パッケージ内のモジュールから絶対インポート
# ROS 2の環境下 (ros2 run) ではこの形式が最も安定します
from data_capture_tools.config import CaptureConfig, load_capture_config
from data_capture_tools.bag_converter import (
    _sanitize_topic, _flatten, _format_csv_value, 
    _load_csv_field_map, _prepare_message_types, 
    _maybe_decompress_file_bag
)
# 既存の抽出ロジック（単一Bag用）をインポート
from data_capture_tools.extract_batch_bag import get_trial_intervals, convert_trial_range

# ログ設定
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("batch_extractor")

def run_batch_process(root_dir: str, config_path: str):
    """
    指定されたルートディレクトリ配下を探索し、未処理のBagセッションを一括処理する
    """
    root_path = Path(root_dir)
    if not root_path.exists():
        logger.error(f"Root directory not found: {root_dir}")
        return

    config = load_capture_config(Path(config_path))
    
    # 1. dicomo_data 下の 'bag/session_...' フォルダをすべて検索
    # パターン: root / object_name / timestamp / bag / session_...
    bag_directories = list(root_path.glob("**/bag/session_*"))

    logger.info(f"Found {len(bag_directories)} bag sessions in total.")

    for bag_path in bag_directories:
        # 出力先判定用の親ディレクトリ (timestampフォルダ)
        # bag_path: .../20260505_221437/bag/session_20260505_221437
        # parent_dir: .../20260505_221437/
        parent_dir = bag_path.parent.parent
        
        # 2. 未処理チェック: すでに 'trial_' で始まるディレクトリがあるか確認
        existing_trials = list(parent_dir.glob("trial_*"))
        if existing_trials:
            logger.info(f"Skipping already processed: {parent_dir.name} (Found {len(existing_trials)} trials)")
            continue

        logger.info(f"--- Processing: {parent_dir.name} ---")
        
        temp_handle = None
        try:
            # 圧縮されている場合は一時的に解凍
            bag_to_read, temp_handle = _maybe_decompress_file_bag(bag_path, config, logger)
            
            # 試行区間の取得
            intervals = get_trial_intervals(bag_to_read, config.bag.storage)
            if not intervals:
                logger.warning(f"No trial markers found in {bag_path}")
                continue
            
            # 各試行（Trial）ごとに抽出実行
            for trial in intervals:
                trial_id = trial["trial_id"]
                trial_output_dir = parent_dir / f"trial_{trial_id}"
                
                logger.info(f"  >>> Extracting Trial {trial_id} to {trial_output_dir}")
                convert_trial_range(
                    bag_to_read, trial_output_dir, config,
                    trial["start_ns"], trial["end_ns"], logger
                )
                
                # パラメータを保存
                with (trial_output_dir / "params.json").open("w", encoding="utf-8") as f:
                    json.dump(trial["params"], f, indent=4)
            
            logger.info(f"Successfully processed: {parent_dir.name}")
                
        except Exception as e:
            logger.error(f"Failed to process {bag_path}: {str(e)}")
        finally:
            if temp_handle:
                temp_handle.cleanup()

def run_as_node():
    """
    ros2 run から呼び出されるエントリーポイント
    """
    # 実行環境に合わせたパス設定
    DICOMO_DATA_ROOT = "/mnt/nvme1/tsumura/dicomo_data"
    CONFIG_PATH = "/home/tsumura/my_robotiq_ws/src/ur3_robotiq_ros2/data_capture_tools/config/data_capture_extract.yaml"
    
    logger.info("Starting batch extraction process...")
    run_batch_process(DICOMO_DATA_ROOT, CONFIG_PATH)
    logger.info("Batch process finished.")

if __name__ == "__main__":
    run_as_node()