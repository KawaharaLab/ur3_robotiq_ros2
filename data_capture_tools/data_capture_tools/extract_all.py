import csv
import json
import logging
import datetime
from pathlib import Path
from collections import defaultdict
from typing import Dict, List, Optional, Any
from concurrent.futures import ProcessPoolExecutor, as_completed

import cv2
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py import message_to_ordereddict

# 同一パッケージ内のモジュールから絶対インポート
from data_capture_tools.config import CaptureConfig, load_capture_config
from data_capture_tools.bag_converter import (
    _sanitize_topic, _flatten, _format_csv_value, 
    _load_csv_field_map, _prepare_message_types, 
    _maybe_decompress_file_bag
)
from data_capture_tools.extract_batch_bag import get_trial_intervals, convert_trial_range

# ログ設定
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] (%(processName)s) %(message)s'
)
logger = logging.getLogger("batch_extractor")

def process_single_bag(bag_path: Path, config_path: str):
    """
    単一のBagセッションを処理する関数（サブプロセス側で実行される）
    """
    # サブプロセス内でロガーを再取得
    proc_logger = logging.getLogger("batch_extractor")
    
    parent_dir = bag_path.parent.parent
    
    # 未処理チェック: すでに 'trial_' で始まるディレクトリがあるか確認
    existing_trials = list(parent_dir.glob("trial_*"))
    if existing_trials:
        proc_logger.info(f"Skipping already processed: {parent_dir.name} (Found {len(existing_trials)} trials)")
        return

    proc_logger.info(f"--- Starting Process: {parent_dir.name} ---")
    
    # 構造体(config)のシリアライズエラーを防ぐため、プロセス内部でロードする
    config = load_capture_config(Path(config_path))
    
    temp_handle = None
    try:
        # 圧縮されている場合は一時的に解凍
        bag_to_read, temp_handle = _maybe_decompress_file_bag(bag_path, config, proc_logger)
        
        # 試行区間の取得
        intervals = get_trial_intervals(bag_to_read, config.bag.storage)
        if not intervals:
            proc_logger.warning(f"No trial markers found in {bag_path}")
            return
        
        # 各試行（Trial）ごとに抽出実行
        for trial in intervals:
            trial_id = trial["trial_id"]
            trial_output_dir = parent_dir / f"trial_{trial_id}"
            
            proc_logger.info(f"  >>> [{parent_dir.name}] Extracting Trial {trial_id} to {trial_output_dir}")
            convert_trial_range(
                bag_to_read, trial_output_dir, config,
                trial["start_ns"], trial["end_ns"], proc_logger
            )
            
            # パラメータを保存
            with (trial_output_dir / "params.json").open("w", encoding="utf-8") as f:
                json.dump(trial["params"], f, indent=4)
        
        proc_logger.info(f"Successfully processed: {parent_dir.name}")
            
    except Exception as e:
        proc_logger.error(f"Failed to process {bag_path}: {str(e)}", exc_info=True)
    finally:
        if temp_handle:
            temp_handle.cleanup()

def run_batch_process(root_dir: str, config_path: str, max_workers: Optional[int] = None):
    """
    指定されたルートディレクトリ配下を探索し、未処理のBagセッションを並列で一括処理する
    
    :param max_workers: 同時実行プロセス数。Noneの場合は自動（CPUコア数ベース）
    """
    root_path = Path(root_dir)
    if not root_path.exists():
        logger.error(f"Root directory not found: {root_dir}")
        return

    # 1. パターンに一致するbagフォルダをすべて検索
    bag_directories = list(root_path.glob("**/bag/session_*"))
    logger.info(f"Found {len(bag_directories)} bag sessions in total.")

    if not bag_directories:
        logger.info("No bag sessions found.")
        return

    # 2. ProcessPoolExecutor を使用して並列処理を実行
    logger.info(f"Starting parallel batch process with max_workers={max_workers}...")
    
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        # 各Bagの処理を非同期タスクとして投入
        future_to_bag = {
            executor.submit(process_single_bag, bag_path, config_path): bag_path
            for bag_path in bag_directories
        }
        
        # 完了したものから順に結果（例外）をチェック
        for future in as_completed(future_to_bag):
            bag_path = future_to_bag[future]
            try:
                future.result()  # 例外が発生していた場合はここでスローされる
            except Exception as exc:
                logger.error(f"Bag session {bag_path.parent.parent.name} generated an exception: {exc}")

def run_as_node():
    """
    ros2 run から呼び出されるエントリーポイント
    """
    DICOMO_DATA_ROOT = "/mnt/nvme1/tsumura/afterDICOMO/after_dicomo_data_test"
    CONFIG_PATH = "/home/tsumura/my_robotiq_ws/src/ur3_robotiq_ros2/data_capture_tools/config/data_capture_extract.yaml"
    
    # 割り当てるCPUプロセス数を指定したい場合は max_workers を変更してください
    # 例: max_workers=4 (指定しない/None の場合はマシンの論理コア数に近い値が自動設定されます)
    MAX_WORKERS = None 

    logger.info("Starting batch extraction process...")
    run_batch_process(DICOMO_DATA_ROOT, CONFIG_PATH, max_workers=MAX_WORKERS)
    logger.info("Batch process finished.")

if __name__ == "__main__":
    run_as_node()