"""Manage rosbag-based teleoperation dataset capture sessions."""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import sys
import threading
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import random
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32  # 追加: 通信用メッセージ
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError

from .bag_converter import convert_bag_to_dataset
from .config import CaptureConfig, load_capture_config

from sensor_msgs.msg import JointState # 追加

from geometry_msgs.msg import WrenchStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import collections

from std_msgs.msg import Float32 # 上部のインポートに追加

class ForceSensorMonitor:
    """力覚センサ(MMS101)の高速受信、ノイズ除去(移動平均)、ゼロ点補正を行うクラス"""
    def __init__(self, node: Node, topic_left='/force_torque/left', topic_right='/force_torque/right'):
        self.node = node
        
        # 1000Hzで受信するため、直近20サンプル（約0.02秒分）の移動平均を取る
        self.window_size = 20
        self._history_left = collections.deque(maxlen=self.window_size)
        self._history_right = collections.deque(maxlen=self.window_size)
        
        self._offset_left = 0.0
        self._offset_right = 0.0
        
        # 最新のデータを逃さず、かつバッファ詰まりを防ぐためのSensorData QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.sub_left = self.node.create_subscription(
            WrenchStamped, topic_left, self._left_callback, qos_profile)
        self.sub_right = self.node.create_subscription(
            WrenchStamped, topic_right, self._right_callback, qos_profile)

    def _left_callback(self, msg: WrenchStamped) -> None:
        # ★ もし挟み込む力がZ軸やX軸の場合は、ここを force.z や force.x に変更してください
        self._history_left.append(msg.wrench.force.y)

    def _right_callback(self, msg: WrenchStamped) -> None:
        self._history_right.append(msg.wrench.force.y)

    def _get_raw_average(self, history: collections.deque) -> float:
        if len(history) == 0:
            return 0.0
        return sum(history) / len(history)

    def tare(self) -> None:
        """現在の移動平均値をゼロ点として記憶する"""
        self._offset_left = self._get_raw_average(self._history_left)
        self._offset_right = self._get_raw_average(self._history_right)
        self.node.get_logger().info(
            f"[Tare] 力覚センサのゼロ点をセットしました: Left={self._offset_left:.3f}N, Right={self._offset_right:.3f}N"
        )

    def get_calibrated_left(self) -> float:
        """ゼロ点補正済みの純粋な力（絶対値）を取得"""
        raw_avg = self._get_raw_average(self._history_left)
        return abs(raw_avg - self._offset_left)

    def get_calibrated_right(self) -> float:
        """ゼロ点補正済みの純粋な力（絶対値）を取得"""
        raw_avg = self._get_raw_average(self._history_right)
        return abs(raw_avg - self._offset_right)


class DataCaptureNode(Node):
    """Coordinates rosbag2 recording and dataset conversion."""

    def __init__(self) -> None:
        super().__init__("data_capture_manager")
        # --- 既存の初期化処理 ---
        default_share: Optional[Path] = None
        try:
            default_share = Path(get_package_share_directory("data_capture_tools"))
        except PackageNotFoundError:
            default_share = Path(__file__).resolve().parent.parent / "config"
        default_config = str(default_share / "data_capture.yaml")
        config_path = (
            self.declare_parameter("config", default_config)
            .get_parameter_value()
            .string_value
        )
        self.config: CaptureConfig = load_capture_config(config_path)

        # --- C++ノード連携用の設定 ---
        self._cmd_pub = self.create_publisher(String, "/robot_cmd", 10)
        self._phase_sub = self.create_subscription(Int32, "/current_phase", self._phase_cb, 10)
        self._sequence_finished_event = threading.Event()
        self._current_phase = 0
        self._is_active_session = False

        # --- 既存のメンバ変数 ---
        self._session_id_override = (
            self.declare_parameter("session_id", "").get_parameter_value().string_value
        )
        self._stop_event = threading.Event()
        self._stop_time_ns: Optional[int] = None
        self._snapped_time_ns: Optional[int] = None
        self._discard_fast: bool = False
        self._force_save: bool = False
        self._score: Optional[str] = None
        self._bag_process: Optional[subprocess.Popen] = None
        self._bag_stdout = None
        self._bag_stderr = None
        self._session_dir: Optional[Path] = None
        self._bag_uri: Optional[Path] = None
        self._viewer_processes: list[subprocess.Popen] = []
        self._aux_processes: list[subprocess.Popen] = []
        self._viewer_subscriptions: list = []
        self._viewer_frames: dict[str, object] = {}
        self._bridge = CvBridge()
        self._convert_after_record: bool = (
            self.declare_parameter("convert_after_record", False).value
        )
        self._user_thread = threading.Thread(
            target=self._watch_user_input, daemon=True
        )
        
        self._joint_sub = self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self._current_joints = None # 常時更新される最新の関節角
        self._saved_base_joints = None # 補正完了後に保存するリセット用角度
        
        # --- 追加: マーカー通信用のパブリッシャ ---
        self._marker_pub = self.create_publisher(String, "/trial_marker", 10)
        self.get_logger().info("Marker publisher initialized on /trial_marker")
        
        self._urscript_pub = self.create_publisher(String, "/urscript_interface/script_command", 10)
        
        # --- 追加: 力覚センサモニターの初期化 ---
        self._force_monitor = ForceSensorMonitor(self)
        self._is_testing_force = False
        
        # __init__ メソッド内の適当な場所に追加
        self._current_gripper_width_m = 0.140 # 初期値(140mm = 0.14m)
        self._width_sub = self.create_subscription(
            Float32, 
            "/robotiq_2f_gripper/finger_distance_mm", 
            self._width_cb, 
            10
        )
        
        # __init__ メソッド内の適当な場所に追加
        self._calibrated_target_m = None  # ★キャリブレーション済みの目標押し込み幅

    def _phase_cb(self, msg: Int32) -> None:
        """C++側からのフェーズ情報を受信."""
        self._current_phase = msg.data
        if msg.data == 0:
            self.get_logger().info("Phase 0 detected, setting event.")
            self._sequence_finished_event.set()
            
    def _joint_cb(self, msg: JointState) -> None:
        """最新の関節角度を常に保持しておく"""
        # UR3のジョイント名順序を固定して取得
        joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        try:
            self._current_joints = [msg.position[msg.name.index(name)] for name in joint_names]
        except (ValueError, IndexError):
            pass
        
    # クラスのメソッドとして以下を追加
    def _width_cb(self, msg: Float32) -> None:
        # トピックが mm 単位なので、m 単位に変換して保持
        self._current_gripper_width_m = msg.data / 1000.0
            
    def _calculate_target_pose(self, init = True, lateral_offset=0.0, vertical_offset=0.0, speed = 0.3):
        w_obj = self.config.target_diameter
        dist = w_obj / 2.0 + self.config.arm_offset
        if not init:
            dist = 0
        
        # 接近方向: 135度
        # 垂直方向: 45度 (ここに lateral_offset を適用)
        angle_approach = math.radians(135)
        angle_perpendicular = math.radians(45)

        dx = dist * math.cos(angle_approach) + (lateral_offset * math.cos(angle_perpendicular))
        dy = dist * math.sin(angle_approach) + (lateral_offset * math.sin(angle_perpendicular))
        dz = vertical_offset

        script = f"def my_slide():\n" \
                 f"  p_curr = get_actual_tcp_pose()\n" \
                 f"  target = pose_add(p_curr, p[{dx}, {dy}, {dz}, 0, 0, 0])\n" \
                 f"  movel(target, a=0.2, v={speed})\n" \
                 f"end\n"
        return script

    def run(self) -> None:
        """メインの実行ループ."""
        self._cleanup_existing_viewers() #
        self._start_internal_viewers() #
        self._start_throttles_and_compressors() #
        
        self._user_thread.start()
        self.get_logger().info("Data Capture Node Ready. Press 'i' to init or 's' to start sequence.")
        
        try:
            while rclpy.ok() and not self._stop_event.is_set():
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self._finalize_all()

    def _watch_user_input(self) -> None:
        while not self._stop_event.is_set():
            print("\n" + "="*40)
            print(" [i]: Init Arm (Move to start pose manually)")
            print(" [s]: Start Single Trial (Record & Run)")
            print(" [ap]: Auto Repeat Loop (Record & Run multiple times)")
            print(" [as]: Auto Repeat Loop (Record & Run multiple times)")
            print(" [t]: Test Force Sensor (Visual Debug)  <-- ★追加")
            print(" [q]: Quit")
            print("="*40)
            
            line = sys.stdin.readline().strip().lower()
            if not line: continue

            if line == 'i':
                # 手動でのみ実行
                self._run_init_sequence()
            elif line == 's':
                self._execute_trial()
            elif line == 'ap':
                self._execute_auto_loop()
            elif line == 'as':
                self._execute_push_slide_loop()
            elif line == 't':
                self._test_force_sensor()  # ★追加
            elif line == 'tc':
                self._test_continuous_interrupt()  # ★追加
            elif line == 'q':
                self._stop_event.set()
                break

    def _execute_auto_loop(self) -> None:
        """10回ごとにBagを切り替えながら、補正済み角度へリセットして連続試行を行う"""
        
        # ★ 安全装置：Initが実行されていない場合は警告して弾く
        if self._calibrated_target_m is None:
            self.get_logger().error("エラー: 目標押し込み幅が未設定です。先に [i] (Init) を実行してセンタリングを行ってください。")
            return

        print("Enter total number of trials: ", end="", flush=True)
        try:
            line = sys.stdin.readline().strip()
            if not line: return
            total_count = int(line)
            if total_count <= 0: return
        except ValueError:
            return

        batch_size = 10 
        
        # ★ ここで、Initで計算・記憶した完璧な幅を使用する
        target_m = self._calibrated_target_m
        
        for i in range(total_count):
            # --- [元のロジック：10回ごとのBag分割] ---
            if i % batch_size == 0:
                if self._bag_process:
                    self._stop_rosbag() #
                self._prepare_session() #
                self._start_rosbag() #
                threading.Event().wait(3.5)

            # 速度をランダムに決定 (0.01 = 最遅, 0.1 = 標準, 1.0 = 最速)
            # 0.05 から 0.2 くらいが現実的な変化量です
            random_speed = random.uniform(0.05, 0.2)
            trial_idx = i + 1
            self.get_logger().info(f"--- Executing Trial {trial_idx} ---")

            # 3. 本番動作（run）
            start_msg = f"START,trial:{trial_idx},push_speed:{random_speed}, model:push"
            self._marker_pub.publish(String(data=start_msg)) #
            
            self._is_active_session = True
            self._sequence_finished_event.clear()
            self._cmd_pub.publish(String(data=f"run {target_m:.5f} {random_speed:.3f}"))
            self._sequence_finished_event.wait() #
            self._is_active_session = False
            
            threading.Event().wait(1.5)

            # 4. 終了処理
            end_msg = f"END,trial:{trial_idx}"
            self._marker_pub.publish(String(data=end_msg)) #
            threading.Event().wait(0.8)

            if self._stop_event.is_set():
                break

        if self._bag_process: self._stop_rosbag() #
        self.get_logger().info("Auto push loop completed.")

    def _execute_push_slide_loop(self) -> None:
        """9回のなぞり動作ループ（キャリブレーション済みの幅を使用）"""
        # ★ 安全装置：Initが実行されていない場合は警告して弾く
        if self._calibrated_target_m is None:
            self.get_logger().error("エラー: 目標押し込み幅が未設定です。先に [i] (Init) を実行してセンタリングを行ってください。")
            return

        total_count = 9
        batch_size = 10 
        
        # ★ ここで、Initで計算した完璧な幅を使用する
        target_m = self._calibrated_target_m
        
        for i in range(total_count):
            # --- [元のロジック：10回ごとのBag分割] ---
            if i % batch_size == 0:
                if self._bag_process:
                    self._stop_rosbag() #
                self._prepare_session() #
                self._start_rosbag() #
                threading.Event().wait(3.5)
                
            actual_speed = 0.03 + (i - 4) * 0.0015

            trial_idx = i + 1
            self.get_logger().info(f"--- Push & Slide (45deg) Trial {trial_idx}/{total_count} ---")
            
            # マーカー送信
            self._marker_pub.publish(String(data=f"START,trial:{trial_idx},slide_speed:{actual_speed:.4f},mode:slide"))
            
            # 1. グリッパを閉じる (C++側の run コマンドを再利用)
            self._sequence_finished_event.clear()
            self._cmd_pub.publish(String(data=f"gripper {target_m:.5f} 0.100")) 
            if not self._sequence_finished_event.wait(timeout=10.0):
                break

            # 2. なぞり動作 (URScript)
            # 押し込みは終わっているので、移動だけのスクリプトを送る
            script = self._calculate_target_pose(False, 0.03, 0.0, actual_speed) # 45度スライドロジックを適用した関数
            self._urscript_pub.publish(String(data=script))
            
            # 3. 物理的な移動を待つ
            threading.Event().wait(3.0) 
            
            # 4. グリッパを開く
            self._sequence_finished_event.clear()
            init_gripper_w = min(0.135, self.config.target_diameter + 0.050)
            self._cmd_pub.publish(String(data=f"gripper {init_gripper_w:.4f} 0.100")) 
            if not self._sequence_finished_event.wait(timeout=10.0):
                break

            # 5. グリッパを開いて init 位置に戻る 
            script = self._calculate_target_pose(False, -0.03, 0.0, 0.5) # 45度スライドロジックを適用した関数
            self._urscript_pub.publish(String(data=script))
            threading.Event().wait(3.0) 
            
            # 6. 終了処理
            end_msg = f"END,trial:{trial_idx}"
            self._marker_pub.publish(String(data=end_msg)) #
            threading.Event().wait(0.8)

            if self._stop_event.is_set():
                break

        if self._bag_process: self._stop_rosbag() #
        self.get_logger().info("Auto slide loop completed.")

    def _run_init_sequence(self) -> bool:
        """初期化：基準姿勢への復帰と、大まかな位置合わせ＋本番センタリング"""
        self.get_logger().info("--- 初期化シーケンスを開始します ---")
        self._sequence_finished_event.clear()
        
        # 1. 初期姿勢へマクロ移動
        init_gripper_w = min(0.135, self.config.target_diameter + 0.050)
        joints_rad = [math.radians(d) for d in self.config.initial_arm_pose]
        joints_str = " ".join(map(str, joints_rad))
        self._cmd_pub.publish(String(data=f"init {init_gripper_w} {joints_str}"))
        self._sequence_finished_event.wait(timeout=20.0)

        # 2. 大まかな位置合わせ (Macro Approach)
        script = self._calculate_target_pose(init=True, lateral_offset=0.0, vertical_offset=0.0, speed=0.2)
        self._urscript_pub.publish(String(data=script))
        time.sleep(3.0)

        pre_approach_width = self.config.target_diameter + self.config.safe_margin
        self._sequence_finished_event.clear()
        self._cmd_pub.publish(String(data=f"gripper {pre_approach_width:.5f} 0.100"))
        self._sequence_finished_event.wait(timeout=5.0)

        # 3. センサのゼロ点補正(Tare)
        time.sleep(0.5)
        self._force_monitor.tare()

        # 4. 本番仕様のアクティブ・センタリング実行
        effective_width_m = self._execute_active_centering()
        
        # 5. 結果の保存と待機
        self._calibrated_target_m = max(0.0, effective_width_m - self.config.push_depth)
        self.get_logger().info(
            f"キャリブレーション完了: 有効幅 {effective_width_m*1000:.1f}mm / "
            f"目標押し込み幅 {self._calibrated_target_m*1000:.1f}mm"
        )
        
        if self._current_joints:
            self._saved_base_joints = list(self._current_joints)

        # 次のデータ収集ループに備えて少し開く
        self._sequence_finished_event.clear()
        self._cmd_pub.publish(String(data=f"gripper {pre_approach_width:.4f} 0.100"))
        self._sequence_finished_event.wait(timeout=5.0)

        return True

    # def _run_init_sequence(self) -> bool:
    #     """初期化：基準姿勢への復帰と、大まかな位置合わせ＋段階的な接触検知"""
    #     self.get_logger().info("--- 初期化シーケンス（Coarse-to-Fine 接触テスト）を開始します ---")
    #     self._sequence_finished_event.clear()
        
    #     # --- 1. アームの初期姿勢へのマクロ移動 ---
    #     init_gripper_w = min(0.135, self.config.target_diameter + 0.050)
    #     joints_rad = [math.radians(d) for d in self.config.initial_arm_pose]
    #     joints_str = " ".join(map(str, joints_rad))
        
    #     self.get_logger().info("1. 初期姿勢へアームを移動中...")
    #     self._cmd_pub.publish(String(data=f"init {init_gripper_w} {joints_str}"))
        
    #     if not self._sequence_finished_event.wait(timeout=20.0):
    #         self.get_logger().error("初期姿勢への移動がタイムアウトしました。")
    #         return False

    #     # --- 2. 大まかな位置合わせ (Macro Approach) ---
    #     self.get_logger().info("2. 対象物の中心付近へ大まかな位置合わせを行います...")
        
    #     # アームを対象物に向けてスライド（元の機能を復活）
    #     script = self._calculate_target_pose(init=True, lateral_offset=0.0, vertical_offset=0.0, speed=0.2)
    #     self._urscript_pub.publish(String(data=script))
    #     time.sleep(3.0) # スライド完了まで待機

    #     # 対象物の幅 ＋ 余裕まで一気に閉じる
    #     safe_margin = self.config.safe_margin
    #     pre_approach_width = self.config.target_diameter + safe_margin
        
    #     self.get_logger().info(f"グリッパを事前幅({pre_approach_width*1000:.1f}mm)まで一気に閉じます...")
    #     self._sequence_finished_event.clear()
    #     self._cmd_pub.publish(String(data=f"gripper {pre_approach_width:.5f} 0.100"))
    #     self._sequence_finished_event.wait(timeout=5.0)

    #     # --- 3. 空中での風袋引き（Tare） ---
    #     self.get_logger().info("3. センサのゼロ点補正(Tare)を行います...")
    #     time.sleep(0.5) # アームの揺れが収まるのを待つ
    #     self._force_monitor.tare()

    #     # --- 4. 段階的な閉じ動作（Micro Step-wise Close） ---
    #     self.get_logger().info("4. 段階的接触探査を開始します...")
        
    #     # ★ config から値を読み出すように変更
    #     step_grip = self.config.centering_step_grip
    #     threshold = self.config.contact_threshold
        
    #     safe_minimum_width = self.config.target_diameter - 0.010 

    #     while rclpy.ok():
    #         current_w = self._current_gripper_width_m

    #         if current_w <= safe_minimum_width:
    #             self.get_logger().error(
    #                 f"異常事態: 接触を検知しないまま安全限界幅({safe_minimum_width*1000:.1f}mm)に達しました。探査を強制停止します。"
    #             )
    #             break

    #         target_w = current_w - step_grip

    #         # 少し速度を上げて設定ステップ幅分閉じる
    #         self._sequence_finished_event.clear()
    #         self._cmd_pub.publish(String(data=f"gripper {target_w:.5f} 0.020"))
    #         self._sequence_finished_event.wait(timeout=5.0)

    #         f_left = self._force_monitor.get_calibrated_left()
    #         f_right = self._force_monitor.get_calibrated_right()

    #         self.get_logger().info(f"現在幅: {target_w*1000:.1f}mm | 力: L={f_left:.3f}N, R={f_right:.3f}N")

    #         if f_left > threshold or f_right > threshold:
    #             touched = []
    #             if f_left > threshold: touched.append("Left")
    #             if f_right > threshold: touched.append("Right")
                
    #             self.get_logger().info(
    #                 f"★ 接触検知: {', '.join(touched)}。 探査を安全に停止しました。"
    #             )
                
    #             # ★ 次のステップのデバッグ用に、どちらが触れたかを保存しておく
    #             self._last_touched_finger = "right" if f_right > threshold else "left"
    #             break

    #     self.get_logger().info("--- 初期接触テスト完了 ---")
    #     return True
    
    # def _test_continuous_interrupt(self) -> None:
    #     """【実験】連続的に閉じながら、力覚センサの割り込みで急停止するテスト"""
    #     self.get_logger().info("--- 連続割り込み(Interrupt)テストを開始します ---")
        
    #     # --- 1. アームの初期姿勢へのマクロ移動 ---
    #     init_gripper_w = min(0.135, self.config.target_diameter + 0.050)
    #     joints_rad = [math.radians(d) for d in self.config.initial_arm_pose]
    #     joints_str = " ".join(map(str, joints_rad))
        
    #     self.get_logger().info("1. 初期姿勢へアームを移動中...")
    #     self._cmd_pub.publish(String(data=f"init {init_gripper_w} {joints_str}"))
        
    #     if not self._sequence_finished_event.wait(timeout=20.0):
    #         self.get_logger().error("初期姿勢への移動がタイムアウトしました。")
    #         return False
        
    #     # 1. センサのゼロ点補正
    #     time.sleep(0.5)
    #     self._force_monitor.tare()

    #     threshold = 0.25  # 停止閾値 (N)
    #     close_speed = 0.0005 # 閉じる速度: 10mm/s (ここを上げるとオーバーシュートが増えます)
        
    #     # 安全限界
    #     safe_minimum_width = self.config.target_diameter - 0.010 

    #     self.get_logger().info(f"速度 {close_speed*1000}mm/s で連続的に閉じます。")
        
    #     # 2. 連続動作の開始（0.0mまで完全に閉じるコマンドを発行）
    #     self._sequence_finished_event.clear()
    #     self._cmd_pub.publish(String(data=f"gripper 0.000 {close_speed:.3f}"))

    #     # 3. 超高速監視ループ（Busy Wait）
    #     touched = False
    #     while rclpy.ok():
    #         current_w = self._current_gripper_width_m
            
    #         # 安全限界チェック
    #         if current_w <= safe_minimum_width:
    #             self.get_logger().error("安全限界到達。強制ブレーキ！")
    #             self._cmd_pub.publish(String(data=f"gripper {current_w:.5f} 0.100"))
    #             break

    #         # センサ値の取得
    #         f_left = self._force_monitor.get_calibrated_left()
    #         f_right = self._force_monitor.get_calibrated_right()

    #         # 4. 割り込み検知！
    #         if f_left > threshold or f_right > threshold:
    #             # ★ 超重要: 検知した瞬間の「現在幅」を目標値として送りつけ、急ブレーキをかける
    #             brake_width = current_w
    #             self._cmd_pub.publish(String(data=f"gripper {brake_width:.5f} 0.100"))
                
    #             touched_finger = "Right" if f_right > threshold else "Left"
    #             self.get_logger().info(f"★ 割り込み検知({touched_finger})！ ブレーキ信号送信。")
    #             touched = True
    #             break
                
    #         # ループの周期を極力短くする（0.001秒 = 1000Hz相当で回す）
    #         time.sleep(0.001) 

    #     # 5. ブレーキ後のオーバーシュート計測
    #     if touched:
    #         time.sleep(0.5) # 完全にモータが止まるのを待つ
    #         final_left = self._force_monitor.get_calibrated_left()
    #         final_right = self._force_monitor.get_calibrated_right()
            
    #         self.get_logger().info("--- ブレーキ結果 ---")
    #         self.get_logger().info(f"停止設定閾値: {threshold:.3f}N")
    #         self.get_logger().info(f"最終的なめり込み力: L={final_left:.3f}N, R={final_right:.3f}N")
    #         if max(final_left, final_right) > threshold * 2:
    #             self.get_logger().warning("⚠️ オーバーシュートが大きいです。速度を下げるかステップ制御に戻すことを推奨します。")
    
    def _execute_active_centering(self) -> float:
        """
        対象物の中心と有効幅を自動探査する本番仕様のセンタリングロジック。
        ※ 事前にマクロな位置合わせとTareが完了している前提で呼び出される。
        """
        self.get_logger().info("--- アクティブ・センタリング（本番仕様：ステップ制御）を開始します ---")

        step_grip = self.config.centering_step_grip
        step_arm = self.config.centering_step_arm
        min_step = self.config.centering_min_step
        threshold = self.config.contact_threshold
        safe_minimum_width = self.config.target_diameter - 0.010 

        # =========================================================
        # フェーズ1: 段階的な初期接触探査
        # =========================================================
        self.get_logger().info("フェーズ1: 段階的な初期接触を探ります...")
        last_touched = None

        while rclpy.ok():
            current_w = self._current_gripper_width_m
            if current_w <= safe_minimum_width:
                self.get_logger().error("異常事態: 安全限界幅に達しました。探査を強制停止します。")
                return current_w

            target_w = current_w - step_grip
            self._sequence_finished_event.clear()
            self._cmd_pub.publish(String(data=f"step_gripper {target_w:.5f} 0.020 0.5"))
            self._sequence_finished_event.wait(timeout=5.0)

            f_left = self._force_monitor.get_calibrated_left()
            f_right = self._force_monitor.get_calibrated_right()

            if f_left > threshold or f_right > threshold:
                # より力が強い方を接触指とする
                last_touched = "left" if f_left > f_right else "right"
                self.get_logger().info(f"初期接触を検知: {last_touched} (L={f_left:.3f}N, R={f_right:.3f}N)")
                break

        if last_touched is None:
            return self._current_gripper_width_m

        # =========================================================
        # フェーズ2: フリップ＆ハーフ（135度平行移動による減衰ループ）
        # =========================================================
        self.get_logger().info(f"フェーズ2: 減衰ループを開始します (初期={last_touched})")

        while step_arm > min_step and rclpy.ok():
            current_w = self._current_gripper_width_m
            if current_w <= safe_minimum_width:
                self.get_logger().error("異常事態: 減衰ループ中に安全限界に達しました。")
                break

            # --- (A) アームの平行移動 (135度方向) ---
            direction = 1.0 if last_touched == "right" else -1.0
            move_dist = direction * step_arm
            
            angle_approach = math.radians(135)
            dx = move_dist * math.cos(angle_approach)
            dy = move_dist * math.sin(angle_approach)
            dz = 0.0

            script = f"def centering_slide():\n" \
                     f"  p_curr = get_actual_tcp_pose()\n" \
                     f"  target = pose_add(p_curr, p[{dx:.6f}, {dy:.6f}, {dz:.6f}, 0, 0, 0])\n" \
                     f"  movel(target, a=0.1, v=0.01)\n" \
                     f"end\n"
            
            self._urscript_pub.publish(String(data=script))
            time.sleep(0.5)

            # --- (B) ★修正部分: どちらかが接触するまで段階的に閉じる ---
            current_touched = None
            
            while rclpy.ok():
                current_w = self._current_gripper_width_m
                if current_w <= safe_minimum_width:
                    break

                target_w = current_w - step_grip
                self._sequence_finished_event.clear()
                self._cmd_pub.publish(String(data=f"step_gripper {target_w:.5f} 0.020 2.2"))
                self._sequence_finished_event.wait(timeout=5.0)

                f_left = self._force_monitor.get_calibrated_left()
                f_right = self._force_monitor.get_calibrated_right()
                
                self.get_logger().info(f"探査中幅: {target_w*1000:.1f}mm | L={f_left:.3f}N, R={f_right:.3f}N")

                # どちらかが閾値を超えたら、再探査ループを抜ける
                if f_left > threshold or f_right > threshold:
                    if f_left > threshold and f_right > threshold:
                        current_touched = "both"
                    else:
                        current_touched = "left" if f_left > f_right else "right"
                    break
            
            if current_touched is None:
                self.get_logger().error("接触を見失いました。安全限界に達した可能性があります。")
                break

            # --- (C) 接触判定と減衰ロジック ---
            if current_touched == "both":
                self.get_logger().info("両指の均等な接触を確認。センタリング完了！")
                break

            self.get_logger().info(f"接触再検知: {current_touched} | last={last_touched}")

            if current_touched != last_touched:
                self.get_logger().info(f"行き過ぎ検知({last_touched}->{current_touched})。ステップ幅を半減します。")
                step_arm /= 2.0
                step_grip /= 2.0
                last_touched = current_touched

        return self._current_gripper_width_m
    
    # def _execute_active_centering(self) -> float:
    #     """両指が均等に触れる位置を探り、その時のグリッパの有効幅(m)を返す"""
    #     self.get_logger().info("--- アクティブ・センタリングを開始します ---")

    #     # 1. 空中でのTare（ヒステリシスと重力のキャンセル）
    #     time.sleep(0.5) # アーム移動の揺れが収まるのを待つ
    #     self._force_monitor.tare()

    #     # 2. 初期接触を探る（ゆっくり閉じる）
    #     self.get_logger().info("初期接触を探査中...")
    #     self._sequence_finished_event.clear()
        
    #     # 完全に閉じる(0.0m)目標を与え、途中で止める
    #     self._cmd_pub.publish(String(data="gripper 0.000 0.010"))

    #     target_finger = None
    #     threshold = 0.25 # ノイズに強く、対象物を壊さない0.25N

    #     while rclpy.ok():
    #         f_left = self._force_monitor.get_calibrated_left()
    #         f_right = self._force_monitor.get_calibrated_right()

    #         if f_left > threshold:
    #             target_finger = "left"
    #             break
    #         if f_right > threshold:
    #             target_finger = "right"
    #             break
    #         time.sleep(0.01)

    #     # どちらかが触れたので、一旦その場で停止させる（現在幅を目標値として再送信）
    #     current_w = self._current_gripper_width_m
    #     self._cmd_pub.publish(String(data=f"gripper {current_w:.5f} 0.050"))
    #     time.sleep(0.5)

    #     # 3. フリップ＆ハーフ（適応的減衰）ループ
    #     step_arm = 0.0010  # アーム移動幅の初期値: 1.0mm
    #     step_grip = 0.0020 # グリッパ閉じ幅の初期値: 2.0mm
    #     min_step = 0.0001  # 終了判定: 0.1mm以下になれば収束

    #     last_touched = target_finger
    #     self.get_logger().info(f"初期接触: {last_touched}. 減衰ループを開始します。")

    #     while step_arm > min_step and rclpy.ok():
    #         # (A) アームの平行移動 (URScript)
    #         # Rightが当たれば正の方向(1.0)、Leftなら負の方向(-1.0)
    #         direction = 1.0 if last_touched == "right" else -1.0
            
    #         # 実際の移動距離（Rightならプラス、Leftならマイナス）
    #         move_dist = direction * step_arm
            
    #         # グリッパの開閉軸（クランプの軸）は135度
    #         angle_approach = math.radians(135)
            
    #         # 135度の斜め移動を、ロボット座標系のX軸成分とY軸成分に分解
    #         dx = move_dist * math.cos(angle_approach)
    #         dy = move_dist * math.sin(angle_approach)
    #         dz = 0.0

    #         script = f"def centering_slide():\n" \
    #                 f"  p_curr = get_actual_tcp_pose()\n" \
    #                 f"  target = pose_add(p_curr, p[{dx:.6f}, {dy:.6f}, {dz:.6f}, 0, 0, 0])\n" \
    #                 f"  movel(target, a=0.1, v=0.01)\n" \
    #                 f"end\n"
                    
    #         self._urscript_pub.publish(String(data=script))
    #         time.sleep(0.3) # アーム移動の完了を待つ

    #         # (B) グリッパを閉じる
    #         current_w = self._current_gripper_width_m
    #         target_w = current_w - step_grip
    #         self._sequence_finished_event.clear()
    #         self._cmd_pub.publish(String(data=f"gripper {target_w:.5f} 0.010"))
            
    #         # C++側が完了するのを待つ (Phase 10 が終わって 0 が返ってくるのを待つ)
    #         self._sequence_finished_event.wait(timeout=5.0)

    #         # (C) 接触判定と減衰ロジック
    #         f_left = self._force_monitor.get_calibrated_left()
    #         f_right = self._force_monitor.get_calibrated_right()

    #         if f_left > threshold and f_right > threshold:
    #             self.get_logger().info("両指の均等な接触を確認。センタリング完了！")
    #             break

    #         current_touched = "left" if f_left > threshold else "right"

    #         if current_touched != last_touched:
    #             self.get_logger().info(f"行き過ぎ検知({last_touched}->{current_touched})。ステップ幅を半減します。")
    #             step_arm /= 2.0
    #             step_grip /= 2.0
    #             last_touched = current_touched

    #     # 最終的な有効幅（ゼロ点）を返す
    #     return self._current_gripper_width_m

    def _test_force_sensor(self) -> None:
        """力覚センサの値をターミナルで視覚的にデバッグするモード"""
        self._is_testing_force = True
        self.get_logger().info("力覚センサのテストモードに入りました。")
        self.get_logger().info("空中で静止している状態でTare(ゼロ点補正)を実行します...")
        
        # 直前キャリブレーションを実行
        self._force_monitor.tare()
        import time
        
        print("\n--- Force Sensor Live View ---")
        print("力を加えてみてください。(Enterキーを押すと終了します)\n")
        
        # Enterキーが押されるまでループするスレッド用のフラグ
        stop_test = [False]
        
        def wait_for_enter():
            sys.stdin.readline()
            stop_test[0] = True
            
        t = threading.Thread(target=wait_for_enter, daemon=True)
        t.start()

        # バーの最大値（例: 2.0N でメーターが振り切れる設定）
        max_force = 2.0 
        bar_length = 30
        
        while not stop_test[0] and rclpy.ok():
            f_left = self._force_monitor.get_calibrated_left()
            f_right = self._force_monitor.get_calibrated_right()
            
            # バーの長さを計算
            l_bars = int(min(f_left / max_force, 1.0) * bar_length)
            r_bars = int(min(f_right / max_force, 1.0) * bar_length)
            
            # ターミナル上で同じ行を上書き更新 (\r を使用)
            sys.stdout.write(f"\rLeft  [{'#' * l_bars}{'-' * (bar_length - l_bars)}] {f_left:.3f} N  |  "
                             f"Right [{'#' * r_bars}{'-' * (bar_length - r_bars)}] {f_right:.3f} N")
            sys.stdout.flush()
            time.sleep(0.05) # 20Hzで描画更新
            
        print("\n\nテストモードを終了し、メインメニューに戻ります。")
        self._is_testing_force = False
    
    def _calculate_push_and_slide_script(self, target_m, slide_dist=0.05):
        """
        押し込み -> 45度方向へスライド -> 解放 -> 元の位置へ復帰
        """
        # ターゲット幅をRobotiqの0-255値に変換 (0.14m=0, 0.0m=255)
        g_pos = int((0.14 - target_m) * (255 / 0.14))
        g_pos = max(0, min(255, g_pos))
        
        # 45度方向の計算 (angle_perpendicular = 45度)
        angle_slide = math.radians(45)
        dx = slide_dist * math.cos(angle_slide)
        dy = slide_dist * math.sin(angle_slide)
        dz = 0.0 # 水平なぞりのため0

        script = "def push_and_slide_recovery():\n"
        # 1. 開始位置（init位置）を保存
        script += "  p_start = get_actual_tcp_pose()\n"
        
        # 2. 押し込み (速度固定)
        script += f"  rq_move_and_wait({g_pos})\n"
        script += "  sleep(1.0)\n"
        
        # 3. 45度方向へスライド (相対移動)
        script += f"  p_slid = pose_add(p_start, p[{dx}, {dy}, {dz}, 0, 0, 0])\n"
        script += "  movel(p_slid, a=0.2, v=0.05)\n"
        script += "  sleep(0.5)\n"
        
        # 4. グリッパを開放して待機
        script += "  rq_move_and_wait(0)\n"
        script += "  sleep(0.5)\n"
        
        # 5. 保存しておいた開始位置 p_start に直接戻る (変位の打ち消し)
        # これにより累積誤差を防ぎ、initを呼び直す手間を省きます
        script += "  movel(p_start, a=0.2, v=0.1)\n"
        script += "end\n"
        
        return script

    def _execute_trial(self) -> None:
        """1回の施行（録画・動作・スコアリング）を実行."""
        self.get_logger().info("Starting new trial...")

        # 1. YAMLから値を読み取って計算 (configクラスに属性がある前提)
        target_m = max(0.0,self.config.target_diameter - self.config.push_depth) # 負の値にならないようガード
        
        # 1. 準備と録画開始
        self._prepare_session() #
        self._start_rosbag() #

        
        # 2. ロボット動作開始命令
        self._is_active_session = True
        self._sequence_finished_event.clear()
        self.get_logger().info(f"Sending run command with target: {target_m}m")
        self._cmd_pub.publish(String(data=f"run {target_m}"))
        
        # 3. 動作完了（Phase 0）を待機
        self.get_logger().info("Sequence in progress. Waiting for completion...")
        self._sequence_finished_event.wait()
        
        # 4. 録画停止
        self._record_stop_time_if_missing() #
        self._stop_rosbag() #
        self._is_active_session = False
        
        # 5. スコア入力待ち
        print("\nTrial finished. Enter score (1 or 2 to save, 0 to discard): ")
        score_line = sys.stdin.readline().strip()
        if score_line in {"0", "1", "2"}:
            self._score = score_line
            self._force_save = (score_line != "0")
            self._finalize_session()
        else:
            self.get_logger().warning("Invalid score. Discarding trial.")
            self._discard_session()

    def _finalize_session(self) -> None:
        """セッションごとのデータ保存処理."""
        # 元の _finalize() のロジックをベースにセッション保存
        if self._score is not None and self._session_dir:
            (self._session_dir / "score.txt").write_text(f"{self._score}\n")
            
        if self._force_save:
            self.get_logger().info(f"Saving session to {self._session_dir}")
            if self._convert_after_record:
                # 必要に応じて変換実行
                pass 
        else:
            self._discard_session()
        
        # 次の施行のためにリセット
        self._score = None
        self._force_save = False

    def _discard_session(self) -> None:
        """現在のセッションデータを破棄."""
        if self._session_dir and self._session_dir.exists():
            shutil.rmtree(self._session_dir, ignore_errors=True)
            self.get_logger().info("Session discarded.")

    def _finalize_all(self) -> None:
        """ノード終了時のクリーンアップ."""
        self._stop_viewers() #
        self._stop_aux_processes() #
        self._cleanup_existing_viewers() #


    def _prepare_session(self) -> None:
        timestamp = self._session_id_override or datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = self.config.output_root / self.config.task_name / timestamp
        session_dir.mkdir(parents=True, exist_ok=True)
        (session_dir / "bag").mkdir(exist_ok=True)
        (session_dir / "logs").mkdir(exist_ok=True)
        shutil.copy2(self.config.source_path, session_dir / "config.yaml")
        (session_dir / "prompt.txt").write_text(
            self.config.prompt + "\n", encoding="utf-8"
        )
        (session_dir / "label.txt").write_text(
            self.config.label + "\n", encoding="utf-8"
        )

        bag_uri = session_dir / "bag" / f"session_{timestamp}"
        self._session_dir = session_dir
        self._bag_uri = bag_uri

    def _start_rosbag(self) -> None:
        if self._bag_uri is None:
            raise RuntimeError("Session directory was not initialized")

        cmd = [
            "ros2",
            "bag",
            "record",
            "-s",
            self.config.bag.storage,
        ]
        if self.config.bag.compression_mode:
            cmd.extend(["--compression-mode", self.config.bag.compression_mode])
        if self.config.bag.compression_format:
            cmd.extend(["--compression-format", self.config.bag.compression_format])
        cmd.extend(["-o", str(self._bag_uri)])
        cmd.extend([topic.name for topic in self.config.topics])

        log_dir = self._session_dir / "logs" if self._session_dir else Path(".")
        stdout_path = log_dir / "rosbag_stdout.log"
        stderr_path = log_dir / "rosbag_stderr.log"
        self._bag_stdout = stdout_path.open("w", encoding="utf-8")
        self._bag_stderr = stderr_path.open("w", encoding="utf-8")
        self.get_logger().info(f"Launching rosbag2: {' '.join(cmd)}")
        self._bag_process = subprocess.Popen(
            cmd,
            stdout=self._bag_stdout,
            stderr=self._bag_stderr,
            env=os.environ.copy(),
        )

    
    def _finalize(self) -> None:
        self._stop_rosbag(fast=self._discard_fast)
        self._stop_viewers()
        self._stop_aux_processes()
        self._cleanup_existing_viewers()  # ensure stragglers are gone
        if not (self._session_dir and self._bag_uri and self._bag_uri.exists()):
            self.get_logger().error("Bag output not found; skipping conversion.")
            return

        if self._discard_fast:
            self.get_logger().info("Discard flag set; skipping save and removing session directory.")
            shutil.rmtree(self._session_dir, ignore_errors=True)
            return

        if not self._force_save and not self._confirm_save():
            self.get_logger().info("Discarding captured data at user request.")
            shutil.rmtree(self._session_dir, ignore_errors=True)
            return

        if self._score is not None:
            try:
                (self._session_dir / "score.txt").write_text(f"{self._score}\n", encoding="utf-8")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"Failed to write score.txt: {exc}")

        if self._prompt_discard_for_zero:
            if not self._confirm_keep_for_zero():
                self.get_logger().info("Score 0: user chose to discard (default). Removing session directory.")
                shutil.rmtree(self._session_dir, ignore_errors=True)
                return
            else:
                self.get_logger().info("Score 0: user chose to keep. Proceeding to save.")
                self._force_save = True

        if not self._convert_after_record:
            self.get_logger().info(
                "Conversion disabled (convert_after_record:=false); keeping bag only."
            )
            return

        try:
            convert_bag_to_dataset(
                self._bag_uri,
                self._session_dir,
                self.config,
                logger=self.get_logger(),
                cutoff_stamp_ns=self._stop_time_ns,
            )
            self.get_logger().info(
                f"Capture complete. Dataset stored in {self._session_dir}"
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to convert bag: {exc}")

    def _stop_rosbag(self, fast: bool = False) -> None:
        if self._bag_process and self._bag_process.poll() is None:
            self.get_logger().info("Stopping rosbag2 process...")
            self._bag_process.send_signal(signal.SIGINT)
            try:
                self._bag_process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                self.get_logger().warning(
                    "rosbag2 did not terminate after SIGINT; sending SIGTERM"
                )
                self._bag_process.send_signal(signal.SIGTERM)
                try:
                    self._bag_process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self.get_logger().warning(
                        "rosbag2 still running after SIGTERM; sending SIGKILL"
                    )
                    self._bag_process.kill()
                    self._bag_process.wait(timeout=5)
        if self._bag_stdout:
            self._bag_stdout.close()
        if self._bag_stderr:
            self._bag_stderr.close()

    def _start_throttles_and_compressors(self) -> None:
        image_rate = self.config.image_throttle_hz
        other_rate = self.config.other_throttle_hz

        for topic in self.config.topics:
            if topic.mode == "image":
                # Subscribe directly to compressed images; upstream is expected to publish them.
                source_topic = f"{topic.name}/compressed"
                topic.type = "sensor_msgs/msg/CompressedImage"

                if image_rate is None or image_rate <= 0:
                    topic.name = source_topic
                    self.get_logger().info(
                        f"Skipping image throttle for {source_topic} (rate <= 0)"
                    )
                else:
                    throttled_topic = f"{source_topic}_throttled"
                    if self._spawn_throttle(source_topic, throttled_topic, image_rate):
                        topic.name = throttled_topic
                    else:
                        topic.name = source_topic
                        self.get_logger().warning(
                            f"Using unthrottled image topic {source_topic} (throttle helper failed)"
                        )
            else:
                source_topic = topic.name
                if other_rate is None or other_rate <= 0:
                    topic.name = source_topic
                    self.get_logger().info(
                        f"Skipping throttle for {source_topic} (rate <= 0)"
                    )
                else:
                    throttled_topic = f"{source_topic}_throttled"
                    if self._spawn_throttle(source_topic, throttled_topic, other_rate):
                        topic.name = throttled_topic
                    else:
                        topic.name = source_topic
                        self.get_logger().warning(
                            f"Using unthrottled topic {source_topic} (throttle helper failed)"
                        )

    def _spawn_throttle(self, input_topic: str, output_topic: str, rate: float) -> bool:
        cmd = [
            "ros2",
            "run",
            "topic_tools",
            "throttle",
            "messages",
            input_topic,
            str(rate),
            output_topic,
        ]
        ok, _ = self._spawn_aux_process(cmd, desc=f"throttle {input_topic} -> {output_topic} @ {rate} Hz")
        return ok

    def _spawn_aux_process(self, cmd: list[str], desc: str, required: bool = False) -> tuple[bool, Path]:
        log_dir = self._session_dir / "logs" if self._session_dir else Path(".")
        log_dir.mkdir(parents=True, exist_ok=True)
        slug = "".join(ch if ch.isalnum() else "_" for ch in desc)[:80] or "helper"
        log_path = log_dir / f"{slug}.log"

        try:
            with log_path.open("w", encoding="utf-8") as log_file:
                proc = subprocess.Popen(
                    cmd,
                    env=os.environ.copy(),
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                )
            self._aux_processes.append(proc)
            self.get_logger().info(
                f"Started helper: {desc} (pid {proc.pid}); logs: {log_path}"
            )

            # Briefly wait to ensure the helper stays alive; if it exits, treat as failure.
            threading.Event().wait(1.0)
            if proc.poll() is not None:
                msg = f"Helper '{desc}' exited early with code {proc.returncode}; see {log_path}"
                if required:
                    self.get_logger().error(msg)
                else:
                    self.get_logger().warning(msg)
                return False, log_path

            return True, log_path
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"Failed to start helper '{desc}': {exc}; see {log_path}"
            )
            return False, log_path

    def _start_internal_viewers(self) -> None:
        if not self.config.viewer_topics:
            return

        for topic in self.config.viewer_topics:
            sub = self.create_subscription(
                Image,
                topic,
                self._make_image_callback(topic),
                10,
            )
            self._viewer_subscriptions.append(sub)
            self.get_logger().info(f"Viewer subscribed to {topic}")

        # Render at ~20 Hz to keep UI responsive without heavy CPU use.
        self._viewer_timer = self.create_timer(0.05, self._render_viewers)

    def _cleanup_existing_viewers(self) -> None:
        # Best-effort: close any lingering showimage windows/processes from prior runs.
        wmctrl_bin = shutil.which("wmctrl")
        if wmctrl_bin:
            try:
                subprocess.run(
                    [wmctrl_bin, "-c", "showimage"],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception:
                pass
        # Also try pkill as a fallback.
        subprocess.run(
            ["pkill", "-f", "image_tools showimage"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _arrange_viewer_windows(self) -> None:
        # No-op: internal OpenCV viewer handles layout.
        return

    def _make_image_callback(self, topic: str):
        def _cb(msg: Image) -> None:
            try:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self._viewer_frames[topic] = frame
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(f"Failed to decode image for {topic}: {exc}")

        return _cb

    def _render_viewers(self) -> None:
        if not self._viewer_frames:
            return

        # Arrange windows side-by-side at smaller size.
        positions = [(100, 100), (100, 500)]
        width, height = 640, 360

        for idx, (topic, frame) in enumerate(self._viewer_frames.items()):
            if frame is None:
                continue
            resized = cv2.resize(frame, (width, height)) if frame is not None else frame
            window_name = f"viewer: {topic}"
            cv2.imshow(window_name, resized)
            x, y = positions[idx] if idx < len(positions) else (idx * 240, 0)
            cv2.moveWindow(window_name, x, y)

        # Needed for imshow to update.
        cv2.waitKey(1)

    def _stop_viewers(self) -> None:
        if self._viewer_timer:
            self._viewer_timer.cancel()
            self._viewer_timer = None
        self._viewer_subscriptions.clear()
        self._viewer_frames.clear()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    def _stop_aux_processes(self) -> None:
        for proc in self._aux_processes:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
        self._aux_processes.clear()

    def _confirm_save(self) -> bool:
        if not sys.stdin.isatty():
            self.get_logger().info("Non-interactive stdin; keeping captured data by default.")
            return True

        prompt = f"Save captured data in {self._session_dir}? [Y/n]: "
        try:
            response = input(prompt)
        except EOFError:
            return True

        return response.strip().lower() not in {"n", "no"}

    def _confirm_keep_for_zero(self) -> bool:
        """Prompt whether to keep data when stop key 0 was used (default discard)."""
        if not sys.stdin.isatty():
            self.get_logger().info(
                "Non-interactive stdin; defaulting to discard for stop key 0."
            )
            return False

        prompt = "Keep captured data for score 0? [y/N]: "
        try:
            response = input(prompt)
        except EOFError:
            return False

        return response.strip().lower() in {"y", "yes"}

    def _record_stop_time_if_missing(self) -> None:
        if self._stop_time_ns is None:
            try:
                self._stop_time_ns = self.get_clock().now().nanoseconds
            except Exception:
                # Fallback to wall time in nanoseconds if ROS clock is unavailable
                self._stop_time_ns = int(datetime.now().timestamp() * 1e9)

    def _record_snapped_time(self) -> None:
        if self._snapped_time_ns is not None:
            return

        try:
            snapped_time_ns = self.get_clock().now().nanoseconds
        except Exception:
            snapped_time_ns = int(datetime.now().timestamp() * 1e9)

        self._snapped_time_ns = snapped_time_ns
        self.get_logger().info(f"Snapped time recorded at {snapped_time_ns} ns.")

        if not self._session_dir:
            return

        snapped_file = self._session_dir / "snapped_time.txt"
        try:
            snapped_file.write_text(f"{snapped_time_ns}\n", encoding="utf-8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Failed to write snapped_time.txt: {exc}")

        config_copy = self._session_dir / "config.yaml"
        if config_copy.exists():
            try:
                with config_copy.open("a", encoding="utf-8") as config_file:
                    config_file.write(f"\nsnapped_time_ns: {snapped_time_ns}\n")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"Failed to append snapped_time to config.yaml: {exc}")


def main() -> None:
    rclpy.init()
    node = DataCaptureNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
