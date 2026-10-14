# 第2段階：取得ノードの任意診断（2026-10-14）

## 今回の変更

- GelSightとMMS101に`timing_diagnostics`起動オプションを追加。既定はfalse。
- trueのときだけ、host側の処理時間等を測り、約1秒ごとに`std_msgs/String`のJSONで報告する。
- 既存画像・力のトピック、header生成の位置と意味、画像変換、センサー設定、init、制御用フィルタは変更しない。
- 診断は計測のために追加負荷を持つ。無効/有効の受信件数を比較できる。診断が無効でも新しいPythonモジュールとstd_msgsの依存は必要。
- 秒ごとの集計は件数・平均・最小・最大。全サンプルの処理時間列やp95ではない。画像とFTの物理的な同期精度の保証ではない。

## トピック

標準launch名の場合：

```
/gelsight_mini_publisher_1/acquisition_diagnostics
/gelsight_mini_publisher_2/acquisition_diagnostics
/mms101_driver_1/acquisition_diagnostics
/mms101_driver_2/acquisition_diagnostics
```

namespaceやnode名を変えた場合は変わる。既存の収集YAMLには自動で追加しない。専用の診断記録スクリプトで保存する。
オプションは起動時に指定する。実行中にROS parameterを変更して切り替える設計ではない。

## 数値の意味

GelSight：

- `read_block_decode_ms`：OpenCV読取。次フレームの待機と画像展開を含み、遅延そのものではない。
- `preprocess_ms`：read後から色変換・crop/resize等が終わるまで。
- `read_return_interval_ms`：画像read完了同士の間隔。露光時刻同士ではない。
- `message_prepare_ms`：ROS画像作成・header付与。
- `read_end_to_header_ms`：host read終了からheader付与直後まで。カメラ内部・USB・OpenCV内部の遅延は含まない。
- `raw_publish_ms`：publish呼出しの時間。受信先に届くまでの時間ではない。
- `jpeg_prepare_encode_ms`：BGR変換とJPEG符号化。
- `compressed_message_publish_ms`：compressedメッセージ作成とpublish呼出し。
- カウンタ：読取成功、読取失敗、間引き、raw/圧縮publish、JPEG失敗。

集計timerとcapture threadが並行するため、同じ1秒窓に入るreadとpublishの件数は境界でずれる場合がある。フレームIDを持つ個別トレースではない。

MMS101：

- `read_block_ms`：read呼出しの待機を含む時間。
- `read_return_interval_ms`：データが返ったread間の時間。
- `bytes_per_read` / `packets_per_read`：readで返ったバイト数と、その後に解析されたパケット数。パケットの分割・連結を含む。
- `multi_packet_reads`：同じreadの処理で複数パケットを解析した回数。
- `parse_publish_ms`：read終了後からパケット解析・publishが終わるまで。追加のROS時計読取・FIONREAD診断も含む。
- `tty_unread_bytes`：read後にFIONREADが返したOSの未読量。機器やUSB内の待ち量ではない。未対応の場合は統計countが0。
- `last_read_end_ros_ns`：直近の成功read後に読んだROS時計。ADC測定時刻ではない。
- `window_seconds`：実測の集計時間。初回は起動待機を含むため、記録側で最初の3秒を捨てる。

時間の長さはmonotonic/steady clockで測定する。PythonとC++の単調時計の絶対epochを直接対応付ける処理はない。JSONのROS時刻は整数nsで保持する。

## ビルド

変更をGitで研究室PCへ反映した後、ワークスペース直下で：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_gelsight_package mms101_driver
source install/setup.bash
```

MMS101のC++ノード全体は研究室のROS環境でビルド検証が必要。ローカルではROS非依存の集計部をC++14でコンパイルして検証した。

## 第1回の実機測定：静止したまま、両センサーを同時取得

既存の同一センサーノードを先に終了する。UR・グリッパドライバ、data_capture_manager、init、動作コマンドは不要。
MMS101ドライバは従来どおりセンサー初期化・測定開始を行うが、ロボットの動作は指令しない。
各ターミナルで上のROS環境・ワークスペースをsourceする。

ターミナル1：

```bash
ros2 launch ros2_gelsight_package gelsight_publisher.launch.py timing_diagnostics:=true
```

ターミナル2：

```bash
ros2 launch mms101_driver mms101_dual.launch.py timing_diagnostics:=true
```

ターミナル3：

```bash
/usr/bin/python3 /home/tsumura/my_robotiq_ws/src/ur3_robotiq_ros2/utils/diagnostics/record_acquisition_diagnostics.py \
  --seconds 60 \
  --out /home/tsumura/my_robotiq_ws/acquisition_diag_run1
```

ワークスペースのパスが違う場合は変更する。出力フォルダが存在する場合は別名を指定する。
`summary.json`、`diagnostics.jsonl`、`sensor_times.csv`が生成される。画像画素・力の値は保存せず、時刻と診断集計のみを保存する。
`missing_sensors`と`missing_diagnostics`は通常どちらも空。空でなければまず起動ログ・topic名を確認する。

最初は他カメラ・ビューア・bag記録を停止した条件で測定し、フォルダを共有する。
次に本番相当の負荷を加えて比較するが、最初の結果を見る前に何条件も収集する必要はない。

## ローカル検証

```bash
PYTHONPATH=ros2_gelsight_package python3 -m unittest discover \
  -s ros2_gelsight_package/test -p test_timing_diagnostics.py -v
c++ -std=c++14 -Wall -Wextra -Werror mms101_driver/test/test_acquisition_timing.cpp \
  -o /tmp/test_acquisition_timing
/tmp/test_acquisition_timing
```

PythonはROS・カメラ入出力を模擬し、実際の画像取得・publish処理の分岐を検証する。実カメラやROS通信を検証済みという意味ではない。
新しい計測結果だけでGelSightの露光時刻やFTのADC時刻を同定せず、物理的な応答差を自動補正しない。
