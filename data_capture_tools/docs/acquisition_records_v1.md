# 取得時刻の追加記録 v1

## 変更範囲

既存の画像・WrenchStamped・headerの定義、取得周期、力の変換係数、カメラ設定、init/ロボット動作を維持する。学習用の加工・補間・遅延補正は行わない。追加記録は `acquisition_records:=true` で有効化し、既定はfalse。秒単位の集計を出す既存 `timing_diagnostics` とは独立。

`std_msgs/msg/String` のJSONを、各センサノードのprivate topic `~/acquisition_timing` に送る。data_capture.yamlには4トピックを追加した。独自configで収集する場合は同じ4エントリを追加すること。通常のbag収集ではセンサとメタデータを同じbagに保存する。

- `/gelsight_mini_publisher_1/acquisition_timing`
- `/gelsight_mini_publisher_2/acquisition_timing`
- `/mms101_driver_1/acquisition_timing`
- `/mms101_driver_2/acquisition_timing`

### 共通の時刻

時刻はJSONの整数で保存する。nanosecond値をJavaScriptのNumber等へ変換すると丸められる可能性があるため、後処理ではint64/整数のまま扱う。`stream_id`でノードの起動を識別。ROS clockのtypeとuse_sim_timeも記録する。

各時計サンプル `[monotonic_before_ns, ros_ns, monotonic_after_ns]` は、ROS時計を読む前後に単調時計を読む。ROS時計と単調時計を同一値と仮定せず、この区間で対応を評価する。これは外部時計の校正ではない。単調時計はLinuxではPythonのtime.monotonic_nsとC++のCLOCK_MONOTONICを用いる。別ホストや再起動をまたいで直接比較しない。

## GelSight

公開したraw画像ごとに、同じ既存header_nsとframe_seqを付けたレコードを送信。frame_seqは成功したホスト読み取りの通番で、カメラ内部のフレーム番号ではない。間引きされた画像はレコードも公開されないため通番に飛びがあり得る。compressed_publishedでJPEG公開の成功も記録。追加情報の公開例外はカウントして画像取得を止めない。

`stages`:

- read_start: OpenCV readを呼ぶ前の時計サンプル。
- read_end: readが返った直後の時計サンプル。待ち・USB転送・JPEG復号等を含み得る。
- preprocess_end: RGB変換とcrop/resizeが完了した時点の時計サンプル。
- backend_pos_msec: read後に取得したCAP_PROP_POS_MSECの生値。正の有限値以外・取得失敗はnull。

OpenCV versionと実際のbackend名も保存。V4L2のOpenCV実装ではCAP_PROP_POS_MSECは直近のv4l2_buffer.timestamp由来（https://github.com/opencv/opencv/blob/4.x/modules/videoio/src/cap_v4l.cpp）。以前の実機v4l2-ctlログでは `ts-monotonic, ts-src-soe` が報告されていたが、OpenCVからはバッファごとのflagsを取得していない。値だけから時計・SOE・精度を確定しない。別backendのPOS_MSECも同じ意味とは限らない。

LinuxのSOEは露光開始として定義される（https://www.kernel.org/doc/html/v4.10/media/uapi/v4l/buffer.html）。ただしフラグだけで実際の露光との絶対精度を保証できない。実機確認が済むまで既存headerを置き換えたり同期補正へ自動使用したりしない。

## MMS101

正のバイト数を受信したread単位に、read_start/read_end、read_seq、byte_offset、bytes_hex、parserのpending_before/after、破棄バイト数、startup_attemptを残す。生バイト列は将来の通信解析や未解釈フィールドの確認に使える。初期化コマンドの送受信やADC時刻そのものは含まない。

各readのsamplesは `[sample_seq, header_ns, packet_byte_offset]` の配列。複数readにまたがるpacketも、完成したreadに対応付け、全readをつないだ受信バイト列の絶対オフセットでpacketの先頭を示す。複数packetが同じreadで届けば、同じread_endを共有する。解析順をADC計測時刻と同一視しない。既存header_nsは従来どおりpacket解析時のROS時刻。

通番はホスト側で付与したもの。設定値configured_interval_us=1000はセンサが実際に欠落なく1000 Hzで送信した保証ではない。force_scale/torque_scaleも保存する。

取得スレッドでは時刻取得・コピー・短いキュー操作を行い、JSON化と公開は100 ms周期の別コールバックで行う。1回最大256 read、キュー上限4096 read。超過時は古いレコードを捨て、dropped_reads_total/dropped_samples_totalを明示。公開例外による消失も別カウント。DDS側でのメッセージ欠落はこれらに反映されないのでbatch/read/sampleの通番でも検出する。元のWrenchStampedはこのキューを経由しない。

batch公開のためメタデータは最大約100 ms以上遅れて届き得る。停止直前の未公開分やbag終了後に公開された分はbagに残らない。接触区間だけbag受信時刻で切ると関連メタデータが外側にある場合があるため、元session bagを保持する。後処理ではheader/byte offsetで対応付ける。今回、試行切り出しや収集終了待ちの動作は変更しない。

## 実機での最初の確認（ロボット動作不要）

修正をgit経由で反映後、ワークスペースでビルドする。起動中の対象センサノードは通常の方法で停止してから、新しい設定で再起動する。出力はすべて実行時のカレントディレクトリ内。設定やrawデータに変更を加える検証ではない。

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_gelsight_package mms101_driver data_capture_tools
source install/setup.bash
```

ターミナル1:

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros2_gelsight_package gelsight_publisher.launch.py acquisition_records:=true
```

ターミナル2:

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mms101_driver mms101_dual.launch.py acquisition_records:=true
```

ターミナル3:

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 src/ur3_robotiq_ros2/utils/diagnostics/record_acquisition_timing.py \
  --seconds 30 --out acquisition_timing_check_run1
```

`acquisition_timing_check_run1`が既にある場合はrun2等に変える（上書きしない）。まずsummary.jsonを共有すればよい。必要時にtiming_records.jsonlとsensor_headers.jsonで詳細解析できる。画像は保存せず、力覚値はpacketとの一致検査のため保存する。

確認項目: 4系統の時刻レコード受信、元headerとの一致、raw packetの復号値とWrenchStamped値の一致、段階時刻の順序、バッファ超過、時刻候補値の有無。受信レートは従来約18.75 Hz/約993 Hzとの比較用に出す。独立subscriberなので多少の受信差はあり得る。clock候補の差だけで物理同期の成功とは判定しない。

物理同期や接触を伴う負荷の評価はこの30秒確認とは別。今回のテストを通過した後、必要なら通常のAP 1試行を新メタデータ付きで記録する。

## ローカル検証と限界

ROS/cameraなしのPythonテスト11件、C++バッファ/JSON検査、実MMS101パーサをfake publisher/clockへ接続した分割/一括受信検査を実施。画像と既存headerの関係、JPEG失敗時の対応記録、64 bit整数、raw packet byte offsetと力変換を確認した。ROSを使ったノード全体のビルド、実機のCAP_PROP_POS_MSECの挙動、追加負荷はワークステーションで確認が必要。既存bagから新しい取得時刻は復元できない。


## 2026-10-15 初回実機確認を受けた検証スクリプトの修正

初回はschema/mappingエラー0、画像左右564枚ずつの対応、力覚左右29,323/29,317点のraw packetとの値一致を確認。取得側通番ベースの力覚レートは左右約993 Hz。GelSightのbackend時刻からread完了まで中央値約87 ms（露光の絶対遅延と確定しない）。

独立検証subscriberでは力覚271/459点が欠け、追加情報も左右1 batchずつ欠けた。取得ノードの記録バッファ超過・公開例外は0だった。従来の検証subscriberはsensor_data既定のdepth=5を全トピックに使い、callback内でJSON再エンコード・ファイル書き込みも行っていたため、この負荷に対して不適切だった。

検証用subscriberのみ、力覚BEST_EFFORT depth4096、画像BEST_EFFORT depth128、追加情報RELIABLE depth100に変更。JSONファイルへの書き込みを収集後へ移した。取得ノード・既存データ・headerは変更しない。source側の通番によるレート見積もりも追加し、callbackレートと区別する。これは原因候補に対する対策で、欠落解消は再確認が必要。

再確認は同じ30秒コマンドで `--out ./acquisition_timing_check_run2` とする。取得ノードは変更していないため、この修正だけなら再ビルド・センサ再起動は不要（git反映後のスクリプトを直接実行）。ロボット操作・APは不要。独立subscriberの欠落が解消しても、通常のbagに全記録があることは別途bagで確認する。
