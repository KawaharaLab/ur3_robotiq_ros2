# initの位置・力・初期化履歴の記録

## 変更範囲

`data_capture_manager_10` の `i` を記録する。通常の探索・センタリング・tare・開き直し・移動速度・力閾値・mm換算式は変更しない。グリッパドライバは既存のアクティベーション条件を保ち、実施有無を起動ごとに保持配信する。記録に使えるinitがない場合はAP/af用の接触基準を無効にする。

## 出力

既定では起動時の作業ディレクトリの `init_records/<日時_UUID>/` に保存する。ホームディレクトリなどへ暗黙に出力しない。保存場所はROSパラメータ `init_record_dir` で絶対パス指定できる。実行ごとに固有IDを付け、以前の結果を上書きしない。

- `summary.json`: 成否、記録品質、接触位置、移動平均Fy・tare、基準物の情報、ドライバ起動情報、設定・コードのハッシュ。
- `samples.jsonl.gz`: 観測した左右の6軸力、Modbus由来ステータス、推定幅、tare履歴、動作イベント、robot_cmd、接触確定マーカー。

開始時点でin_progressのsummaryを作り、正常な記録終了時にfinishedへ更新する。強制終了で残ったin_progress記録は成功として扱わない。コールバックでは有界キューへの追加だけを行い、圧縮・書き込みは別スレッドで行う。キュー超過・書き込み失敗・終了待ちタイムアウトは明示し、そのinitは再利用しない。力の購読は既存の最新性優先depth=1のままで、1000 Hz全サンプルの無欠損保存は保証しない。

接触スナップショットはセンタリング終了後、最後の開き直しの直前に採取する。左右力・位置の最新値は非同期であり、単一の物理時刻を表さない。それぞれのheader、受信ROS/monotonic時刻、受信経過時間、headerの受信時年齢を保存する。位置はg_po/g_prのcountを一次情報とし、mmは未校正の既存換算値として区別する。接触時の基準値はinit中の最後のtareと20サンプルの移動平均。centering_resultには最後の力が既存の左右接触・バランス条件を満たしていたかも記録するが、探索条件や戻り方は変更しない。

## 再利用とセッションとの対応

新しいinit開始時に以前の成功結果を無効化する。失敗・例外時は履歴を残すが、旧結果へ戻さない。接触時に位置/左右力が欠落、受信後250 ms超、header受信年齢が-10〜250 msの範囲外、無効な力、グリッパfaultの場合、動作が完了してもvalid_for_collection=falseにする。250 msはログ参照の鮮度基準であり、制御停止時間や物理同期の許容誤差ではない。

記録が成功した直近のinitは、収集セッション作成時に `init/<init_id>/` へコピーし、`init_reference.json` に相対パスを保存する。AP/af/afsなど通常のセッション準備経路に共通して適用する。initを要求しない旧来の単発機能ではavailable=falseも明示できる。元ログのハッシュ不一致やコピー失敗ではセッション準備を中断する。

## ドライバの起動履歴

`/robotiq_2f_gripper/initialization` (std_msgs/String、reliable/transient_local、depth=1) に、起動固有ID、ROS/steady時刻、port/baud/slave、起動時にアクティブだったか、今回activateを実施したか、終了時deactivateの設定、換算モデルIDを保持配信する。アクティベーションによる実距離のずれを測定したものではない。

収集ノード起動が後でもこの通知を受け取れる。init実行後に別の起動IDを受信した場合、結果を無効化して再initを要求する。init開始後まで初回通知の発見が遅れた場合も安全側に無効化する。通知を受け取れない旧ドライバはactivation_history_known=falseと記録し、実施有無を推測しない。

## 今回の検証手順

反映後、静止状態で収集ノードとグリッパドライバを通常終了する。他の動作が進行中でないことを確認してビルドする。

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robotiq_2f_gripper_hardware data_capture_tools
source install/setup.bash
```

グリッパドライバは通常のlaunchで起動する。収集ノードは各ターミナルでsourceを行ってから、基準物の実測幅とIDを設定して起動する。次は実測20.00 mmだった場合の例で、実際の測定値に置き換える。

```bash
ros2 run data_capture_tools data_capture_manager_10 --ros-args \
  -p config:=/home/tsumura/my_robotiq_ws/src/ur3_robotiq_ros2/data_capture_tools/config/data_capture.yaml \
  -p init_record_dir:=/home/tsumura/my_robotiq_ws/init_records \
  -p init_reference_id:=rigid_reference_A \
  -p init_reference_width_mm:=20.00 \
  -p init_mount_id:=gelsight_finger_v1
```

同一起動でiを3回。収集ノードも通常終了してからグリッパドライバを通常終了・再起動し、同じ条件で収集ノードを起動し直してiを3回。計3起動×3回を目安とする。基準物・接触面・高さ・配線・取り付けは維持する。再起動の順番を変える検証は別条件として扱う。本データではreference_id/widthは省略し、必要なmount_idだけ指定できる。

reference_id/width/mount_idは記録用ラベルであり、動作位置や換算式を変えない。通常initに使うconfigのtarget_diameterや事前アプローチ幅も基準物に適した設定にする。既存initの接触条件・安全条件を変更する機能は今回追加していない。

終了後、ワークスペース直下に小さな共有結果を作る。

```bash
/usr/bin/python3 src/ur3_robotiq_ros2/utils/diagnostics/summarize_init_records.py \
  --root ./init_records --out ./init_width_check_run1.json
```

まずJSONだけを共有すれば、起動ごとの接触count・力・tare・成否が確認できる。前後波形が必要なら対応するinit_idのsamples.jsonl.gzを追加で共有する。以前のログは消さず、reference_idやmount_idで条件を分ける。出力JSONの上書きはしないため、再実行は--outを別名にする。
