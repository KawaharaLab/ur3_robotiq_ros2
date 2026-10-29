# 物体単位の自動収集と中断・再開

## 変更範囲

メニューac（新規）、ar（再開）、ai（計画位置でinitのみ）を追加する。既存i/ap/ad/ak等の動作条件は維持する。acは新しいPython runnerが通常グリッパActionを1つずつ送信し、run/kneadの目標幅・速度・駆動力・待機時間を既存C++と同じ値で実行する。自動runnerのイベント名はauto_*、TrialEvent.modeはauto_depth_speed/auto_aur/auto_knead。従来のphase番号だけで区間を切らず、イベントとautomatic_plan/progressを参照する。準備の全開は0.140mで既存runと同じ。完了後の1.5秒待機と0.8秒の試行間休止も維持し、設定で変更可能。各試行の無荷重tareと監視の追加により、厳密な所要時間は従来と同一ではない。

Armは従来どおり初期関節移動がROS trajectory、マクロ接近・センタリングがURScript。URScriptがExternal Controlを停止する挙動は変更しない。各init前に/dashboard_client/playを必要時だけ呼び、/io_and_status_controller/robot_program_runningを確認する。ポリスコープにExternal Controlのプログラムをロードしておく。別プログラムのload、保護停止の解除、電源/ブレーキ制御、headless化は行わない。play不可なら中断して手動再生後ar/aiを使う。仕様参考: https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/script_code.html

最初の実機検証前に完全無人運転の保証はない。接触画像の内容・接触の妥当性・クランプ緩み・センサ物理取り付け変更を自動判定するものではない。既存のアーム動作に対する経路計画/衝突検証を追加する変更でもない。

## 設定

全てdata_capture.yaml内automatic_collectionで指定。depths_mm/ loading_speedsは未確定のため初期値null。nullでは移動前に拒否する。深さは公称片側mm、速度は0..1の要求値で実速度mm/sではない。

例は条件表の書き方であり、この数値の全物体への適合を保証しない。

```yaml
automatic_collection:
  depths_mm: [1.0, 2.0, 3.0]  # 実験者が決めた値に置き換える
  loading_speeds: [0.02, 0.10, 0.20]
  repeats: 2
  offset_m: 0.010
  hold_s: 4.0
  release_speed: 0.3
  release_speeds: [0.02, 0.20]
  knead_conditions:
    - {shallow_mm: 1.0, deep_mm: 3.0, cycles: 5, loading_speed: 0.1, unloading_speed: 0.1, deep_hold_s: 0.1, shallow_hold_s: 0.1}
    - {shallow_mm: 1.0, deep_mm: 3.0, cycles: 5, loading_speed: 0.1, unloading_speed: 0.1, deep_hold_s: 0.5, shallow_hold_s: 0.5}
  hard_limit_n: 20.0
  rest_s: 0.8
  completion_settle_s: 1.5
  seed: 20261028
  batch_size: 10
  force_timeout_s: 0.25
  image_timeout_s: 0.5
  min_free_gib: 5.0
  require_timing: true
  auto_init: true
  auto_play: true
```

この例は4位置×(3深さ×3速度×2反復 + aur2速度×2反復 + ak2条件×2反復)=104試行。各反復内で動作条件をseed付きで混ぜる。基準位置は最初、残り3位置はseedで順序を固定する。aurはソートした深さの中央、入力速度リストの中央を使う。重複して同じ条件を含めた場合も独立したvariantとして記録されるため、不要な重複は設定で避ける。mm要求が同一countに換算される可能性は既存同様で、送信countはcommand_eventに保存する。

前後は既存_calculate_target_poseと同じ世界XY45度方向の±offset_m、上は世界Zの+offset_m。毎回同じ初期関節姿勢からマクロ接近するため、直前の補正位置への累積移動にはしない。各位置で通常initを実行する。自作指先や実物の接触面の向きがこの方向と一致するか、まず小さいoffsetで検証する。initのsummary.metadata.automatic_positionに要求offset/位置IDを保存する。

## 起動と初回確認

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robotiq_2f_gripper_interfaces robotiq_2f_gripper_hardware gripper_client data_capture_tools
source install/setup.bash
```

グリッパが静止した状態で、更新したグリッパドライバ・gripper_client・収集ノードを再起動。URドライバ、MMS101、GelSightは通常どおり。センサはacquisition_records:=true。新しいターミナルはROSとinstallをsourceする。ポート追加設定は不要。

移動せず条件表を確認:

```bash
/usr/bin/python3 src/ur3_robotiq_ros2/utils/diagnostics/preview_object_plan.py \
  --config src/ur3_robotiq_ros2/data_capture_tools/config/data_capture.yaml \
  --out ./object_plan_preview.json
```

通常のマネージャーを起動しac→表示した物体・試行数・進捗パスを確認→GO。一度のGOで全位置のinitと全試行を行う。Enterで一時停止。q/Ctrl+C終了時は可能な範囲でキャンセルと記録終了を試みる。Ctrl+CではROSを先にshutdownせず、最長55秒の後処理待機中もキャンセル応答を処理する。プロセス強制終了・PC停止ではこの後処理を保証しない。最初はdepths/speeds各1条件、repeats1、ak/aurなし、offsetを小さくして4試行＋4initを検証する。最初の位置でEnterによる停止→arによる再開を確認してから、センサ再起動の試験、最後に本番条件へ進む。本番条件に変える場合は別ジョブにする。

## 再開

中断したらセンサだけを再起動し、同じマネージャーでar→進捗パスはEnter→GO。物体・アーム・指・センサの取り付けを変えていないことを確認する。コードは物理的な付け替えを検出できない。

プロセスを再起動した場合もarから進捗JSONのパスを入力できる。YAMLの内容が元ジョブと同じ、グリッパの起動IDが同じ、アーム関節位置が記録した位置と全軸0.015 rad以内、状態・センサが新鮮、initファイルのチェックサムが正しいことが必要。センサstream IDは再開時に更新可能で、試行中の変更は中断する。センサの再起動でゼロ点が変わり得るため、試行開始時に既存と同じ全開→1秒待機→最新20サンプル平均のtareを行う。幾何学的initは再利用し、このtareと区別する。

YAML変更や別物体への交換は同じジョブとして再開しない。アーム位置変更/グリッパ再起動でinitが必要な場合はaiで現在の計画位置をinit→ar。auto_init:falseの場合も各位置でai→arを使う。aiは初期位置へのアーム移動を含む従来initなので、位置を手動復帰してから実行する必要はないが、通常initと同様に作業空間を確認する。ai/自動initにはDashboard playが使われ、auto_play:falseなら手動再生が必要。

途中再開は波形の途中からではなく試行の最初から。中断・記録不完全の試行は別attempt/別bagに残す。記録完了確認済み試行は飛ばす。正常終了した試行でも、そのbagの終了前にプロセスが落ちた場合は未検証として再実行する。元のbag/attemptは削除しない。

## 監視と中断

- 左右力覚の受信時刻/元header年齢・有限値、左右GelSightの受信/header年齢・約1秒ごとのJPEGデコード確認。
- 力覚受信コールバックで各受信サンプルのFy増分を確認し20 N以上をラッチ。購読depth=1の欠損やDDS/シリアル遅延は残り、物理力が20 N以下に収まる保証はない。既定はmax(abs(Fy_left-offset_left), abs(Fy_right-offset_right))。合力ではない。
- Gripper status/関節状態の更新、fault、意図しないアーム位置変化、グリッパ起動ID変更。URのrobot_mode=RUNNINGかつsafety_mode=NORMAL/REDUCED以外は中断。非常停止や保護停止を自動解除しない。
- acquisition_timingの受信とstream再起動、空き容量、rosbagプロセス終了、Action拒否/失敗/時間超過、手動中断。
- initではセンサ/状態と180秒の時間上限を監視。異常時はC++シーケンサstop（予約ステップ無効化＋Actionキャンセル）とURScript stopjを要求し、以後のinit指令を遮断する。通常initの移動平均/接触探索条件は変更しない。init開始前の大きい基準ずれを本番用の20N差分として評価しない。
- 本番はActionキャンセルを行い、下層ドライバでrGTO=0送信・ACK/echo確認してからキャンセル完了を返す。力上限の場合だけ、状態が健全で停止確認できた場合に開放する。それ以外はSTOPし、復帰後の次試行で開放する。通信断では停止/開放の送達を保証できない。

STOP-on-cancelは通常ドライバにも適用される。通常の順次動作の指令値は変わらないが、cancel/fault/timeout時は従来と異なりSTOPを送る。タイムアウトやACK不明の場合、次の指令を盲目的に送らない。応答不明が解消しない場合は状態を確認してドライバ/マネージャーを再起動し、aiで再initする。

## 保存と品質の意味

output_root/task_name/automatic_jobs/<jobID>.jsonに全条件・seed・位置・init参照・試行状態・全attemptをatomic/fsync保存する。セッションは<jobID>_<位置>_<UUID>で重複上書きせず、最大10試行ごとのbag。従来のcollection_provenance/initコピー/trial_conditionsに加えautomatic_plan.jsonに試行開始時のジョブを保存する。最新進捗はautomatic_jobsのJSONが正本。

試行はAction成功と待機/開放完了後もawaiting_bag。bag終了後、metadata.yamlに必須topicの正の件数があること、プロセス終了コード、ログのERROR/損失報告を確認してcompleteにする。手動確認なしで正常データを完全保証するものではない。全サンプルの欠損・header対応、露光/ADC時刻の同期、接触の物理的妥当性は別のQAが必要。収集時情報は元bag/acquisition_timingに保存し、自動時間補正しない。MMS101は1000Hz設定のまま。

## Enter中断とbagの端末入力

bag記録プロセスの標準入力はDEVNULLに接続し、収集ノードの端末を継承させない。
rosbagのキーボードハンドラによるEnterの読み取り競合・端末モード変更を避ける。
収集ターミナルのEnterはac/arの保持・移動待機中にも監視する。
bag自体へのスペースキーによる一時停止は使用せず、記録終了は従来どおりSIGINTで行う。
この変更はdata_capture_manager_10の共通bag起動処理に適用するため、同ノードのap等にも適用される。
トピック、圧縮、動作条件、保存先は変更しない。
実機では柔らかい試料・検証条件で保持中にEnterを1回押し、停止・メニュー復帰後、
arで同じ位置のinitを再利用して中断試行を再実行できることを確認する。
