# 実験用 af：力による停止・位置保持・開放

## 変更と範囲

`data_capture_manager_10` に `af` を追加する。既存の `i` / `ap` の動作、通常グリッパドライバの制御経路は変更しない。initで得た有効幅を使い、開始幅へ準備移動、無荷重基準計測、最低設定速度で閉じる、力で直接STOP、位置保持、開放を実行する。一定力を追従して保持する機能ではない。

実機で確認した `force_stop_probe.py` を共有して使用する。setup.pyが同じソースをパッケージshare/diagnosticsへインストールするため、Pythonプローブを変更した場合も収集用にはdata_capture_toolsの再ビルドが必要。単独プローブのCLIは従来どおり。ハードリアルタイムや物理停止時間の保証はない。

同じUSBシリアルを二重使用しないため、実験版ではinit後に通常グリッパドライバとgripper_clientを終了し、afに通信を引き渡す。収集マネージャーは終了しない。これによりinit結果がメモリ内に残る。afを複数試行しても引き渡しは一度でよい。

## 既定条件

| 項目 | 値 |
|---|---|
| 開始幅 | initの有効幅 + 2 mm |
| 最小幅 | max(1 mm, 有効幅 − 2 × config.push_depth) |
| 相対閉じ量上限 | 開始位置から20 raw count。最小幅との厳しい方 |
| 目標力 | 左右Fyの基準からの絶対増分の最大値が0.5 N |
| 目標の継続確認 | 同じ側の新しいサンプルで受信・header両方の時刻が5 ms以上継続、3サンプル以上。途中で目標の80%未満なら取消し、確定時は目標以上 |
| 過大値による中断 | 片側の生の増分が1 N以上。継続確認を待たない |
| 速度・駆動力レジスタ | 0 / 0。機器の最低値であり、速度ゼロ・力ゼロではない |
| 閉じ動作の時間上限 | 2000 ms |
| 保持 | STOP後4秒、位置保持。実測力は低下してよい |

幅と深さは既存2F-140ドライバの経験的変換で制限する。実物の押し込み量や指片側の変位を校正した値ではない。試行前に計算した開始幅・最小幅が表示される。

準備移動前にも現在位置で0.5秒の暫定基準を測り、準備移動中の力・鮮度・時間を監視する。準備移動の閉じ方向は最大40 count、各準備/開放動作は最大3秒。準備位置で0.3秒待機後、1秒の最終基準を測る。初期状態から圧縮されている試料を絶対無荷重として識別することはできない。init後の開始位置と試料の配置は操作者が確認する。

継続確認にはヒステリシスを用い、最初に0.5 Nへ達した後、0.4 N未満へ戻ると候補を取り消す。確定時も0.5 N以上である必要がある。これにより閾値近くの小さな揺れだけで確認時間が毎回リセットされることを避ける。

5 msの確認によって従来の単発判定より判定が遅れる。0.5 Nを瞬間的に超えること、実停止まで力がさらに増えることは許容して記録する。大きな単発ノイズは1 Nの中断条件で止まる場合がある。元の生データはフィルタしない。

## 実行手順

1. GitでWSへ反映し、WSのワークスペースで一度ビルドする。

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select data_capture_tools
source install/setup.bash
```

2. UR3・MMS101左右・GelSight左右を通常どおり起動する。initはGelSightのヘルスも確認するため必要。MMS101とGelSightはacquisition_records:=trueを維持する。新しく起動する各ターミナルでROSとinstall/setup.bashをsourceする。

3. 通常グリッパドライバだけは以下で起動する。既存ドライバがある場合は静止状態で終了してから。起動時の有効化で全ストローク動く可能性があるため手と試料を離す。

```bash
ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_launch.py \
  serial_port:=/dev/ttyUSB0 deactivate_on_shutdown:=false
```

4. gripper_clientと通常のdata_capture_manager_10を起動する。後者は更新したconfigを指定する。

```bash
ros2 run data_capture_tools data_capture_manager_10 --ros-args \
  -p config:=/home/tsumura/my_robotiq_ws/src/ur3_robotiq_ros2/data_capture_tools/config/data_capture.yaml
```

5. スポンジを普段の試料位置へ配置し、マネージャーで `i` を実行する。完了後、マネージャーは起動したまま、gripper_clientを終了し、静止したグリッパドライバを終了する。終了ログのLeaving gripper activated for handoffを確認する。falseは終了時のSTOPを追加する設定ではない。

6. 同じマネージャーで `af` を入力する。初回は試行数 `1`、表示条件を確認して `GO`。以後は自動準備→記録→基準計測→閉じる→STOP→保持→開放。試行中Enterで中断、Ctrl+Cで終了とSTOP要求。通信断やプロセス停止時の送達保証はないため、グリッパに実際に効く停止手段を用意する。

目標未達、力のhard limit、手動停止、通信・鮮度異常の場合はABORTとしてシリーズを終了し、自動で開放・再試行しない。操作者が状態を確認してから再開する。bag開始失敗時は動作しない。bagプロセスが終了した場合も子プロセスへ中断を要求する。

APや再initへ戻る場合は通常グリッパドライバとgripper_clientを起動し直す。通常運用ではdeactivate_on_shutdownの既定trueに戻す。

## 保存内容と意味

10試行ごとにbagを分ける。保存場所は従来のconfig.output_root/task_name/日時。

- bag：既存の力・GelSight・取得時刻情報に加え、TrialEventと `/force_press_event`（std_msgs/String JSON）。afをAPと区別するmodeは `force_press_hold`。
- `force_press_metadata.json`：initの有効幅、条件、実際の試行順、成功/中断と結果ファイルの相対パス。
- `force_press/trial_0001_plan.json`：その試行の全設定。
- `force_press/trial_0001/{summary.json,events.jsonl,status.jsonl,force.jsonl}`：基準、各段階の時刻、STOP理由、送信/応答、位置、元headerとホスト受信時刻、生6軸力覚。

START〜ENDは準備・基準・閉じ・保持・開放を含む。ABORTは不成功。解析では個別イベントのbaseline_start、baseline_ready、motion_write_start、stop_write_start、stop_write_ack、hold_start、hold_complete、release_start、collection_completeで区間を定義する。過去のAPと同じphase番号を仮定しない。gPOを含むstatusイベントも記録する。通常ドライバを止めている間は、workerが同じ既存変換で `/robotiq_2f_gripper/finger_distance_mm` と `/robotiq_2f_gripper/status` も発行する。statusのheaderはホストが読み取り完了後に付けた時刻であり、モータ内部の計測時刻ではない。

hold_complete→release_startの前に異常があれば試行は不成功。正常な力停止、保持、開放のすべてを完了した場合のみENDとする。力停止の理由が残っていても、その後の異常を成功と扱わない。

STOP応答は物理停止ではない。位置の取得はおおむね20 Hz、8bit位置レジスタである。力はADC時刻未校正であり、元bag、取得時刻情報、試行別ログを併せて保存する。既存の画像や力のheader定義を変更したり、自動的に時間補正したりしない。

## 設定変更

マネージャーの起動時ROSパラメータ `force_press_port`、`force_press_baud`、`force_press_slave`、`force_press_threshold_n`、`force_press_hard_limit_n`、`force_press_hold_seconds`、`force_press_clearance_m`、`force_press_max_depth_per_side_m`、`force_press_close_ticks`、`force_press_max_motion_ms`、`force_press_target_confirm_ms` を使用する。通常init/APのconfig値とは別で、max_depth_per_side_mの既定だけconfig.push_depthを引き継ぐ。深さ・幅・力の変更は実行前の表示とplanにも記録される。
