# 力覚閾値によるグリッパ停止の実験用プローブ

この機能は通常のinit/APに組み込んでいない。Linuxの研究室WSでMMS101の左右Fyを受け、RobotiqのrGTOを0にするModbus RTU指令を直接送る。力制御サーボやハードリアルタイム制御ではない。ホストのスケジューリング・ROS配信・シリアル通信・機器の減速による遅れと力の超過は残る。実機の成功は未確認。

## 変更内容

- 新規 `utils/diagnostics/force_stop_probe.py`。既定は読み取りのみ。`--enable-motion` と端末での `GO` 入力が揃った場合だけ、現在の位置から指定したraw countだけ閉じる。
- 標準のAction cancel経路を経由せずrACT=1/rGTO=0のSTOPを書き込む。位置指令のrGTO=1と区別する。自動で開く、リセット、アクティベーション、アーム移動は行わない。
- force判定は生Fyを1秒の基準区間中央値でtareした絶対値。既存initの20点移動平均は使用しない。既存init/APの処理や設定は変更しない。
- 初回試験は左右いずれかが閾値に達したら止める `--force-metric max` が既定。`mean`も選べるが、いずれの場合も片側のhard limitは独立に監視する。初期候補はthreshold=0.5 N / hard-limit=1.0 N。これらは機器の校正済み把持力設定ではない。
- 停止条件：力覚閾値、片側hard limit、センサ受信途絶/古いheader（既定100 ms）、無効な力値、相対位置上限、動作時間上限（既定1000 ms）、Enter、Ctrl+C/SIGTERM、通信/機器エラー。
- 最大追加閉じ量は `--close-ticks`（既定4、許容1..20）でハードウェアの位置目標にも設定する。tickは0..255の位置レジスタ単位であり、物体の押し込みmm・片側指変位ではない。上限に達して目標力未達なら、それ以上追い込まない。
- `--speed-byte 0` / `--force-byte 0` は機器の最低設定であり、速度0、力0 Nではない。5 mm/sや0.5 Nをこのレジスタで直接指定しているわけではない。
- STOP後は位置指令を追加せず最大4秒観測（`--hold-seconds`）。手動中断時は観測を早く終了する。停止ACKやgGTO=0の観測は、厳密な物理停止時刻や一定力保持の保証ではない。
- 新しい `deactivate_on_shutdown` パラメータ/launch引数を通常グリッパノードに追加。既定trueで従来の終了時deactivateを維持。試験への引き継ぎ時だけfalseにして、**静止した状態で**終了すれば有効状態と開き幅を引き継げる。falseは終了時の停止を追加する機能ではなく、運動中に終了してはいけない。

メーカー定義：rGTO=0がStop、rGTO=1がGo To。rACTは有効状態では1を維持する。
https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_TM_InstructionManual_20210623.pdf （4.3.1節）

## 試験の前提

- 同じシリアル回線へ同時にアクセスしない。プローブを実行する間は通常のグリッパノード、gripper_client、data_capture_managerを停止する。URアームのドライバー自体は本スクリプトで操作しない。
- MMS101左右のノードは起動したまま。通常の `/force_torque/left` と `/force_torque/right` を使う。GelSightはこの停止試験には必須ではない。
- 現在のノードのserial_port/baudrate/slave_addressを事前に確認する。`/dev/ttyUSB0`はlaunchの既定値にすぎず、実際の値を使う。
- /procで見える他プロセスの既存ポート利用、ROSの標準status publisherを検査し、TIOCEXCLで後からのポートopenを抑止する。ただし権限外プロセスや競合の完全排除を保証するものではなく、操作者による単独使用が前提。
- ソフトウェアがハングした場合やケーブルが切れた場合、Enter/Ctrl+Cで停止指令を届けられる保証はない。グリッパに実際に効く独立した停止手段を手元に用意する。URアームの停止が独立グリッパにも効くとは仮定しない。

## 1. 反映と接続設定の確認

gitで反映後、まだ起動している通常ノードから次を確認する。

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 param get /robotiq_2f_gripper_node serial_port
ros2 param get /robotiq_2f_gripper_node baudrate
ros2 param get /robotiq_2f_gripper_node slave_address
```

表示されたportを、この後に使う端末で設定する。**下記の値は例で、実際の表示値に置き換える。** baud/slaveが115200/9でなければ各プローブコマンドにも `--baud` / `--slave` を付ける。

```bash
FORCE_STOP_PORT=/dev/ttyUSB0
```

既存の通常グリッパノードを静止状態で終了し、物体を外した状態で更新版をビルドして起動する。通常の起動時アクティベーションは全ストロークの動きを伴う場合があるため、手と物体を離しておく。

```bash
colcon build --packages-select robotiq_2f_gripper_hardware
source install/setup.bash
ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_launch.py \
  serial_port:="$FORCE_STOP_PORT" deactivate_on_shutdown:=false
```

通常使っている他の接続引数（baudrate等）があれば同じものを付ける。起動して有効状態になった後、**グリッパが静止していることを確認してから**このlaunchをCtrl+Cで終了する。ここで初めて試験用のシリアル単独使用へ切り替える。毎回このhandoffをやり直す必要はなく、プローブは終了時にdeactivateしない。

## 2. 読み取りのみの接続確認

```bash
/usr/bin/python3 src/ur3_robotiq_ros2/utils/diagnostics/force_stop_probe.py \
  --port "$FORCE_STOP_PORT" --out ./force_stop_readonly_run1
```

このコマンドは動かさず、力覚受信と機器の状態を確認する。グリッパが非アクティブなら、物体を外して手順1の通常起動→静止handoffを行う。プローブはアクティベーションしない。

## 3. 最初は物体なしの時間指定STOP

```bash
/usr/bin/python3 src/ur3_robotiq_ros2/utils/diagnostics/force_stop_probe.py \
  --port "$FORCE_STOP_PORT" --enable-motion \
  --close-ticks 8 --stop-after-ms 20 --hold-seconds 0.5 \
  --out ./force_stop_empty_run1
```

端末で `GO` を入力すると1秒の無荷重基準を測ってから動き始める。20 msは移動要求の送信開始からのホスト経過時間であり、物理的に20 msで止まる保証ではない。位置目標は現在値+8に制限し、Enter/Ctrl+CでSTOP要求できる。機器のACK待ちやstatus読み取り中は通信の完了/タイムアウト（既定30 ms）までSTOP送信が遅れる場合がある。

summary.jsonでreason=timed_stop、stop_acknowledged=true、error=nullを確認。status.jsonl内のgGTO=0も確認する。gPOと実際の動きを見て、目標位置に先に到達しただけではないかを区別する。これは自動の物理停止精度判定ではない。まずこの結果を共有してSTOP経路を確認する。

## 4. 柔らかい物体での閾値STOP（無接触STOPを確認した後）

必要なら通常グリッパノードを同じhandoff設定で起動し、通常の操作で接触直前の幅へ調整して静止させ、再度handoffする。**基準計測時は無荷重とし、物体を既に強く挟んだ状態をtareしない。** 位置準備には物体に応じた判断が必要なため、本スクリプトは自動で近づけない。

```bash
/usr/bin/python3 src/ur3_robotiq_ros2/utils/diagnostics/force_stop_probe.py \
  --port "$FORCE_STOP_PORT" --enable-motion \
  --threshold-n 0.5 --hard-limit-n 1.0 --force-metric max \
  --close-ticks 4 --max-motion-ms 1000 --hold-seconds 4 \
  --out ./force_stop_soft_run1
```

GO後は無荷重基準を1秒測ってから、幅の上限まで閉じるか、力閾値で止める。最小移動量/停止遅れによるオーバーシュートはあり得る。0.5 Nを達成・保持する保証はない。raw単発値を使うため、配線由来のスパイクで早く止まる場合もある。

reason=force_thresholdの場合に初めて閾値STOPの試験となる。位置上限/時間上限等で止まった試行を、目標力達成とは扱わない。目的力へ届かないからといって自動で幅上限を拡大しない。

## 保存データと評価

出力は指定した**現在のディレクトリ内**。既存出力先は上書きしない。

- summary.json：設定、開始gPO、tare、停止理由、STOP ACK有無、callback停止判定→STOP書き込み開始/ACKまでの時間、判定後の力の最大値。
- force.jsonl：左右の受信した生6軸力覚、元header時刻、受信ROS/monotonic時刻。ROS subscriberは最新性優先depth1なので、全物理サンプルの保存は保証しない。
- events.jsonl：基準計測、移動要求、停止要求、ACK、gGTO=0を観測した時刻、異常。
- status.jsonl：Modbus statusの要求/応答時刻とgPO/gPR/gGTO/gOBJ等。

ROS受信スレッドとシリアル処理を分ける。受信スレッドは閾値判定をラッチし、シリアル所有スレッドがそれを見てSTOPを送る。制御中はファイル書き込みを行わず、終了時に保存する。力覚ログは上限200,000件で、超過を明示する。

STOP ACKはレジスタ書き込みの応答であり、完全停止の時刻ではない。statusは通常約20 Hz、位置も8 bitのため、実際の停止運動を細かく評価するには外部映像等が別途必要。MMS101内部の計測→配信の遅れも未校正。停止後の力の最大値・生波形と合わせて、実験に使えるかを判断する。

## 検証済みの範囲

ローカルで力覚判定、鮮度、ハード上限、手動停止、CRC、rACT/rGTOビット、擬似TTY応答/タイムアウト、擬似ROS全体フロー、移動ACK喪失後のSTOP試行、既定の終了時deactivate維持を検証した。実機停止、Linuxの実ポート、ROS Humble環境での動作・hardware package全体ビルドはWSでの確認が必要。

```bash
/usr/bin/python3 -m unittest discover \
  -s src/ur3_robotiq_ros2/utils/diagnostics -p 'test_force_stop_probe.py' -v
```

試験後に通常収集へ戻す場合はプローブの終了を確認し、通常のグリッパノードを起動する。通常運用ではdeactivate_on_shutdownを既定trueへ戻す。

## 通信タイムアウトを伴う引き渡し（2026-10-22修正）

`gFLT=0x09` は1秒以上通信がないことを示す。通常ノードの終了からプローブの起動までに発生し得る。接続直後の1回だけで異常終了せず、有効・初期化済み・運動中でない状態の0x09に限り、50 ms間隔で最長約1秒、状態を読み取って復帰を待つ（個々の通信は従来のタイムアウトで制限）。読み取りだけで解消するかは実機確認が必要。正常なgFLT=0を確認できなければ終了し、他の異常は待機せず終了する。自動reset/activate/STOP書き込みは追加していない。

GO入力待ちと1秒の基準計測中も約50 ms間隔で状態を読み取り、通信空白を避ける。ここで異常・非アクティブ・運動中を検知した場合も移動指令を出さない。`events.jsonl`に接続直後の状態と0x09復帰の観測を記録し、異常メッセージにgACT/gSTA/gFLTを表示する。

変更はPythonプローブ・テスト・この文書のみ。git反映後のcolcon再ビルドや通常ドライバの変更は不要。既存出力フォルダを避けて新しい出力先で読み取り試験を行う。
