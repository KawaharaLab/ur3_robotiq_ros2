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

同一起動でiを3回。収集ノードも通常終了してからグリッパドライバを通常終了・再起動し、同じ条件で収集ノードを起動し直してiを3回。計3起動×3回を目安とする。基準物・接触面・高さ・配線・取り付けは維持する。再起動の順番を変える検証は別条件として扱う。本データではreference_id/width/mount_idは指定不要。取り付け基準は自動init_idで識別する。

reference_id/width/mount_idは記録用ラベルであり、動作位置や換算式を変えない。通常initに使うconfigのtarget_diameterや事前アプローチ幅も基準物に適した設定にする。既存initの接触条件・安全条件を変更する機能は今回追加していない。

終了後、ワークスペース直下に小さな共有結果を作る。

```bash
/usr/bin/python3 src/ur3_robotiq_ros2/utils/diagnostics/summarize_init_records.py \
  --root ./init_records --out ./init_width_check_run1.json
```

まずJSONだけを共有すれば、起動ごとの接触count・力・tare・成否が確認できる。前後波形が必要なら対応するinit_idのsamples.jsonl.gzを追加で共有する。以前のログは消さず、reference_idやmount_idで条件を分ける。出力JSONの上書きはしないため、再実行は--outを別名にする。

## 再現性のための追加記録（schema 2）

今回も復帰方法・探索閾値・移動量・速度・mm換算は変更しない。既存のmm往復換算の不整合は未修正であり、記録の強化によって校正されたことにはならない。

### 接触位置と復帰後位置

最良状態を更新した時点の `selected_contact` を別保存する。選択時の判定値、観測したg_po/g_pr、元header/受信時刻、鮮度、推定幅とcountの換算値の一致、アーム関節状態を保存する。`contact`は従来どおり最終復帰後・開き直し前の状態。`selected_to_final_count_delta`に差を残す。これらは非同期の最新観測であって同時の物理測定ではない。`status_width_consistent`と`usable`を併せて確認する。

`record_quality_valid`はファイル保存・受信鮮度などの記録品質、`contact_criteria_met`は最終の左右接触・バランス条件を満たしたかを表す。後者は未成立でも既存のinitの受け入れ条件を変えない。`valid_for_collection`は後方互換のため残す記録品質フラグの別名で、接触成立の保証には使わない。

センタリングの左右エッジ、推定中心、相対アーム補正、選択時と最終の荷重、tare履歴、アーム関節header付き時系列も保存する。接触閾値や20サンプル移動平均の計算は変更しない。

### 物体ごとの手入力（YAMLに一本化）

通常は `data_capture.yaml` の `specimen_metadata` に任意の計測値を記入する。
`specimen_id` は省略時に `task_name` を使用する。同一物体のap/ad/av/af等は同じtask_nameで収集し、動作は各試行の記録で区別する。別IDが必要な場合だけspecimen_metadata内にspecimen_idを明示できる。

```yaml
task_name: object1
target_diameter: 0.010  # 動作計画用の調整値（m）
push_depth: 0.004       # 固定深さ動作の既定値（m）
specimen_metadata:
  specimen_width_mm: null
  width_measurement_state: null  # 例: クランプ固定前・無荷重
  clamp_gap_mm: null
  notes: ""
```

実測幅・クランプ幅・動作計画用target_diameterは独立した値で、自動相互変換や制御への反映はしない。push_depthは全試行の実適用深さを意味しない。深さ可変のadなどは試行イベントの目標幅と実指令を参照する。いずれも試料だけの実変形量を直接測定した値ではない。

取り付けの多少の揺れ、使用済み試料、未測定値を許容する。累積使用回数、mount_id、挿入深さ、正確な接触位置、写真の入力は必須にしない。取り付け変更後は通常のiを行い、実際の接触基準をinit_idで識別する。同一initを使ったデータが物理的に同一条件であるという保証ではない。notesに変形や取り付け変更などを必要に応じて記す。width_measurement_state、measurement_method、measurement_uncertainty_mm、prior_use、setup_photo、orientation、clamp_insertion_depth_mm、material_id、ambient_temperature_c等も任意で追加できる。未知はnullのままとし、日時・温度・寸法・不確かさは推測しない。

YAMLは動作設定と同じく収集ノード起動時のスナップショットを使う。物体変更時はYAML更新→収集ノード再起動→i→収集の順。設定編集だけのためにドライバを再起動する必要はない。起動後にファイルを書き換えても、そのノードで使用する設定・計測値は変わらない。保存されるconfig/effective_config/measurementsが使用した内容で、現時点ファイルのハッシュとは区別する。

旧 `specimen_metadata.example.json` は互換用のひな形であり、通常はコピーも編集も不要。作業ディレクトリに残っているspecimen_metadata.jsonは自動では読まない。既存JSONを使う場合だけROSパラメータ `specimen_metadata_file` にパスを明示する。この場合はそのJSONを計測情報の入力元とし、YAMLのspecimen_metadataと混ぜない（試料ID省略時のtask_name補完は共通）。明示したファイルの欠落・構文誤り・不正な数値はinit開始前に中断する。JSONは各init/収集準備時に再読込し、報告された寸法等がinit後に変わった場合は新しいinitを要求する。

init_reference_width_mmは校正用基準物の幅であり、試料幅とは別物。従来のinit_mount_idも互換用途で残すが、通常運用では指定不要。通常の取り付け基準の識別には自動init_idを使う。

### セッションと指令の記録

- `collection_provenance.json`: init ID、選択時count・最終基準count、接触判定、使用した計測情報の入力元・内容・ハッシュ、実際にロードした設定と既定値を含むeffective_config、設定ファイルの現時点ハッシュ、実行Pythonソースのハッシュ、参照可能なGit commit/追跡ファイルの変更状態、ホスト・OS・Python・許可リストに限定したROS環境変数、時計情報、換算式と制約を保存。Git情報はソースcheckoutの状態であり、実行中C++バイナリとの一致の証明ではない。
- `trial_conditions.jsonl`: START/END/ABORTのID・時刻・モード・イベント目標幅・速度・モデル換算count・その時点の観測ステータス。START/ENDではグリッパが開いている場合があり、これを押し込み後位置とは扱わない。
- `/robotiq_2f_gripper/command_event`: 通常ドライバがposition書込関数を呼ぶ前・戻った後・例外時の時刻と実際の指令count/速度count/駆動力count、指令元、起動ID、指令連番を記録。write_startはシリアルの最初のビットを測った時刻ではなく、write_returnも物理停止や到達の証明ではない。通信・制御命令そのものは元のままで、付加ログのpublish失敗で命令を変更しない。
- af/afsは通常ドライバを迂回するため、実際の指令・STOP・ステップの情報は従来のforce_press配下のplan/イベント/ステータスに記録する。

configにcommand_event、initialization、robot_cmdを追加する。ステータス・力・動作イベント・command_eventを記録しない設定や、更新前のドライバでは本収集を開始せず、設定/再起動の不足を表示する。独自configを使う場合は新しいtopicsを反映する。

MMS101とGelSightのacquisition_timingから、利用可能なstream ID・port・scale・設定周期・backend・OpenCV版・時計種別等を抜き出してスナップショット化する。既存のacquisition_records:=trueで有効になる。取得レコードを再配信することはなく、observerではメタデータの変更だけをinit履歴に保存する。未受信はnull、最終受信からの年齢は別記し、未受信を正常と推測しない。センサ本体の固有IDが通知されない場合は手入力IDを使う。完全な元パケット・画像・時系列は引き続きbag/acquisition_timingが一次情報。

### 反映と確認

YAMLへの入力統合のみの更新では `colcon build --packages-select data_capture_tools` を実施し、収集ノードを再起動する。前段のcommand_event対応ドライバをまだ反映していない場合だけ、robotiq_2f_gripper_hardwareもビルドし、静止中にドライバを再起動する。新規ポート設定は不要。既定のdata_capture.yamlもGitで反映する。YAMLの任意計測値を必要に応じて編集し、収集ノード再起動後にiを1回、通常収集を1回行い、init_recordsのsummary、セッションのcollection_provenance/init_reference/trial_conditions、およびcommand_eventの記録を確認する。まとめコマンドは従来と同じで、新旧init記録を読み、選択時countと最終基準countを区別して出力する。
