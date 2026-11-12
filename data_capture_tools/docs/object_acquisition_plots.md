# extract_all出力の波形・画像比較

`plot_object_acquisition.py` は、`extract_all --write` で展開した物体別フォルダを入力にする。エクスポート済みのJPEG・CSV・manifestから、試行別PNGと同条件の比較PNGを作成する。元bagの読み出しやbagの解凍は行わない。

## 実行例

`object7` は実際の物体フォルダ名に置き換える。最初は3試行で確認する。

```bash
cd /home/tsumura/my_robotiq_ws
/usr/bin/python3 src/ur3_robotiq_ros2/utils/diagnostics/plot_object_acquisition.py \
  --export-dir /mnt/nvme1/tsumura/masters_thesis/exports/object7 \
  --out ./object7_qc_plots_smoke_20261112 \
  --max-trials 3
```

全件の描画では `--max-trials` を外し、新しい出力先を指定する。

```bash
/usr/bin/python3 src/ur3_robotiq_ros2/utils/diagnostics/plot_object_acquisition.py \
  --export-dir /mnt/nvme1/tsumura/masters_thesis/exports/object7 \
  --out ./object7_qc_plots_20261112
```

入力には、エクスポート先の `report.json` や `source_progress.json` があるフォルダを指定する。セッション単体の `manifest.json` があるフォルダも指定できる。`trial_1` を探して分割する方式には依存せず、manifestにある試行IDと開始・終了時刻で選択する。

プロットはソースを直接実行するため、colcon buildやROSノードの起動は不要。NumPy、Matplotlib、OpenCVを使う。JPEGをメモリ上で読み込んで描画するため、実行はデータ収集後に行う。

## 既存の展開済みデータ

すでに `extract_all` で展開した画像・力覚CSVをそのまま利用できる。解凍のやり直しは不要。

旧出力にはグリッパ状態・動作イベントCSVがないため、そのパネルには取得できない旨を表示する。力覚、GelSight差分、代表画像、同条件比較は生成できる。動作記録がない場合は実際の動作時刻を再構成せず、解放指令に依存する離脱時刻の推定も省く。

今後の `extract_all` では次の小さなCSVも追加する。

| ファイル | 内容 |
|---|---|
| `robot_action_event.csv` | 実際の指令・成功・保持イベント、Header時刻、目標幅・速度 |
| `gripper_width.csv` | 記録された幅mm、bag受信時刻。Header欄は空欄 |
| `gripper_status.csv` | Header時刻、raw入力レジスタ、読み取り時間 |

これらの追加を今後のextract_allへ反映するには、pull後に一度 `colcon build --packages-select data_capture_tools` と `source install/setup.bash` を実行する。既存のJPEG・力覚CSVの内容や試行範囲は維持する。補助CSVはSTART直前最大0.5秒からENDまでを対象とする。

## 出力

- `trials/物体/press/条件/` と `trials/物体/knead/条件/` に1試行につき1枚のPNG。
- `comparisons/物体/ジョブ/press/` と `comparisons/物体/ジョブ/knead/` に同条件の比較PNG。各行が位置、左列が力覚、右列がGelSight差分。同一試行は同じ色、左センサは実線、右センサは破線。
- `trial_plot_index.csv` に試行ID、位置、条件ID、画像パス、接触・離脱推定時刻、注意点。
- `report.json` に生成結果、条件、収集時のtare、スパイク候補数、CSV・画像の読み取りエラー。
- 入力に `source_progress.json` があれば、そのコピー。

出力先は既存フォルダを上書きしない。PNG名にはジョブIDと位置を含む試行IDを使う。同条件の比較は物体・ジョブごとに分け、異なる日の別ジョブは自動で混ぜない。深さ・速度・保持・開放速度・揉み回数・開き幅などで条件を識別する。

`plots_generated` は図の生成完了を表す。データ品質の合否には既存の厳格チェックを併用する。読み取れないデータや未完了のエクスポートが含まれる場合は `incomplete` として終了コード1を返す。

## 図の見方

試行別PNGは次の4段。

1. 左右のFy。収集時のtareを引いた符号付き波形を既定とし、全サンプルを平滑化せず表示。単発スパイク候補は赤い×。
2. 左右GelSightの平均絶対画像差。56×56のグレースケール画像を使う。
3. グリッパ幅、指令幅、raw g_po。mm値はドライバの公称換算値。Headerのない幅mmにはbag受信時刻を使い、その他はHeader時刻を使う。
4. 左右それぞれの基準フレーム、最大絶対Fy付近、終了時の画像。

横軸0秒はTrialEvent START。整数nsの差を取ってから秒へ変換し、実際の経過秒を保つ。波形の時間正規化は行わない。力覚の5 ms超、画像の100 ms超の隙間では線を途切れさせる。

青・黄の背景は記録された指令の実行区間と保持区間。紫の線は力覚閾値から推定した接触・離脱時刻。物理的な正解ラベルとして利用するには、別途妥当性の確認が必要。

## 基準値と欠落

力覚はmanifestに保存された収集時のtareを使う。欠けている場合は先頭区間から勝手に再推定せず、raw Fyと注意書きを表示する。CSV内の基準値・相対Fyとmanifestに不一致がある場合はエラーとして記録する。

GelSightは、あればSTART直前の最後の最大5枚の中央値を基準にする。現在のextract_allの画像・力覚CSVはSTART〜ENDだけなので、通常はその直前区間がない。次に力覚による接触推定より前の最大5枚を基準にし、それも得られなければ最初の最大5枚を使う。どの基準を使ったかは注記する。基準区間の代表画像は、中央値を作ったフレーム群の最後の1枚。

接触判定は、無荷重基準からのFy変化が閾値を30 ms継続して超える時刻を使う。閾値は直前区間のサンプルがあれば `max(0.05 N, MAD由来sigma×6)`。現在の展開済みデータでは直前区間がないため0.05 Nを使う。離脱判定では解放直前の力の10%も考慮する。

この図は通常のWrenchと記録されたHeader時刻で描画する。生パケットによるWrench欠落の補完や、追加時刻記録を用いた時刻補正は別処理として残る。元bag、元JPEG、CSV、manifestは変更しない。

## フィルタ

| 追加する引数 | 用途 |
|---|---|
| `--position base` | 基準位置だけ |
| `--trial-id base_0001 --trial-id up_0009` | 指定試行だけ |
| `--absolute-force` | 添付コードと同様に絶対値の力を表示 |
