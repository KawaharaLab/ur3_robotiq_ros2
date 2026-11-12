# 収集済みデータの一括処理と実行履歴

`utils/diagnostics/process_acquisition_batch.py`は、物体ごとに厳密チェック → extract_allと同じ処理による展開 → 比較プロットを順番に実行します。ロボット操作は行いません。元bag・進捗JSON・既存出力は変更しません。

## 保存場所の役割

```text
masters_thesis/
├── data/                               元データ
├── exports/object7/                    展開結果
├── quality_check/object7/              品質チェック結果
├── plots/object7/
│   ├── trials/*.png                    試行別プロット
│   ├── comparisons/*.png               条件別の比較
│   ├── report.json
│   └── trial_plot_index.csv
└── postprocess/
    ├── index.html                      全実行履歴：物体名・日付・ジョブIDで検索
    ├── history.csv                     検索用の履歴一覧
    ├── runs/                           履歴索引の元データ
    ├── 20261112_220000_a1b2c3/          ある回の実行記録
    │   ├── summary.html
    │   ├── summary.txt
    │   ├── review_trials.csv
    │   ├── batch_state.json
    │   └── objects/                    進捗コピー・各工程のログ
    └── 20261113_220000_d4e5f6/          次の回：前回分を上書きしない
```

確認用の図・チェック結果は物体名で探し、処理の実行履歴は日時で探します。「処理日時」と「元の収集ジョブID」は別々に記録します。history.csvには各工程の実行／再利用、保存先、結果を最初に作成した実行IDも含めます。

`plots`と`quality_check`の既定位置は`--data-root`の親フォルダ直下です。`--plots-root`・`--quality-root`で変更できます。履歴の親は`--history-root`で変更でき、既定は同じ親の`postprocess`です。実行ごとの出力先は自動的に日時＋識別子で作成します。`--out`で明示指定も可能ですが、指定先が既にある場合は再開以外では拒否します。

## 通常実行：データを追加した後も同じコマンド

最新版を反映してから実行します。今回の入口はソースを直接使うのでcolcon buildは不要です。既存チェック用にROS環境を読み込みます。Pythonにはnumpy、matplotlib、OpenCV、PyYAMLが必要です。

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
POSTPROCESS_LOG="./postprocess_$(date +%Y%m%d_%H%M%S).log"
nohup /usr/bin/python3 src/ur3_robotiq_ros2/utils/diagnostics/process_acquisition_batch.py \
  --data-root /mnt/nvme1/tsumura/masters_thesis/data \
  --exports-root /mnt/nvme1/tsumura/masters_thesis/exports \
  > "$POSTPROCESS_LOG" 2>&1 < /dev/null &
echo "$POSTPROCESS_LOG"
```

SSH切断後も継続します。ワークステーション本体の自動サスペンドは無効にしてください。全bag読込み・圧縮bagの一時展開・全JPEG検査・描画でCPUとディスクを使うため、収集を止めた時間帯での実行を勧めます。今回作成した工程ログは日時別の実行フォルダ内に残り、端末の全体ログも上記の日時別ファイルに残ります。

- 毎回、各物体の`automatic_jobs`内で更新時刻が最新の進捗JSONを選び直します。新しい物体も対象になります。最新ジョブが未完了ならスキップし、古い完了ジョブには切り替えません。
- 以前の一括処理の履歴を参照し、対象進捗・処理コード・元データのサイズと更新時刻等が一致し、結果ファイルが揃う工程を再利用します。品質チェックのREVIEWも再利用可能です。FAILや実行エラーは再試行します。
- 元データやコードが変わった工程、未処理の物体は新しく処理します。既存の物体別出力を上書きせず、必要な場合だけ`object7_002`などの別名を作ります。今回の保存先は履歴一覧から開けます。
- `--objects object1 object7`を追加すると対象を絞れます。
- `--skip-quality`は全対象の厳密チェックを省略します。初回は省略しないことを推奨します。
- `--reexport`は新しい実行では過去の展開を使わず新規展開します。その同じ実行の再開時は、自身が完了させた展開を再利用します。
- 同じ履歴フォルダで一括処理を同時起動することは拒否します。

### 既存データとの混在

指定した`--exports-root`にあるextract_allの完了出力は、対応report.json、進捗ハッシュ、試行ID、記録されたファイルサイズ等で確認して再利用します。再利用できない場合は`exports/物体名/収集ジョブID`以下に新規展開します。JPEGは元の圧縮データをそのまま取り出します。全画像をPNGに変換する処理ではありません。

個別実行したチェックやプロットは、今回の一括処理の履歴情報がないため自動で再利用しません。以前の版の一括処理については、履歴の親直下または一段下にbatch_state.jsonがある場合、自動で履歴へ登録します。別の場所にある以前の実行は、`--out 以前の実行フォルダ --resume`で再開すると登録されます。処理コード変更等により再実行が必要になることはあります。既存フォルダを自動移動することはありません。

古い展開にグリッパ幅・指令CSVがない場合も再利用します。欠けている補助パネルは明示し、力覚・GelSightは描画します。補助CSVも取り出したい回は`--reexport`を使ってください。

## 履歴を探す

まず`postprocess/index.html`を開きます。検索欄に`object7`、`2026-11-12`、収集ジョブIDなどを入力すると、対応する実行・結果・summaryに移動できます。複数語は空白で区切り、すべてに一致する行を表示します。表にあるのは処理日であり、収集日は収集ジョブID・元進捗JSONから区別してください。

ターミナルでも検索できます。この操作は処理を実行しません。

```bash
cd /home/tsumura/my_robotiq_ws
/usr/bin/python3 src/ur3_robotiq_ros2/utils/diagnostics/process_acquisition_batch.py \
  --history-root /mnt/nvme1/tsumura/masters_thesis/postprocess \
  --find object7
```

日付も絞る場合は`--find object7 2026-11-12`とします。再利用した物体も今回の履歴に記録されるため、同一物体の検索結果は複数行になり得ます。新しく計算したかどうかは各工程の「実行／再利用」を確認してください。MacでHTMLを見る場合は、postprocess・plots・quality_checkを同じ親フォルダへコピーすると結果間の相対リンクを維持できます。

## 品質チェックをどう使うか

各実行のsummary.htmlで処理結果と品質結果を確認し、review_trials.csvで全指摘と対応PNGを確認します。HTMLは代表例のみです。

|分類|用途・対応|
|---|---|
|優先確認|破損、JPEG不正、未回収欠落、値不一致など。該当試行を学習候補から一時保留し、詳細確認。全物体の取り直しを直ちに意味しない|
|波形確認|スパイクなどの疑い。対応PNGで接触前後・左右差・継続時間を確認し、後処理の対象を決める|
|時刻確認|Header間隔・遅延・フレーム系列の空きなど。追加記録と照合し、Headerの空きだけで実サンプル欠落とは断定しない|
|補完情報|ローカル時刻記録・生パケットの補完候補。学習前に補完要否を確認。展開処理が自動補完するわけではない|

既存の厳密チェックの閾値は変えていません。REVIEWは要確認であり、破損・取り直しの自動判定ではありません。PASS_WITH_NOTESにも補完が必要な内容が含まれます。プロット生成成功は品質合格を意味しません。ある物体に要確認や失敗があっても、後続物体は処理します。

## 中断した実行を再開する

「新しいデータも検索する通常実行」と「中断した対象だけを再開する操作」は別です。再開には元の実行フォルダを明示します。省略したまま`--resume`は使えません。

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
POSTPROCESS_LOG="./postprocess_resume_$(date +%Y%m%d_%H%M%S).log"
nohup /usr/bin/python3 src/ur3_robotiq_ros2/utils/diagnostics/process_acquisition_batch.py \
  --data-root /mnt/nvme1/tsumura/masters_thesis/data \
  --exports-root /mnt/nvme1/tsumura/masters_thesis/exports \
  --out /mnt/nvme1/tsumura/masters_thesis/postprocess/20261112_220000_a1b2c3 \
  --resume > "$POSTPROCESS_LOG" 2>&1 < /dev/null &
```

完了工程は条件が一致すれば再利用します。失敗した展開・プロットは別出力先で工程単位でやり直すため、途中出力が残る場合があります。再開はbag内の途中メッセージ単位ではありません。変更検出は完全な全ファイル内容ハッシュではなく、同サイズ・同更新時刻の意図的な改変まで検出する保証はありません。元bagとローカル時刻記録は展開後も保管してください。
