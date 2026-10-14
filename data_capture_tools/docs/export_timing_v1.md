# 変換時刻索引 v1（2026-10-14）

## 変更の範囲

bag→PNG/CSV変換に`timing_index.csv`と`export_manifest.json`を追加した。
既存の画像名、CSV列と値、`stamp_ns`の意味（bag受信時刻）、画像の加工内容、試行区間の選択は維持する。
取得ノード、header生成、init、FTの1,000 Hz設定、制御用フィルタは変更していない。
これは時刻情報の保持であり、物理的な同期精度が改善・校正されたという変更ではない。

## timing_index.csv

選択されたメッセージごとに1行を持つ。

|列|意味|
|---|---|
|topic|bag内のトピック名|
|topic_message_index|今回選択された同一topic内での1始まりの番号。機器のsequenceではない|
|bag_receive_ns|bag readerが返す受信時刻。整数ns|
|source_stamp_ns|メッセージのトップレベルheader時刻。整数ns。未取得は空欄|
|source_stamp_semantics|`publisher_header`、`unavailable`、`zero_header`、`invalid_header`|
|frame_id|元headerのframe_id|
|artifact_path|出力先を基準にした画像またはCSVの相対パス|
|artifact_row|CSV内のデータ行番号。ヘッダ行を除いて1始まり。画像は空欄|
|export_status / error|書込・復号等の結果|

headerのないグリッパFloat32やlegacy markerは、bag時刻でsourceを埋めない。
TFMessageのような複数headerを持つ型も、勝手に先頭のheaderを全体のsource時刻にしない。
`publisher_header`は露光やADC時刻の保証ではない。時刻の生成箇所はpublisher実装を確認する。
時刻列をpandas等で読み込む際は、欠損のある整数をfloatへ変換しないよう、文字列またはnullable Int64で読む。

## export_manifest.json

- `in_progress`、`failed`、`complete`を区別する。
- 設定ファイルのSHA-256、実際に使用したtopic設定、元bagパス、bag受信時刻による抽出範囲を記録する。
- topic別件数、0件の設定topic、変換失敗件数、出力サイズと索引のSHA-256を記録する。
- `complete`は選択データの変換処理の完了を意味する。センサーの全サンプル保存、必須topicの充足、物理同期の保証ではない。
- 元bag自体の全内容のハッシュは計算しない。元bagを同じパスで差し替えた場合には別の出力先を使う。

画像書込失敗、同一出力名となる画像の重複、復号失敗を含む変換は成功扱いしない。
途中の索引・成果物は調査用に残るが、manifestで失敗と区別できる。
新しいバッチ抽出は、同じ入力条件で正常完了し、索引ハッシュと出力サイズが一致する試行だけをskipする。

## 互換性上の注意

旧試行フォルダや、失敗・未検証の試行フォルダをバッチ抽出が自動上書きすることはない。
存在する場合はエラーを返す。旧データは削除せず、別の作業場所へ元bagをコピーして再抽出する。
既存trialフォルダが1個あるだけでbag全体を完了とみなす処理は廃止した。

`convert_bag_to_dataset`と`convert_trial_range`を直接呼ぶ場合には、従来どおり出力先を指定できる。
この2関数自体は既存出力の上書きを全面禁止していないため、比較試験では必ず新規出力先を使う。
新規抽出でもtrial区間は従来のlegacy markerとbag受信時刻で決まる。時刻付きtrial/actionイベントへ移行する変更は今回含めていない。

旧PNGから元の画像header時刻は復元できない。元bagがあれば再抽出できる。
既存の`_calibrated_smooth.csv`は入力bagの出力とは別の派生データで、この索引は自動的に付与しない。

## ローカルの検証

ROS依存部分を模擬したテストで、両変換経路、旧CSV/画像名の維持、ns整数の保持、header欠損、列マップ、書込失敗、復号失敗、完了検証を確認する。
これは実ROSメッセージ・rosbagライブラリ・画像codecによる統合試験の代替ではない。

```bash
PYTHONPATH=data_capture_tools python3 -m unittest discover \
  -s data_capture_tools/test -p test_export_timing.py -v
```

## 研究室での確認

1. 元bagと既存trialを保存し、比較用の別ディレクトリへbagをコピーする。
2. 研究室ワークスペースで`data_capture_tools`を再ビルドしてsourceする。
3. コピーしたbagを`extract_batch_bag`で新規抽出する。
4. 旧版と新版で画像枚数・画像内容・CSV列と値が一致することを確認する。
5. 索引の画像source時刻を元bagのheaderと照合し、manifestがcompleteであることを確認する。
6. 欠落topic、legacy区間選択、元headerの意味は別途確認し、completeだけで同期品質を判断しない。
