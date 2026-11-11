# ac開始時の手首RealSense写真

`ac` のGO確認後、アームやグリッパの自動動作を開始する前に、手首RealSenseのカラー画像を1枚保存する。現在のアーム姿勢で撮影するため、init前のカメラが物体を向いているかを最初に確認する。画像は試料・取り付け状態の参考資料として扱う。

既存の `realsense2_camera_node` を専用namespaceで一時起動し、最初のフレームから2秒間を捨て、その後の1フレームをJPEG品質95で保存する。撮影後はカメラプロセスを終了させてから収集へ進む。カラーのみ640×480・15 Hzを要求し、深度・赤外・IMUを無効にする。準備中には複数フレームを受信するが、保存する画像は1枚。

カメラの選択には、既存の `realsense_dual.launch.py` で手首用として使っているシリアル番号 `143122066280` を既定値として使う。Python版SDKの追加導入は不要。手首カメラを別プロセスが使用している場合は、撮影に失敗する可能性がある。既存プロセスは操作せず、今回の一時プロセスだけを終了する。

## 反映と短い確認

進行中の収集が完了してから変更を反映し、収集ノードを再起動する。

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select data_capture_tools
source install/setup.bash

/usr/bin/python3 -m data_capture_tools.context_snapshot \
  --out ./wrist_context_check_run1
```

ロボットの動作やデータ収集を開始せずに写真だけ確認できる。`wrist_context_check_run1/wrist.jpg` に物体が写っているか確認する。撮影の詳細は同じ場所の `metadata.json` と `camera.log` に保存する。再実行時は新しい出力名を指定する。撮影機能を組み込んだ収集ノードは、新しくsourceしたターミナルで起動する。

## 保存場所と再開

通常の収集時は以下に保存する。

```text
<output_root>/<task_name>/automatic_jobs/
  <job_id>.json
  <job_id>_context/
    capture_<識別子>/
      wrist.jpg
      metadata.json
      camera.log
      capture.log
```

進捗JSONの `context_snapshot` に画像パスと成功・失敗が残る。`context_snapshot_attempts` に撮影の試行履歴が残る。成功済みの写真が存在する場合、`ar` では再撮影しない。失敗していた場合や写真が削除されていた場合は、次の `ar` で再試行する。既存ジョブを更新後のノードで初めて再開するときも、写真がなければ現在の姿勢で撮影を試みる。

取得失敗は警告として保存し、収集を継続する。撮影待ちのEnterは自動収集の開始を取り消す。カメラのUSB接続自体の不調でGelSight等が停止した場合は、従来の収集前確認・動作中監視が検知する。

写真は元データの物体フォルダ内にある `automatic_jobs` と一緒に保管する。現在の `extract_all` は進捗JSONに写真への参照を残すが、写真ファイルをエクスポート先へ自動コピーしない。過去の写真のないデータも引き続き使用できる。

## 必要な場合の設定

YAMLのトップレベルに次を追加すると変更できる。指定がなければ同じ既定値で有効になる。

```yaml
context_snapshot:
  enabled: true
  serial_no: "143122066280"
```

シリアル番号の変更時は文字列として指定する。収集中・再開予定のジョブでは設定ファイルの変更が再開時の整合性チェックに影響するため、追加・変更は次の新規ジョブから行う。

カメラ起動時のシリアル指定方法はRealSense ROSドライバの仕様に従う。[公式ドキュメント](https://github.com/realsenseai/realsense-ros#parameters-that-cannot-be-changed-in-runtime)

撮影タイムアウト時は、metadataに受信フレーム数、画像publisher数、専用namespace内のトピック一覧も保存する。ドライバログ自体にフレームタイムアウトがある場合は、USB接続やカメラ側の状態も確認する。
