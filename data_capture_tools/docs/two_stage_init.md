# 二段階init（球体等・試験導入）

`data_capture_manager_10`専用の追加モード。設定しなければ従来のinitを使う。
通常の収集設定や既存の初期姿勢は変更しない。まず別の試験用YAMLで確認する。

## 処理

1. 従来の教示姿勢・135 mm開放へ移動。
2. クランプ固定面から、`clamp_center_width_m / 2 + arm_offset`だけ**ベースXYの135度方向**へ移動。
3. 元の向きで左右エッジを探索し、クランプ軸上の中心へ移動する。この段階で最終把持はしない。
4. 135 mmへ開放。`retreat_base_m`だけ退避し、ベースZ軸周りに指定角度だけ回転。
   `pivot_tcp_m`で指定した物理的中心が復帰後に同じ場所へ戻るようTCP位置も補正。
5. 退避前の中心へ復帰。acの前後オフセットがある場合、**ここで**回転後の開閉軸に垂直な方向へ移動。
   先にオフセットすると1段階目の中心探索で打ち消されるため。
6. 新しい開閉軸（135度 + rotation_deg）で再度エッジ・均等接触を探索し、有効幅を確定。
7. 2段階目のプローブ幅へ開放し終了。

`target_diameter`は**回転後の開閉方向**の動作計画用幅として残す。
クランプ方向の寸法とは独立。上位置のオフセットは従来どおりベースZ方向。
正回転と逆回転はどちらも初期教示姿勢から始めるので、試行ごとに90度が累積することはない。

## 回転中心と退避経路

`pivot_tcp_m`は、**UR3のInstallationで現在有効なTCPから、保持したい接触面中央までのベクトルを、そのTCPの座標系で表した値**。
フランジ座標やベース座標で入力しない。TCPがすでにその中央なら明示的に`[0, 0, 0]`。
不明なままゼロを代入しない。制御側の実TCP姿勢 `/tcp_pose_broadcaster/pose`（標準frame `base`）を使い、
URDFから計算したTFは回転位置の算出に使わない。

`retreat_base_m`は**ベース座標系**の退避ベクトル。姿勢回転をクランプに接触したまま行わないために必要。
上へ50 mmという例は経路の安全性を保証しない。クランプ、指、ケーブルの回転中の干渉を実機で確認する。
回転中はTCPの直線補間と姿勢補間を行う。物理的な中心の完全固定を保証するのは退避前と復帰後の端点であり、
回転中の全軌道ではない。

初期姿勢・各位置を変更して再実行する前は、外部から見て指が試料から離れているか確認する。
中断姿勢から無条件でinitを再開すると、教示姿勢への移動経路が前回と異なる。
初回は柔らかい試料、周囲に余裕のある配置で1回ずつ試す。

## 設定と起動

通常の`data_capture.yaml`を別名へコピーし、`config/two_stage_init.example.yaml`の
`initialization:`ブロックを**ルートに一つだけ**追加。nullの寸法・TCP位置・退避ベクトルを記入する。
このexampleは追加ブロックであり、単独では収集設定にならない。

`rotation_deg: 90.0`を`-90.0`へ変えるだけで逆回転を試せる。
正の角度はベース+Zから原点方向を見て反時計回り。
値を変更したら収集ノードを終了し、変更したYAMLを指定して起動し直す。

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select data_capture_tools
source install/setup.bash
ros2 topic echo /tcp_pose_broadcaster/pose --once
ros2 run data_capture_tools data_capture_manager_10 --ros-args \
  -p config:=/home/tsumura/my_robotiq_ws/src/ur3_robotiq_ros2/data_capture_tools/config/data_capture_two_stage_test.yaml
```

UR3ドライバ、External Control、グリッパドライバ、gripper_client、左右MMS101は必要。
新規ポート設定や追加ドライバは不要。新しいターミナルでは従来どおり両setupをsourceする。
GelSightノード・RealSenseノードは`test_only: true`かつ`require_gelsight: false`なら不要。
メニューで`i`を実行。手動init中のEnterでSTOP、Ctrl+Cでも中断。実機の停止手段も利用できる状態で行う。
**センサを接続しない設定は力覚センサには適用されない。**

## 監視・記録

- TCP、力覚、グリッパ状態の受信が250 msを超えて古い／非数値／グリッパfault時は停止。
- 接触探索前にtareしてから、左右のtare相対Fy（従来の20点平均）の大きい方を`max_init_force_n`と比較。
  本番acの30 N上限とは別のinit用上限。ソフトウェア監視は機械の安全機能の代替ではない。
- 新規のアーム動作は実TCPの端点位置・姿勢を確認してから次へ進む。
  第1・第2段階の微小アーム移動も同様。グリッパ開放・プローブ幅は応答と報告幅を確認。
- 探索移動上限、350秒のinit全体上限、均等な低荷重接触が得られない場合は失敗。
- 異常時に停止後の回転・再試行・自動開放は行わない。障害物との位置関係を確認して手動で復帰。
- `init_records/<init_id>/samples.jsonl.gz`等の既存init記録に各段階、設定、目標・到達位置を保存。
  ここでの動作検証に画像・bagの取得は必要ない。
- test_only結果は`test_only_motion_success`で成功を表示するが`valid_for_collection: false`。
  `i/t/q`以外は拒否する。検証結果をそのまま本番の接触基準として再利用しない。

動作検証後に本番へ切り替えるときは`test_only: false`、`require_gelsight: true`として
左右GelSightを起動し、**改めてinitする**。初回の実機検証では回転方向、中心の保持、退避の余裕、
左右探索方向、終了後の指の開放を目視し、init記録の両段階と最終有効幅を確認する。
通常モードへ戻すには`initialization.mode: legacy`（またはブロック削除）。
