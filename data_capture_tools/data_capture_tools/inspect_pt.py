import torch
import matplotlib.pyplot as plt
import numpy as np

# 確認したいファイルのパスを一つ指定してください
file_path = "/home/tsumura/my_robotiq_ws/processed_data/softbaseball_fieldforce_test_0303/20260304_211001/data.pt"

def inspect_data(path):
    # データをロード
    data = torch.load(path)
    
    print(f"--- File: {path} ---")
    print(f"Keys: {data.keys()}")
    
    # 1. ラベルの確認
    label = data['label'].item()
    print(f"Label ID: {label}")
    
    # 2. センサデータの確認 (Shape: [50, 6])
    sensor = data['sensor']
    print(f"Sensor Shape: {sensor.shape} (Steps, Channels)")
    print(f"Sensor Sample (first 2 steps):\n{sensor[:2]}")
    
    # 3. 画像データの確認 (Shape: [5, 3, 128, 128])
    images = data['images']
    print(f"Images Shape: {images.shape} (Num, C, H, W)")
    print(f"Images Pixel Range: min={images.min():.3f}, max={images.max():.3f}")

    # --- 可視化 ---
    fig = plt.figure(figsize=(15, 8))
    
    # 6軸センサのプロット (左側)
    ax_sensor = fig.add_subplot(2, 1, 1)
    ax_sensor.plot(sensor.numpy())
    ax_sensor.set_title("6-axis Sensor Data (Normalized/Extracted)")
    ax_sensor.legend(['fx', 'fy', 'fz', 'tx', 'ty', 'tz'])
    
    # GelSight画像のプロット (右側/下側)
    for i in range(images.shape[0]):
        ax_img = fig.add_subplot(2, images.shape[0], images.shape[0] + i + 1)
        # [C, H, W] -> [H, W, C] に戻して表示
        img_to_show = images[i].permute(1, 2, 0).numpy()
        ax_img.imshow(img_to_show)
        ax_img.axis('off')
        ax_img.set_title(f"Frame {i}")

    plt.tight_layout()
    plt.savefig("data_check.png") # 結果を画像として保存
    print("Visualization saved as 'data_check.png'")

if __name__ == "__main__":
    inspect_data(file_path)