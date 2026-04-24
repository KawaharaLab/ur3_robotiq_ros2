import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, random_split
from torchvision import models
from pathlib import Path
from tqdm import tqdm
import numpy as np

# ==========================================
# 1. 設定パラメータ
# ==========================================
PROCESSED_DATA_DIR = Path("/home/tsumura/my_robotiq_ws/processed_data")
SAVE_MODEL_PATH = "tactile_fusion_model.pth"

BATCH_SIZE = 16
EPOCHS = 30
LEARNING_RATE = 1e-4
NUM_CLASSES = 8 

# 分割割合
TRAIN_RATIO = 0.7
VAL_RATIO = 0.15
# TEST_RATIO は残り (0.15)

# ==========================================
# 2. Dataset / Model 定義 (前回と同様)
# ==========================================
class TactileDataset(Dataset):
    def __init__(self, root_dir):
        self.file_paths = list(Path(root_dir).rglob("data.pt"))
        if not self.file_paths:
            raise RuntimeError(f"No data.pt found in {root_dir}")

    def __len__(self):
        return len(self.file_paths)

    def __getitem__(self, idx):
        data = torch.load(self.file_paths[idx])
        return data["images"], data["sensor"], data["label"]

class LateFusionModel(nn.Module):
    def __init__(self, num_classes=8):
        super(LateFusionModel, self).__init__()
        resnet = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
        self.vision_backbone = nn.Sequential(*list(resnet.children())[:-1])
        self.vision_fc = nn.Linear(512, 128)
        
        self.sensor_encoder = nn.Sequential(
            nn.Conv1d(6, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool1d(2),
            nn.Conv1d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(1),
            nn.Flatten(),
            nn.Linear(64, 128),
            nn.ReLU()
        )
        
        self.classifier = nn.Sequential(
            nn.Linear(128 + 128, 64),
            nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(64, num_classes)
        )

    def forward(self, x_img, x_sensor):
        batch_size, seq_len, c, h, w = x_img.shape
        x_img = x_img.view(batch_size * seq_len, c, h, w)
        img_feats = self.vision_backbone(x_img).view(batch_size * seq_len, -1)
        img_feats = self.vision_fc(img_feats)
        img_feats = img_feats.view(batch_size, seq_len, -1).mean(dim=1)
        
        x_sensor = x_sensor.permute(0, 2, 1)
        sensor_feats = self.sensor_encoder(x_sensor)
        
        combined = torch.cat([img_feats, sensor_feats], dim=1)
        return self.classifier(combined)

# ==========================================
# 3. 学習・評価・テスト実行
# ==========================================
def run_experiment():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    # データのロードと分割
    dataset = TactileDataset(PROCESSED_DATA_DIR)
    n_total = len(dataset)
    n_train = int(TRAIN_RATIO * n_total)
    n_val = int(VAL_RATIO * n_total)
    n_test = n_total - n_train - n_val

    # 再現性のためのシード固定 (任意)
    train_ds, val_ds, test_ds = random_split(
        dataset, [n_train, n_val, n_test], 
        generator=torch.Generator().manual_seed(42)
    )

    train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=BATCH_SIZE, shuffle=False)
    test_loader = DataLoader(test_ds, batch_size=BATCH_SIZE, shuffle=False)

    print(f"Data split: Train={n_train}, Val={n_val}, Test={n_test}")

    model = LateFusionModel(num_classes=NUM_CLASSES).to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)

    # --- 学習ループ ---
    best_val_acc = 0.0
    for epoch in range(EPOCHS):
        model.train()
        for imgs, sensors, labels in train_loader:
            imgs, sensors, labels = imgs.to(device), sensors.to(device), labels.to(device)
            optimizer.zero_grad()
            loss = criterion(model(imgs, sensors), labels)
            loss.backward()
            optimizer.step()

        # 検証 (Validation)
        model.eval()
        val_correct, val_total = 0, 0
        with torch.no_grad():
            for imgs, sensors, labels in val_loader:
                imgs, sensors, labels = imgs.to(device), sensors.to(device), labels.to(device)
                outputs = model(imgs, sensors)
                val_correct += (outputs.argmax(1) == labels).sum().item()
                val_total += labels.size(0)
        
        val_acc = 100. * val_correct / val_total
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            torch.save(model.state_dict(), SAVE_MODEL_PATH)
            print(f"Epoch {epoch+1}: Best Val Acc updated to {val_acc:.2f}%")

    # --- 最終テスト (Test) ---
    print("\n--- Final Evaluation on Test Set ---")
    model.load_state_dict(torch.load(SAVE_MODEL_PATH))
    model.eval()
    test_correct, test_total = 0, 0
    with torch.no_grad():
        for imgs, sensors, labels in test_loader:
            imgs, sensors, labels = imgs.to(device), sensors.to(device), labels.to(device)
            outputs = model(imgs, sensors)
            test_correct += (outputs.argmax(1) == labels).sum().item()
            test_total += labels.size(0)
    
    print(f"Final Test Accuracy: {100. * test_correct / test_total:.2f}%")

if __name__ == "__main__":
    run_experiment()