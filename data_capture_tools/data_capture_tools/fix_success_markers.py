import os
from pathlib import Path

def fix_success_markers(root_path: str):
    root_p = Path(root_path).resolve()
    print(f"🔍 Checking for missing .success markers in: {root_p}")
    
    count = 0
    # 構造: root / 物体名 / 時刻フォルダ
    # glob("*/*") を使うことで、画像フォルダの中身を無視して高速にスキャンします
    for session_dir in root_p.glob("*/*"):
        if not session_dir.is_dir():
            continue
            
        csv_dir = session_dir / "csv"
        success_file = session_dir / ".success"
        
        # 条件：csvフォルダが存在し、かつ .success ファイルが存在しない場合
        if csv_dir.exists() and not success_file.exists():
            # .success ファイルを作成
            success_file.touch()
            print(f"✅ Created .success in: {session_dir.name}")
            count += 1

    print("\n" + "="*30)
    print(f"✨ Task Completed!")
    print(f"Added {count} missing .success markers.")
    print("="*30)

if __name__ == "__main__":
    # あなたのデータルートパスを指定してください
    DATA_ROOT = "/home/tsumura/my_robotiq_ws/data"
    fix_success_markers(DATA_ROOT)