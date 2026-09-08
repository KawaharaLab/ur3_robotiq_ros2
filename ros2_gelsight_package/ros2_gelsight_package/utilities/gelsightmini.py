import cv2
import platform
import glob
import time
try:
    from cv2.typing import MatLike
except Exception:
    from typing import Any as MatLike
import os
import re
import datetime
from typing import Optional
from .logger import log_message
from .image_processing import crop_and_resize


class Camera:
    def __init__(self, device):
        self.device = device
        self.cap = None

    def open(self) -> None:
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
        last_err = None
        for backend in backends:
            cap = cv2.VideoCapture(self.device, backend)
            if cap.isOpened():
                self.cap = cap
                return
            try:
                cap.release()
            except Exception:
                pass
            last_err = backend

        raise RuntimeError(f"Could not open camera device: {self.device} (backends tried: {backends}, last={last_err})")

    def read_frame(self) -> MatLike:
        if self.cap is None:
            raise RuntimeError("Camera is not opened.")
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to read frame from device.")
        return frame

    def release(self) -> None:
        if self.cap:
            self.cap.release()
            self.cap = None

    @staticmethod
    def list_devices() -> dict:
        devices = {}
        os_name = platform.system()
        if os_name == "Linux":
            paths = glob.glob("/dev/v4l/by-id/*")
            for idx, path in enumerate(paths):
                devices[idx] = path
        else:
            for idx in range(6):
                cap = cv2.VideoCapture(idx)
                if cap.isOpened():
                    devices[idx] = f"Camera {idx}"
                    cap.release()
        return devices

    def find_cameras_windows(camera_name):
        from pygrabber.dshow_graph import FilterGraph
        graph = FilterGraph()
        allcams = graph.get_input_devices()
        description = ""
        for cam in allcams:
            if camera_name in cam:
                description = cam

        try:
            device = graph.get_input_devices().index(description)
        except ValueError as e:
            print("Device is not in this list")
            print(graph.get_input_devices())
            import sys
            sys.exit()

        return (device,description)


class GelSightMini:
    def __init__(
        self,
        target_width: int = 320,
        target_height: int = 240,
        border_fraction: float = 0.15,
    ):
        self.camera: Camera = None
        self.recording: bool = False
        self.record_filepath: str = None
        self.frame_count: int = 0
        self.time_prev: float = time.time()
        self.fps: float = 0
        self.current_frame_rgb: MatLike = None
        self.current_frame: MatLike = None
        self.target_width: int = target_width
        self.target_height: int = target_height
        self.border_fraction: float = border_fraction
        self.video_writer = None
        self.serial_number = None

    def get_device_list(self) -> dict:
        return Camera.list_devices()

    def select_device(self, device_idx=None, device_path: Optional[str] = None) -> None:
        if device_idx==None and platform.system() == "Windows":
                (dev, desc) = Camera.find_cameras_windows("GelSight Mini")
                print("Found: ", desc, ", dev: ", dev)
                device_idx = dev
                match = re.search("[A-Z0-9]{4}-[A-Z0-9]{4}", desc)
                if match:
                    self.serial_number = match.group()

        if platform.system() == "Linux":
            if device_path:
                device_id = device_path
            else:
                devices = Camera.list_devices()
                for ix in range(0,len(devices)):
                    print("Device: ", devices[ix])
                if isinstance(devices.get(device_idx), str):
                    device_id = devices[device_idx]
                else:
                    device_id = device_idx
            if isinstance(device_id, str):
                device_id = os.path.realpath(device_id)
        else:
            device_id = device_idx

        if self.camera:
            self.camera.release()

        def try_open(device_value):
            cam = Camera(device=device_value)
            cam.open()
            
            # 【重要追加】OpenCVのバッファサイズを最小(1)に制限し、常に最新フレームを取得する
            cam.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
            
            cam.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            cam.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.target_width)
            cam.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.target_height)

            # ⬇️ 【ここを追加】カメラに対してネイティブ30fpsでの動作を要求する
            # cam.cap.set(cv2.CAP_PROP_FPS, 30)
            
            if cam.cap.get(cv2.CAP_PROP_FRAME_WIDTH) != self.target_width:
                cam.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.target_width)
                cam.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.target_height)
                
            current_width = cam.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            current_height = cam.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            log_message(
                f"Camera opened successfully with resolution {current_width}x{current_height}!"
            )
            return cam

        try:
            self.camera = try_open(device_id)
        except Exception as e:
            if isinstance(device_id, str) and device_id.endswith("-video-index1"):
                alt = device_id.replace("-video-index1", "-video-index0")
                log_message(f"Retrying with device {alt} after failure: {e}")
                try:
                    self.camera = try_open(alt)
                except Exception as e2:
                    log_message(f"Could not open selected device: {e2}")
            else:
                log_message(f"Could not open selected device: {e}")

    def start(self) -> None:
        if not self.camera:
            log_message("Please select a device first!")
            return
        self.recording = False
        self.frame_count = 0

    def start_recording(self, filepath: str = None) -> None:
        if not self.camera:
            log_message("Please select a device first!")
            return

        if filepath is None or not os.path.isdir(filepath):
            filepath = self.create_folder()

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(filepath, f"recording_{timestamp}.mp4")

        if not filepath:
            log_message("Error: File path is empty!")
            return

        fps = self.fps if self.fps > 0 else 30

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.video_writer = cv2.VideoWriter(
            filepath, fourcc, fps, (self.target_width, self.target_height)
        )
        self.record_filepath = filepath
        self.recording = True
        self.frame_count = 0
        log_message(f"Started recording to {filepath}")

    def stop_recording(self) -> None:
        self.recording = False
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
            log_message(f"Recording saved to {self.record_filepath}")
            self.record_filepath = None

    def update(self, dt: float) -> Optional[MatLike]:
        if not self.camera:
            return None

        try:
            frame = self.camera.read_frame()
        except Exception as e:
            log_message(f"Error reading frame: {e}")
            return None

        time_now = time.time()
        actual_dt = time_now - self.time_prev
        self.fps = 1.0 / actual_dt if actual_dt > 0 else 0
        self.time_prev = time_now

        self.current_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.frame_count += 1
        
        # 【重要変更】target_size を強制的に ML用の (224, 224) に固定
        self.current_frame = crop_and_resize(
            image=self.current_frame,
            target_size=(224, 224), 
            border_fraction=self.border_fraction,
        )

        if self.recording and self.video_writer is not None:
            self.video_writer.write(cv2.cvtColor(self.current_frame, cv2.COLOR_RGB2BGR))

        return self.current_frame

    def create_folder(self) -> str:
        desktop = os.path.join(os.path.expanduser("~"), "Desktop")
        folder_name = datetime.datetime.now().strftime("%Y-%m-%d")
        folder_path = os.path.join(desktop, folder_name)
        os.makedirs(folder_path, exist_ok=True)
        return folder_path

    def save_screenshot(self, filepath: str = None) -> bool:
        saved = False
        if filepath is None:
            return saved

        if self.current_frame is not None:
            now = datetime.datetime.now()
            filename = os.path.join(
                filepath, f"screenshot_{now.strftime('%Y%m%d_%H%M%S')}.png"
            )
            try:
                cv2.imwrite(
                    filename,
                    cv2.cvtColor(self.current_frame, cv2.COLOR_RGB2BGR),
                )
                saved = True
                log_message(f"Screenshot saved to {filename}")
            except Exception as e:
                log_message(f"Failed to save image: {e}")

        return saved