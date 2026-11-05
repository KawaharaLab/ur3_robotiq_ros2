#!/usr/bin/env python3
"""Passive bag/local-copy check with automatic leases. Sends no motion commands."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'data_capture_tools'))

TOPICS = ('/gelsight/left/image_raw/compressed', '/gelsight/right/image_raw/compressed',
    '/gelsight_mini_publisher_1/acquisition_timing', '/gelsight_mini_publisher_2/acquisition_timing',
    '/force_torque/left', '/force_torque/right',
    '/mms101_driver_1/acquisition_timing', '/mms101_driver_2/acquisition_timing')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=40.)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 180:parser.error('--seconds must be in [1,180]')
    if args.out.exists():parser.error('--out exists; choose a new run directory')
    root = args.out.resolve();root.mkdir(parents=True)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from data_capture_tools.gelsight_local_recording import GelsightLocalRecordingClient
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('gelsight_local_check')
    client = GelsightLocalRecordingClient(node)
    executor = SingleThreadedExecutor();executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True);thread.start()
    process = None;error = None
    try:
        if not client.start(root):
            raise RuntimeError('Both GelSight local writers must acknowledge startup. Check rebuilt nodes, acquisition_records:=true and acquisition_record_mode:=bag. No bag or robot motion started.')
        command = ['ros2','bag','record','-s','sqlite3','--compression-mode','file',
                   '--compression-format','zstd','-o',str(root/'bag'),*TOPICS]
        with (root/'recorder.log').open('w') as log:
            process = subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
            client.attach_process(process)
            print(f'Recording {args.seconds:g} seconds without motion: {root}',flush=True)
            deadline = time.monotonic()+args.seconds
            while time.monotonic()<deadline:
                if process.poll() is not None:raise RuntimeError('Recorder exited early; inspect recorder.log')
                time.sleep(.05)
            process.send_signal(signal.SIGINT)
            process.wait(timeout=30.)
            if process.returncode != 0:raise RuntimeError('Recorder exit code '+str(process.returncode))
    except KeyboardInterrupt:
        error = 'Manual interruption'
    except Exception as exc:
        error = type(exc).__name__+': '+str(exc)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=30.)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:process.wait(timeout=10.)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5.)
        try:client.stop()
        except Exception as exc:error = error or ('Local stop: '+str(exc))
        executor.shutdown(timeout_sec=2.);thread.join(timeout=2.)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    report = dict(run_error=error,output=str(root))
    manifest = root/'gelsight_local_recording.json'
    if manifest.exists():report['recording_control'] = json.loads(manifest.read_text())
    if (root/'bag/metadata.yaml').exists():
        try:
            from compare_gelsight_local_bag import compare
            report.update(compare(root/'bag',root/'gelsight_acquisition_records'))
        except Exception as exc:report['comparison_error'] = type(exc).__name__+': '+str(exc)
    (root/'comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print('Result:',root/'comparison.json')
    if error:print(error,file=sys.stderr)
    return 1 if error or report.get('comparison_error') else 0


if __name__ == '__main__':raise SystemExit(main())
