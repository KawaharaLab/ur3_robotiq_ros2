"""Capture one contextual wrist-camera image using a temporary ROS camera node."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import traceback
import time

from .object_plan import atomic_json

DEFAULT_SERIAL = '143122066280'  # Wrist camera in realsense_dual.launch.py.


def stop_camera(process):
    """Stop only the native camera process started by this capture."""
    if process is None:
        return None
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        if process.poll() is not None:
            return process.returncode
        try:
            process.send_signal(sig)
        except ProcessLookupError:
            return process.wait(timeout=1.)
        try:
            return process.wait(timeout=2.)
        except subprocess.TimeoutExpired:
            continue
    raise RuntimeError('Temporary RealSense process did not exit')


def camera_command(executable, namespace, serial_no):
    # A distinct namespace keeps temporary frames out of acquisition bag topics.
    return [str(executable), '--ros-args', '-r', '__ns:=/' + namespace,
            '-r', '__node:=realsense2_camera',
            '-p', 'serial_no:=_' + serial_no,
            '-p', 'enable_color:=true', '-p', 'enable_depth:=false',
            '-p', 'enable_infra1:=false', '-p', 'enable_infra2:=false',
            '-p', 'enable_gyro:=false', '-p', 'enable_accel:=false',
            '-p', 'rgb_camera.color_profile:=640x480x15']


def capture(directory, serial_no, warmup_s=2., timeout_s=20.):
    """Start RGB only, discard warm-up frames, save one JPEG, then stop camera."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    if (directory / 'metadata.json').exists() or (directory / 'wrist.jpg').exists():
        raise FileExistsError('Snapshot output exists; choose a new directory')
    report = dict(schema_version=1, status='failed', serial_no=serial_no,
                  requested_color_profile='640x480x15', warmup_s=warmup_s,
                  timeout_s=timeout_s, started_time_ns=time.time_ns(),
                  purpose='Object/mounting reference image before automatic collection',
                  snapshot_stage='Stationary arm pose before automatic run begins or resumes',
                  camera_stop_confirmed=False)
    process = node = context = executor = None
    received = dict(first_at=None, frames=0, image=None, camera_info=None)
    try:
        import cv2
        import rclpy
        from ament_index_python.packages import get_package_prefix
        from cv_bridge import CvBridge
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
        prefix = Path(get_package_prefix('realsense2_camera'))
        executable = prefix / 'lib/realsense2_camera/realsense2_camera_node'
        if not executable.is_file():
            raise FileNotFoundError('RealSense ROS camera executable not found: ' + str(executable))
        namespace = 'ac_context_' + str(os.getpid())
        topic = '/' + namespace + '/realsense2_camera/color/image_raw'
        report['topic'] = topic
        context = Context()
        rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('context_snapshot_receiver', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        bridge = CvBridge()

        def image_callback(message):
            now = time.monotonic()
            received['frames'] += 1
            if received['first_at'] is None:
                received['first_at'] = now
            if now - received['first_at'] < warmup_s or received['image'] is not None:
                return
            image = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            if image is None or image.size == 0:
                raise ValueError('Empty RealSense color image')
            received['image'] = image
            report.update(image_header_ns=message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                          image_frame_id=message.header.frame_id,
                          host_receive_time_ns=time.time_ns(), shape=list(image.shape),
                          discarded_frames=received['frames'] - 1)

        def info_callback(message):
            received['camera_info'] = dict(width=message.width, height=message.height,
                distortion_model=message.distortion_model, d=list(message.d),
                k=list(message.k), r=list(message.r), p=list(message.p))

        image_sub = node.create_subscription(Image, topic, image_callback, qos_profile_sensor_data)
        info_sub = node.create_subscription(CameraInfo, topic.rsplit('/', 1)[0] + '/camera_info',
                                            info_callback, qos_profile_sensor_data)
        command = camera_command(executable, namespace, serial_no)
        report['camera_command'] = command
        deadline = time.monotonic() + timeout_s
        with (directory / 'camera.log').open('x') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       stdin=subprocess.DEVNULL)
            while received['image'] is None:
                if process.poll() is not None:
                    raise RuntimeError('Temporary RealSense driver exited; see camera.log')
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    try:
                        report['image_publishers'] = node.count_publishers(topic)
                        report['observed_camera_topics'] = [dict(name=name, types=types)
                            for name, types in node.get_topic_names_and_types()
                            if name.startswith('/' + namespace + '/')]
                    except Exception as exc:
                        report['graph_inspection_error'] = str(exc)
                    raise TimeoutError('No warmed-up wrist image before timeout; see camera.log')
                executor.spin_once(timeout_sec=min(.1, remaining))
            success, jpeg = cv2.imencode('.jpg', received['image'], [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not success:
                raise ValueError('JPEG encoding failed')
            with (directory / 'wrist.jpg').open('xb') as image_file:
                image_file.write(jpeg.tobytes())
            report.update(status='saved', image='wrist.jpg', jpeg_quality=95,
                          camera_info=received['camera_info'])
    except (Exception, KeyboardInterrupt) as exc:
        report.update(status='failed', error=type(exc).__name__ + ': ' + str(exc),
                      traceback=traceback.format_exc())
    finally:
        try:
            report['camera_exit_code'] = stop_camera(process)
            report['camera_stop_confirmed'] = True
        except Exception as exc:
            report.update(status='failed', cleanup_error=str(exc))
        try:
            if executor is not None:
                executor.shutdown(timeout_sec=2.)
        except Exception as exc:
            report.update(status='failed', executor_cleanup_error=str(exc))
        try:
            if node is not None:
                node.destroy_node()
        except Exception as exc:
            report.update(status='failed', receiver_cleanup_error=str(exc))
        try:
            if context is not None and context.ok():
                context.shutdown()
        except Exception as exc:
            report.update(status='failed', context_cleanup_error=str(exc))
        report['received_frames'] = received['frames']
        report['finished_time_ns'] = time.time_ns()
        atomic_json(directory / 'metadata.json', report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--serial', default=DEFAULT_SERIAL)
    parser.add_argument('--warmup-s', type=float, default=2.)
    parser.add_argument('--timeout-s', type=float, default=20.)
    args = parser.parse_args(argv)
    if not args.serial or not args.serial.isascii() or not args.serial.isdigit():
        parser.error('--serial must be the camera numeric serial number')
    if not 0 <= args.warmup_s <= 5 or not args.warmup_s < args.timeout_s <= 30:
        parser.error('Require warmup in 0..5 seconds and warmup < timeout <= 30 seconds')

    def terminate(signum, frame):
        raise InterruptedError('Snapshot process terminated')

    previous = signal.signal(signal.SIGTERM, terminate)
    try:
        report = capture(args.out, args.serial, args.warmup_s, args.timeout_s)
    finally:
        signal.signal(signal.SIGTERM, previous)
    print('Context snapshot ' + report['status'] + ': ' + str(args.out.resolve()), flush=True)
    for key in ('error', 'cleanup_error', 'executor_cleanup_error',
                'receiver_cleanup_error', 'context_cleanup_error'):
        if report.get(key):
            print(key + ': ' + report[key], file=sys.stderr, flush=True)
    return 0 if report['status'] == 'saved' else 1


if __name__ == '__main__':
    raise SystemExit(main())
