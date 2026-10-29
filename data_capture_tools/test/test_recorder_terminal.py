"""Exercise the real recorder spawn with a competing reader on a real POSIX TTY."""
import ast
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import tempfile
import termios
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from data_capture_tools.object_automation import ObjectAutomationMixin, Paused


# Model rosbag's competing keyboard reader. With a shared TTY it changes terminal
# mode and consumes Enter. With DEVNULL it cannot touch the collection terminal.
READER = r'''
import os, signal, sys, termios, time, tty
from pathlib import Path
root = Path(sys.argv[1])
def stop(*args):
    raise SystemExit(0)
signal.signal(signal.SIGINT, stop)
attributes = None
try:
    if os.isatty(0):
        attributes = termios.tcgetattr(0)
        tty.setcbreak(0)
    (root/'ready').touch()
    if attributes is not None:
        while True:
            if os.read(0, 1) in (b'\n', b'\r'):
                (root/'stolen').touch()
    else:
        while True:
            time.sleep(.02)
finally:
    if attributes is not None:
        termios.tcsetattr(0, termios.TCSANOW, attributes)
'''


class RecorderTerminalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = ROOT/'data_capture_tools/data_capture_node_10times.py'
        tree = ast.parse(source.read_text())
        node = next(x for x in tree.body if isinstance(x, ast.ClassDef) and x.name == 'DataCaptureNode')
        method = next(x for x in node.body if isinstance(x, ast.FunctionDef) and x.name == '_start_rosbag')
        # Load the actual method without importing hardware/ROS dependencies.
        namespace = dict(subprocess=subprocess, os=os, Path=Path)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
        cls.start_recorder = staticmethod(namespace['_start_rosbag'])

    def exercise(self, shared_stdin):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'logs').mkdir()
            master, slave = pty.openpty()
            self.addCleanup(os.close, master)
            self.addCleanup(os.close, slave)
            terminal = os.fdopen(os.dup(slave), 'r')
            self.addCleanup(terminal.close)
            before = termios.tcgetattr(slave)
            node = NS(_bag_uri=root/'bag', _session_dir=root,
                      config=NS(bag=NS(storage='sqlite3', compression_mode='file', compression_format='zstd'),
                                topics=[NS(name='/force_torque/left')]),
                      get_logger=lambda: NS(info=lambda *args: None))
            spawn = subprocess.Popen
            commands = []

            def launch(command, **kwargs):
                commands.append((command, dict(kwargs)))
                if shared_stdin:
                    # Reproduce the pre-fix inheritance: substitute this PTY for
                    # the manager's actual terminal in this isolated test.
                    kwargs['stdin'] = slave
                return spawn([sys.executable, '-c', READER, str(root)], **kwargs)

            with patch.object(subprocess, 'Popen', side_effect=launch):
                self.start_recorder(node)
            child = node._bag_process
            try:
                deadline = time.monotonic()+3
                while not (root/'ready').exists():
                    if time.monotonic() > deadline:
                        self.fail('Keyboard fixture did not start')
                    self.assertIsNone(child.poll())
                    time.sleep(.01)
                os.write(master, b'\n')
                if shared_stdin:
                    deadline = time.monotonic()+2
                    while not (root/'stolen').exists() and time.monotonic() < deadline:
                        time.sleep(.01)
                    self.assertTrue((root/'stolen').exists())
                monitor = ObjectAutomationMixin()
                monitor._stop_event = threading.Event()
                monitor._auto_health = lambda **kw: monitor._auto_manual()
                with patch.object(sys, 'stdin', terminal):
                    if shared_stdin:
                        monitor._auto_wait(.1)  # Enter was swallowed; no pause.
                    else:
                        started = time.monotonic()
                        with self.assertRaisesRegex(Paused, 'Manual pause requested'):
                            monitor._auto_wait(10.)
                        self.assertLess(time.monotonic()-started, .5)
                if not shared_stdin:
                    self.assertEqual(commands[0][1]['stdin'], subprocess.DEVNULL)
                    self.assertEqual(termios.tcgetattr(slave), before)
                    self.assertFalse((root/'stolen').exists())
                    self.assertIn('/force_torque/left', commands[0][0])
                    self.assertIn('--compression-format', commands[0][0])
            finally:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=3)
                    self.fail('Recorder fixture failed to stop on SIGINT')
                node._bag_stdout.close()
                node._bag_stderr.close()
            self.assertEqual(child.returncode, 0)

    def test_shared_terminal_reproduces_missed_enter(self):
        self.exercise(shared_stdin=True)

    def test_isolated_recorder_allows_enter_during_hold(self):
        self.exercise(shared_stdin=False)


if __name__ == '__main__':
    unittest.main()
