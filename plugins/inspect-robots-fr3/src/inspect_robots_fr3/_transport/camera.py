"""RGB camera capture through OpenCV/V4L2 in the harness's own environment."""

from __future__ import annotations

from multiprocessing.connection import Client
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np


class FramePending(RuntimeError):
    """Waiting for USB enumeration or the first RGB frame during startup."""


class GeminiCamera:
    def __init__(
        self,
        python: str,
        log_path: Path,
        *,
        serial="",
        device="",
        width=1280,
        height=720,
        fps=15,
        maximum_age_s=0.5,
        startup_timeout_s=20,
    ):
        if not np.isfinite(startup_timeout_s) or startup_timeout_s <= 0:
            raise ValueError("camera startup_timeout_s must be finite and positive")
        self.maximum_age_s = maximum_age_s
        self.temp = tempfile.TemporaryDirectory(prefix="inspect-robots-camera-")
        self.connection = None
        self.process = None
        self.log = Path(log_path).open("w")
        address = str(Path(self.temp.name) / "camera.sock")
        command = [
            python,
            str(Path(__file__).with_name("camera_worker.py")),
            "--socket",
            address,
            "--serial",
            serial,
            "--device",
            device,
            "--width",
            str(width),
            "--height",
            str(height),
            "--fps",
            str(fps),
            "--startup-timeout-s",
            str(startup_timeout_s),
        ]
        try:
            self.process = subprocess.Popen(command, stdout=self.log, stderr=self.log)
            deadline = time.monotonic() + startup_timeout_s + 2
            while time.monotonic() < deadline:
                if self.process.poll() is not None:
                    raise RuntimeError(f"RGB capture process exited; see {log_path}")
                try:
                    self.connection = Client(address, family="AF_UNIX")
                    break
                except (FileNotFoundError, ConnectionRefusedError):
                    time.sleep(0.05)
            if self.connection is None:
                raise TimeoutError(f"RGB capture process did not start; see {log_path}")
            last_wait_message = None
            while True:
                try:
                    self.snapshot()
                    break
                except FramePending as exc:
                    if time.monotonic() >= deadline or self.process.poll() is not None:
                        raise TimeoutError(f"Gemini335 startup timed out: {exc}") from exc
                    if str(exc) != last_wait_message:
                        print(f"[camera] {exc}", flush=True)
                        last_wait_message = str(exc)
                    time.sleep(0.1)
        except BaseException:
            self.close()
            raise

    def snapshot(self, *, after_ns=None):
        deadline = time.monotonic() + 5
        while True:
            rgb, packet = self._snapshot()
            if after_ns is None or packet["host_monotonic_ns"] >= after_ns:
                return rgb, packet
            if time.monotonic() >= deadline:
                raise TimeoutError("Gemini335 did not produce a new frame after the action")
            time.sleep(0.01)

    def _snapshot(self):
        self.connection.send("snapshot")
        if not self.connection.poll(5):
            raise TimeoutError("Gemini335 snapshot request timed out")
        packet = self.connection.recv()
        if "error" in packet:
            if packet.get("pending"):
                raise FramePending(packet["error"])
            raise RuntimeError(f"Gemini335: {packet['error']}")
        age = (time.monotonic_ns() - packet["host_monotonic_ns"]) / 1e9
        if not 0 <= age <= self.maximum_age_s:
            raise RuntimeError(f"Gemini335 frame is stale ({age:.3f}s)")
        data = packet.pop("rgb_bytes")
        rgb = np.frombuffer(data, dtype=np.uint8).reshape(packet["shape"]).copy()
        return rgb, packet

    def close(self):
        if self.connection is not None:
            try:
                self.connection.send("close")
            except (OSError, EOFError):
                pass
            self.connection.close()
            self.connection = None
        if self.process is not None:
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
        self.log.close()
        self.temp.cleanup()


class MockCamera:
    def snapshot(self, *, after_ns=None):
        import cv2

        rgb = np.full((480, 640, 3), 210, dtype=np.uint8)
        cv2.circle(rgb, (420, 250), 90, (60, 100, 210), 8)
        cv2.rectangle(rgb, (170, 210), (230, 300), (220, 100, 40), -1)
        cv2.putText(rgb, "MOCK - not a physical task", (15, 35), 0, 0.65, (20, 20, 20), 2)
        stamp = time.monotonic_ns()
        return rgb, {
            "host_monotonic_ns": stamp,
            "device_timestamp_ns": stamp,
            "sequence": stamp,
            "source": "mock",
            "serial": "mock",
        }

    def close(self):
        pass
