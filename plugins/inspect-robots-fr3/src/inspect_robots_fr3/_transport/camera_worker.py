"""RGB UVC capture in the SAME harness environment; no Orbbec SDK.

A subprocess makes a wedged VideoCapture.read interruptible without blocking
robot cleanup. IPC contains RGB bytes and host-receive timestamps only.
"""

from __future__ import annotations

import argparse
import errno
import fcntl
from multiprocessing.connection import Listener
import os
from pathlib import Path
import struct
import threading
import time


class CameraNotReady(RuntimeError):
    """Retryable absence during USB/V4L2 enumeration, before robot startup."""


def gemini_usb_devices(usb_root=Path("/sys/bus/usb/devices")):
    devices = []
    for device in sorted(usb_root.iterdir()):
        try:
            if (device / "idVendor").read_text().strip().lower() == "2bc5" and "gemini" in (
                device / "product"
            ).read_text().lower():
                devices.append(device)
        except FileNotFoundError:
            continue  # USB interfaces and devices disappearing during hotplug.
    return devices


def select_usb_device(serial="", usb_root=Path("/sys/bus/usb/devices")):
    devices = gemini_usb_devices(usb_root)
    exact = []
    for device in devices:
        try:
            if serial and (device / "serial").read_text().strip() == serial:
                exact.append(device)
        except FileNotFoundError:
            continue
    if len(exact) == 1:
        return exact[0]
    if len(devices) == 1:
        return devices[0]  # This baseline supports a sole attached Gemini.
    if devices:
        raise RuntimeError(f"multiple Gemini cameras found ({devices}); select an exact serial")
    return None


def reattach_uvc(device, dev_root=Path("/dev/bus/usb")):
    """Restore only unbound UVC control interfaces left detached by SDK clients.

    Linux USBDEVFS_IOCTL/CONNECT, the operation used by libusb's
    attach_kernel_driver. No SDK, sudo, USB reset, detach or configuration change.
    A currently claimed interface is never forcibly taken from another process.
    """
    try:
        interfaces = [
            p
            for p in sorted(device.glob(device.name + ":*"))
            if (p / "bInterfaceClass").read_text().strip() == "0e"
            and (p / "bInterfaceSubClass").read_text().strip() == "01"
            and not (p / "driver").exists()
        ]
        if not interfaces:
            return
        path = (
            dev_root
            / f"{int((device / 'busnum').read_text()):03d}"
            / f"{int((device / 'devnum').read_text()):03d}"
        )
        fd = os.open(path, os.O_RDWR | os.O_CLOEXEC)
        try:
            for interface in interfaces:
                number = int((interface / "bInterfaceNumber").read_text(), 16)
                # struct usbdevfs_ioctl { int ifno; int ioctl_code; void *data; }
                arg = bytearray(struct.pack("@iiP", number, 0x5517, 0))
                request = 0xC0005512 | (len(arg) << 16)
                fcntl.ioctl(fd, request, arg)
                print(f"[camera] restored UVC driver: {interface.name}", flush=True)
        finally:
            os.close(fd)
    except OSError as exc:
        if exc.errno in (errno.ENOENT, errno.ENODEV, errno.EINTR):
            raise CameraNotReady(
                "Gemini disconnected while restoring UVC; waiting for replug"
            ) from exc
        if exc.errno == errno.EBUSY:
            raise RuntimeError(
                "Gemini USB interface is in use; close OrbbecViewer/other camera capture first"
            ) from exc
        raise RuntimeError(
            f"Gemini USB found but UVC driver restoration failed: {exc}. "
            "Check USB device permissions and that uvcvideo is loaded."
        ) from exc


def capture_formats(path):
    """Linux VIDIOC_ENUM_FMT (v4l2_fmtdesc, 64 bytes, VIDEO_CAPTURE=1)."""
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    try:
        formats = []
        for index in range(64):
            descriptor = bytearray(struct.pack("=II", index, 1) + bytes(56))
            try:
                fcntl.ioctl(fd, 0xC0405602, descriptor)
            except OSError as exc:
                if exc.errno == errno.EINVAL:
                    break
                raise
            formats.append(bytes(descriptor[44:48]).decode("ascii"))
        return formats
    finally:
        os.close(fd)


def discover_rgb_device(
    serial="", sys_root=Path("/sys/class/video4linux"), dev_root=Path("/dev"), usb_device=None
):
    matches = []
    for node in sorted(sys_root.glob("video*")):
        try:
            if usb_device is not None and usb_device not in node.resolve().parents:
                continue
            if "gemini" not in (node / "name").read_text().lower():
                continue
            serial_paths = [
                p / "serial" for p in node.resolve().parents if (p / "serial").is_file()
            ]
            found_serial = serial_paths[0].read_text().strip() if serial_paths else ""
            path = dev_root / node.name
            if "MJPG" in capture_formats(path):
                matches.append((str(path), found_serial))
        except OSError as exc:
            if exc.errno in (errno.ENOENT, errno.ENODEV):
                continue
            raise RuntimeError(f"cannot inspect Gemini RGB node {node.name}: {exc}") from exc
    exact = [match for match in matches if serial and match[1] == serial]
    if len(exact) == 1:
        return exact[0]
    if not matches:
        raise CameraNotReady("no Gemini RGB MJPG node yet; waiting for USB/V4L2 enumeration")
    if len(matches) != 1:
        raise RuntimeError(
            f"expected one Gemini RGB MJPG node for serial {serial!r}, found {matches}"
        )
    return matches[0]


def prepare_rgb_device(serial=""):
    device = select_usb_device(serial)
    if device is None:
        raise CameraNotReady("no Gemini USB camera detected; waiting for connection")
    reattach_uvc(device)
    # Scope discovery to the chosen physical device, avoiding partial enumeration
    # of another camera when multiple USB devices are present.
    path, actual_serial = discover_rgb_device(serial, usb_device=device.resolve())
    return path, actual_serial


def open_rgb_stream(args, stop, report_pending, cv2):
    """Retry enumeration/open/first-frame failures only BEFORE a session starts."""
    deadline = time.monotonic() + args.startup_timeout_s
    last_error = "waiting for camera"
    while not stop.is_set():
        if time.monotonic() >= deadline:
            raise RuntimeError(f"camera startup timed out: {last_error}")
        cap = None
        try:
            device, serial = (
                (args.device, args.serial) if args.device else prepare_rgb_device(args.serial)
            )
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not cap.isOpened():
                if args.device:
                    raise RuntimeError(f"cannot open RGB device {device}")
                raise CameraNotReady(f"cannot open {device} yet; waiting for RGB device")
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            cap.set(cv2.CAP_PROP_FPS, args.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ok, bgr = cap.read()
            received_ns = time.monotonic_ns()
            if not ok or bgr is None:
                raise CameraNotReady(
                    f"{device} did not deliver its first RGB frame; reopening stream"
                )
            print(
                f"[camera] RGB {device}, serial={serial}, requested_serial={args.serial}",
                flush=True,
            )
            return cap, device, serial, bgr, received_ns
        except CameraNotReady as exc:
            if cap is not None:
                cap.release()
            last_error = str(exc)
            report_pending(last_error)
            stop.wait(0.2)
        except BaseException:
            if cap is not None:
                cap.release()
            raise
    raise RuntimeError("camera startup cancelled")


def main():
    # OpenCV reads this before its first V4L2 select. Bound a missing first frame
    # so startup can reopen a stopped UVC stream within the overall deadline.
    os.environ["OPENCV_VIDEOIO_V4L_SELECT_TIMEOUT"] = "2"
    import cv2

    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", required=True)
    parser.add_argument("--serial", default="")
    parser.add_argument("--device", default="")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--startup-timeout-s", type=float, default=20)
    args = parser.parse_args()
    stop, lock = threading.Event(), threading.Lock()
    latest = {"error": "camera has not produced a frame", "pending": True}
    cap = None

    def report_pending(message):
        nonlocal latest
        with lock:
            latest = {"error": message, "pending": True}

    def capture():
        nonlocal latest, cap
        try:
            cap, device, actual_serial, bgr, received_ns = open_rgb_stream(
                args, stop, report_pending, cv2
            )
            sequence = 0
            while not stop.is_set():
                if bgr.shape != (args.height, args.width, 3):
                    raise RuntimeError(f"unexpected RGB shape: {bgr.shape}")
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                sequence += 1
                with lock:
                    latest = {
                        "rgb_bytes": rgb.tobytes(),
                        "shape": rgb.shape,
                        "host_monotonic_ns": received_ns,
                        "device_timestamp_ns": None,
                        "timestamp_source": "host_receive_after_v4l2_read",
                        "sequence": sequence,
                        "serial": actual_serial,
                        "device": device,
                        "fps": cap.get(cv2.CAP_PROP_FPS),
                        "source": "gemini335_uvc_rgb",
                    }
                ok, bgr = cap.read()
                received_ns = time.monotonic_ns()
                if not ok or bgr is None:
                    raise RuntimeError(
                        "Gemini335 RGB stream disconnected/read failed; restart after reconnecting"
                    )
        except Exception as exc:
            with lock:
                latest = {"error": str(exc)}
        finally:
            if cap is not None:
                cap.release()

    with Listener(args.socket, family="AF_UNIX") as listener:
        with listener.accept() as conn:
            worker = threading.Thread(target=capture, daemon=True)
            try:
                worker.start()
                while conn.recv() == "snapshot":
                    with lock:
                        packet = latest.copy()
                    conn.send(packet)
            except EOFError:
                pass
            except Exception as exc:
                try:
                    conn.send({"error": str(exc)})
                except (BrokenPipeError, EOFError):
                    pass
            finally:
                stop.set()
                worker.join(timeout=2)
                # Allow normal capture to release/STREAMOFF before process exit;
                # parent still terminates a wedged driver read after its deadline.
                # Process exit releases the camera even if its capture thread is
                # stuck in a kernel read; parent enforces the shutdown deadline.


if __name__ == "__main__":
    main()
