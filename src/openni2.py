"""Small OpenNI 2 binding for the Orbbec DaBai Max.

DaBai Max cameras currently speak the OpenNI protocol. Orbbec SDK v2 support
is planned for a later UVC firmware and is not what units in the field use.
"""

from __future__ import annotations

import ctypes
import os
import threading
from pathlib import Path

from frames import VideoMode

ONI_STATUS_OK = 0
SENSOR_IR = 1
SENSOR_COLOR = 2
SENSOR_DEPTH = 3
ONI_DEVICE_PROPERTY_SERIAL_NUMBER = 3
ONI_STREAM_PROPERTY_HORIZONTAL_FOV = 1
ONI_STREAM_PROPERTY_VERTICAL_FOV = 2
ONI_STREAM_PROPERTY_VIDEO_MODE = 3
ONI_MAX_STR = 256
# OpenNI 2.2 and 2.3 both answer to API versions built as major * 1000 + minor.
_API_VERSIONS = (2002, 2003, 2000)

_LIBRARY_LOCK = threading.Lock()
_LIBRARY: ctypes.CDLL | None = None
_LIBRARY_USERS = 0


class OniVideoMode(ctypes.Structure):
    _fields_ = [
        ("pixelFormat", ctypes.c_int),
        ("resolutionX", ctypes.c_int),
        ("resolutionY", ctypes.c_int),
        ("fps", ctypes.c_int),
    ]


class OniSensorInfo(ctypes.Structure):
    _fields_ = [
        ("sensorType", ctypes.c_int),
        ("numSupportedVideoModes", ctypes.c_int),
        ("pSupportedVideoModes", ctypes.POINTER(OniVideoMode)),
    ]


class OniDeviceInfo(ctypes.Structure):
    _fields_ = [
        ("uri", ctypes.c_char * ONI_MAX_STR),
        ("vendor", ctypes.c_char * ONI_MAX_STR),
        ("name", ctypes.c_char * ONI_MAX_STR),
        ("usbVendorId", ctypes.c_uint16),
        ("usbProductId", ctypes.c_uint16),
    ]


class OniFrame(ctypes.Structure):
    _fields_ = [
        ("dataSize", ctypes.c_int),
        ("data", ctypes.c_void_p),
        ("sensorType", ctypes.c_int),
        ("timestamp", ctypes.c_uint64),
        ("frameIndex", ctypes.c_int),
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("videoMode", OniVideoMode),
        ("croppingEnabled", ctypes.c_int),
        ("cropOriginX", ctypes.c_int),
        ("cropOriginY", ctypes.c_int),
        ("stride", ctypes.c_int),
    ]


class Frame:
    def __init__(
        self, data: bytes, width: int, height: int, stride: int, pixel_format: int, timestamp: int, index: int
    ):
        self.data = data
        self.width = width
        self.height = height
        self.stride = stride
        self.pixel_format = pixel_format
        self.timestamp = timestamp
        self.index = index


class OpenNI2:
    def __init__(self, redist: str | None = None):
        self.lib = acquire_library(redist)
        self.device = ctypes.c_void_p()
        self.streams: dict[int, ctypes.c_void_p] = {}
        self.info: dict[str, str] = {}

    def list_devices(self) -> list[dict[str, str | int]]:
        devices = ctypes.POINTER(OniDeviceInfo)()
        count = ctypes.c_int()
        _check(self.lib, self.lib.oniGetDeviceList(ctypes.byref(devices), ctypes.byref(count)), "list devices")
        found: list[dict[str, str | int]] = []
        try:
            for index in range(count.value):
                item = devices[index]
                found.append(
                    {
                        "uri": _cstr(item.uri),
                        "vendor": _cstr(item.vendor),
                        "name": _cstr(item.name),
                        "usb_vendor_id": int(item.usbVendorId),
                        "usb_product_id": int(item.usbProductId),
                    }
                )
        finally:
            self.lib.oniReleaseDeviceList(devices)
        return found

    def open_device(self, serial_number: str = "", device_uri: str = "") -> None:
        devices = self.list_devices()
        if not devices:
            raise RuntimeError("no OpenNI camera was found; check the USB cable and udev rules")
        if device_uri:
            match = next((device for device in devices if device["uri"] == device_uri), None)
            if match is None:
                available = ", ".join(str(device["uri"]) for device in devices)
                raise RuntimeError(f"device_uri {device_uri!r} is not connected; available: {available}")
            self._open_uri(str(match["uri"]), str(match["name"]), str(match["vendor"]))
            return
        if serial_number:
            for device in devices:
                self._open_uri(str(device["uri"]), str(device["name"]), str(device["vendor"]))
                if self.info.get("serial_number") == serial_number:
                    return
                self.close_device()
            raise RuntimeError(f"no OpenNI camera with serial number {serial_number!r} was found")
        if len(devices) > 1:
            names = ", ".join(f"{device['name']} ({device['uri']})" for device in devices)
            raise RuntimeError(
                f"multiple OpenNI cameras are connected; set serial_number or device_uri. Found: {names}"
            )
        only = devices[0]
        self._open_uri(str(only["uri"]), str(only["name"]), str(only["vendor"]))

    def sensor_modes(self, sensor_type: int) -> list[VideoMode]:
        info = self.lib.oniDeviceGetSensorInfo(self.device, sensor_type)
        if not info:
            return []
        count = int(info.contents.numSupportedVideoModes)
        modes = info.contents.pSupportedVideoModes
        return [
            VideoMode(
                int(modes[index].pixelFormat),
                int(modes[index].resolutionX),
                int(modes[index].resolutionY),
                int(modes[index].fps),
            )
            for index in range(count)
        ]

    def start_stream(self, sensor_type: int, mode: VideoMode) -> None:
        stream = ctypes.c_void_p()
        _check(
            self.lib,
            self.lib.oniDeviceCreateStream(self.device, sensor_type, ctypes.byref(stream)),
            "create stream",
        )
        video_mode = OniVideoMode(mode.pixel_format, mode.width, mode.height, mode.fps)
        status = self.lib.oniStreamSetProperty(
            stream,
            ONI_STREAM_PROPERTY_VIDEO_MODE,
            ctypes.byref(video_mode),
            ctypes.sizeof(video_mode),
        )
        if status != ONI_STATUS_OK:
            self.lib.oniStreamDestroy(stream)
            _check(self.lib, status, f"set video mode {mode.width}x{mode.height}@{mode.fps}")
        _check(self.lib, self.lib.oniStreamStart(stream), "start stream")
        self.streams[sensor_type] = stream

    def read_frame(self, sensor_type: int, timeout_ms: int) -> Frame:
        stream = self.streams[sensor_type]
        handles = (ctypes.c_void_p * 1)(stream)
        index = ctypes.c_int(-1)
        _check(
            self.lib,
            self.lib.oniWaitForAnyStream(handles, 1, ctypes.byref(index), int(timeout_ms)),
            "wait for frame",
        )
        frame_ptr = ctypes.POINTER(OniFrame)()
        _check(self.lib, self.lib.oniStreamReadFrame(stream, ctypes.byref(frame_ptr)), "read frame")
        try:
            frame = frame_ptr.contents
            if not frame.data or frame.dataSize <= 0:
                raise RuntimeError("OpenNI returned an empty frame")
            payload = ctypes.string_at(frame.data, frame.dataSize)
            return Frame(
                data=payload,
                width=int(frame.width),
                height=int(frame.height),
                stride=int(frame.stride),
                pixel_format=int(frame.videoMode.pixelFormat),
                timestamp=int(frame.timestamp),
                index=int(frame.frameIndex),
            )
        finally:
            self.lib.oniFrameRelease(frame_ptr)

    def field_of_view(self, sensor_type: int) -> tuple[float, float]:
        stream = self.streams[sensor_type]
        return (
            _float_property(self.lib, stream, ONI_STREAM_PROPERTY_HORIZONTAL_FOV),
            _float_property(self.lib, stream, ONI_STREAM_PROPERTY_VERTICAL_FOV),
        )

    def close_device(self) -> None:
        for stream in self.streams.values():
            self.lib.oniStreamStop(stream)
            self.lib.oniStreamDestroy(stream)
        self.streams.clear()
        if self.device:
            self.lib.oniDeviceClose(self.device)
            self.device = ctypes.c_void_p()

    def close(self) -> None:
        self.close_device()
        release_library()

    def _open_uri(self, uri: str, name: str, vendor: str) -> None:
        handle = ctypes.c_void_p()
        _check(self.lib, self.lib.oniDeviceOpen(uri.encode("utf-8"), ctypes.byref(handle)), f"open {uri}")
        self.device = handle
        self.info = {
            "uri": uri,
            "name": name,
            "vendor": vendor,
            "serial_number": _serial_number(self.lib, handle),
        }


def acquire_library(redist: str | None) -> ctypes.CDLL:
    global _LIBRARY, _LIBRARY_USERS
    with _LIBRARY_LOCK:
        if _LIBRARY is None:
            loaded = _load_library(redist)
            _initialize(loaded)
            _LIBRARY = loaded
        _LIBRARY_USERS += 1
        return _LIBRARY


def release_library() -> None:
    global _LIBRARY, _LIBRARY_USERS
    with _LIBRARY_LOCK:
        if _LIBRARY_USERS == 0:
            return
        _LIBRARY_USERS -= 1
        if _LIBRARY_USERS == 0 and _LIBRARY is not None:
            _LIBRARY.oniShutdown()
            _LIBRARY = None


def installed_redist_roots() -> list[Path]:
    """Directories written by setup.sh after it downloads the OpenNI SDK."""
    roots: list[Path] = []
    data = os.environ.get("VIAM_MODULE_DATA")
    if not data:
        return roots
    install_root = Path(data) / "openni"
    marker = install_root / "redist.path"
    if marker.is_file():
        recorded = marker.read_text(encoding="utf-8").strip()
        if recorded:
            roots.append(Path(recorded))
    roots.append(install_root)
    return roots


def library_candidates(redist: str | None) -> list[Path]:
    roots: list[Path] = []
    if redist:
        roots.append(Path(redist))
    env = os.environ.get("OPENNI2_REDIST")
    if env:
        roots.append(Path(env))
    roots.extend(installed_redist_roots())
    roots.extend(
        [
            Path("/usr/lib"),
            Path("/usr/local/lib"),
            Path("/usr/lib/x86_64-linux-gnu"),
            Path("/usr/lib/aarch64-linux-gnu"),
        ]
    )
    names = ("libOpenNI2.so", "libOpenNI2.dylib")
    candidates: list[Path] = []
    for root in roots:
        for name in names:
            candidates.append(root / name)
            candidates.append(root / "Redist" / name)
    return candidates


def _load_library(redist: str | None) -> ctypes.CDLL:
    mode = getattr(ctypes, "RTLD_GLOBAL", 0)
    for candidate in library_candidates(redist):
        if candidate.is_file():
            return ctypes.CDLL(str(candidate), mode=mode)
    searched = ", ".join(str(path) for path in library_candidates(redist))
    raise RuntimeError(
        "libOpenNI2 was not found. setup.sh downloads it on first run; "
        "set openni2_redist or OPENNI2_REDIST to skip that copy. "
        f"Looked in: {searched}"
    )


def _initialize(lib: ctypes.CDLL) -> None:
    _bind(lib)
    last_error = "unknown error"
    for version in _API_VERSIONS:
        status = lib.oniInitialize(version)
        if status == ONI_STATUS_OK:
            lib.oniSetLogConsoleOutput(0)
            return
        last_error = _extended_error(lib) or f"status {status}"
        lib.oniShutdown()
    raise RuntimeError(f"OpenNI failed to initialize: {last_error}")


def _bind(lib: ctypes.CDLL) -> None:
    lib.oniInitialize.argtypes = [ctypes.c_int]
    lib.oniInitialize.restype = ctypes.c_int
    lib.oniShutdown.argtypes = []
    lib.oniShutdown.restype = None
    lib.oniGetExtendedError.argtypes = []
    lib.oniGetExtendedError.restype = ctypes.c_char_p
    lib.oniGetDeviceList.argtypes = [ctypes.POINTER(ctypes.POINTER(OniDeviceInfo)), ctypes.POINTER(ctypes.c_int)]
    lib.oniGetDeviceList.restype = ctypes.c_int
    lib.oniReleaseDeviceList.argtypes = [ctypes.POINTER(OniDeviceInfo)]
    lib.oniReleaseDeviceList.restype = ctypes.c_int
    lib.oniDeviceOpen.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_void_p)]
    lib.oniDeviceOpen.restype = ctypes.c_int
    lib.oniDeviceClose.argtypes = [ctypes.c_void_p]
    lib.oniDeviceClose.restype = ctypes.c_int
    lib.oniDeviceGetSensorInfo.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.oniDeviceGetSensorInfo.restype = ctypes.POINTER(OniSensorInfo)
    lib.oniDeviceGetProperty.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
    lib.oniDeviceGetProperty.restype = ctypes.c_int
    lib.oniDeviceCreateStream.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_void_p)]
    lib.oniDeviceCreateStream.restype = ctypes.c_int
    lib.oniStreamDestroy.argtypes = [ctypes.c_void_p]
    lib.oniStreamDestroy.restype = None
    lib.oniStreamStart.argtypes = [ctypes.c_void_p]
    lib.oniStreamStart.restype = ctypes.c_int
    lib.oniStreamStop.argtypes = [ctypes.c_void_p]
    lib.oniStreamStop.restype = None
    lib.oniStreamSetProperty.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_int]
    lib.oniStreamSetProperty.restype = ctypes.c_int
    lib.oniStreamGetProperty.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
    lib.oniStreamGetProperty.restype = ctypes.c_int
    lib.oniWaitForAnyStream.argtypes = [
        ctypes.POINTER(ctypes.c_void_p),
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_int),
        ctypes.c_int,
    ]
    lib.oniWaitForAnyStream.restype = ctypes.c_int
    lib.oniStreamReadFrame.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(OniFrame))]
    lib.oniStreamReadFrame.restype = ctypes.c_int
    lib.oniFrameRelease.argtypes = [ctypes.POINTER(OniFrame)]
    lib.oniFrameRelease.restype = None
    lib.oniSetLogConsoleOutput.argtypes = [ctypes.c_int]
    lib.oniSetLogConsoleOutput.restype = ctypes.c_int


def _check(lib: ctypes.CDLL, status: int, action: str) -> None:
    if status == ONI_STATUS_OK:
        return
    detail = _extended_error(lib)
    message = f"OpenNI could not {action} (status {status})"
    if detail:
        message = f"{message}: {detail}"
    raise RuntimeError(message)


def _extended_error(lib: ctypes.CDLL) -> str:
    raw = lib.oniGetExtendedError()
    if not raw:
        return ""
    return raw.decode("utf-8", "replace").strip()


def _serial_number(lib: ctypes.CDLL, device: ctypes.c_void_p) -> str:
    buf = ctypes.create_string_buffer(ONI_MAX_STR)
    size = ctypes.c_int(ONI_MAX_STR)
    status = lib.oniDeviceGetProperty(device, ONI_DEVICE_PROPERTY_SERIAL_NUMBER, buf, ctypes.byref(size))
    if status != ONI_STATUS_OK:
        return ""
    return buf.value.decode("utf-8", "replace").strip()


def _float_property(lib: ctypes.CDLL, stream: ctypes.c_void_p, property_id: int) -> float:
    value = ctypes.c_float()
    size = ctypes.c_int(ctypes.sizeof(value))
    status = lib.oniStreamGetProperty(stream, property_id, ctypes.byref(value), ctypes.byref(size))
    if status != ONI_STATUS_OK:
        return 0.0
    return float(value.value)


def _cstr(value: bytes) -> str:
    return value.split(b"\x00", 1)[0].decode("utf-8", "replace")
