"""Depth, infrared, and point-cloud conversions for the DaBai Max."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from io import BytesIO
from typing import Iterable, Sequence

import numpy as np

DEPTH_1_MM = 100
DEPTH_100_UM = 101
RGB888 = 200
YUV422 = 201
GRAY8 = 202
GRAY16 = 203
JPEG = 204
YUYV = 205

DEPTH_FORMATS = (DEPTH_1_MM, DEPTH_100_UM)
IR_FORMATS = (GRAY8, GRAY16)
COLOR_FORMATS = (JPEG, RGB888, YUV422, YUYV)

# Rated optical field of view from the DaBai Max product page, used when the
# device does not report a usable field of view.
DEFAULT_DEPTH_HFOV_DEG = 110.0
DEFAULT_DEPTH_VFOV_DEG = 71.0


@dataclass(frozen=True)
class VideoMode:
    pixel_format: int
    width: int
    height: int
    fps: int


@dataclass(frozen=True)
class CameraConfig:
    serial_number: str
    device_uri: str
    openni2_redist: str
    width: int
    height: int
    fps: int
    ir_width: int
    ir_height: int
    ir_fps: int
    min_depth_mm: int
    max_depth_mm: int
    point_cloud_stride: int


def parse_attributes(attrs: dict) -> CameraConfig:
    min_depth_mm = _positive_or_zero(attrs, "min_depth_mm", 200)
    max_depth_mm = _positive_or_zero(attrs, "max_depth_mm", 2500)
    if min_depth_mm and max_depth_mm and min_depth_mm > max_depth_mm:
        raise ValueError("min_depth_mm must be less than or equal to max_depth_mm")

    return CameraConfig(
        serial_number=str(attrs.get("serial_number", "") or ""),
        device_uri=str(attrs.get("device_uri", "") or ""),
        openni2_redist=str(attrs.get("openni2_redist", "") or ""),
        width=_positive(attrs, "width", 640),
        height=_positive(attrs, "height", 320),
        fps=_positive(attrs, "fps", 10),
        ir_width=_positive(attrs, "ir_width", 640),
        ir_height=_positive(attrs, "ir_height", 400),
        ir_fps=_positive(attrs, "ir_fps", 10),
        min_depth_mm=min_depth_mm,
        max_depth_mm=max_depth_mm,
        point_cloud_stride=_positive(attrs, "point_cloud_stride", 1),
    )


def choose_video_mode(
    modes: Sequence[VideoMode],
    width: int,
    height: int,
    fps: int,
    preferred_formats: Sequence[int],
) -> VideoMode:
    if not modes:
        raise ValueError("camera reported no video modes")

    def sort_key(mode: VideoMode) -> tuple[int, int, int, int]:
        exact = 0 if (mode.width == width and mode.height == height and mode.fps == fps) else 1
        size_penalty = abs(mode.width - width) + abs(mode.height - height)
        try:
            format_rank = preferred_formats.index(mode.pixel_format)
        except ValueError:
            format_rank = len(preferred_formats)
        return (exact, size_penalty, format_rank, abs(mode.fps - fps))

    return min(modes, key=sort_key)


def intrinsics_from_fov(width: int, height: int, hfov_rad: float, vfov_rad: float) -> tuple[float, float, float, float]:
    """Return fx, fy, cx, cy for a pinhole camera.

    OpenNI reports field of view in radians. Some Orbbec drivers report degrees
    instead, so values larger than pi are treated as degrees.
    """
    hfov = _as_radians(hfov_rad, DEFAULT_DEPTH_HFOV_DEG)
    vfov = _as_radians(vfov_rad, DEFAULT_DEPTH_VFOV_DEG)
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    fy = (height / 2.0) / math.tan(vfov / 2.0)
    return fx, fy, width / 2.0, height / 2.0


def depth_to_color_jpeg(depth_mm: np.ndarray) -> bytes:
    """Render depth as a bright false-color JPEG.

    A raw depth value of a few thousand millimeters is almost black when it is
    stored in a 16-bit image and then turned into a JPEG. This stretches the
    measured range across a full-brightness hue sweep instead. Near is warm,
    far is cool, and unmeasured pixels stay black.
    """
    if depth_mm.ndim != 2:
        raise ValueError("depth map must be a 2-D array")
    valid = depth_mm > 0
    rgb = np.zeros((depth_mm.shape[0], depth_mm.shape[1], 3), dtype=np.uint8)
    if np.any(valid):
        samples = depth_mm[valid].astype(np.float32)
        near = float(samples.min())
        span = max(float(samples.max()) - near, 1.0)
        ratio = (depth_mm.astype(np.float32) - np.float32(near)) / np.float32(span)
        rgb[valid] = _hue_to_rgb((30.0 + 200.0 * ratio)[valid])
    return _jpeg(rgb)


def encode_depth_map(depth_mm: np.ndarray) -> bytes:
    """Encode a uint16 millimeter depth map as image/vnd.viam.dep."""
    if depth_mm.ndim != 2:
        raise ValueError("depth map must be a 2-D array")
    height, width = depth_mm.shape
    pixels = np.ascontiguousarray(depth_mm, dtype="<u2").astype(">u2")
    header = b"DEPTHMAP" + struct.pack(">QQ", width, height)
    return header + pixels.tobytes()


def depth_frame_to_mm(data: bytes, width: int, height: int, stride: int, pixel_format: int) -> np.ndarray:
    row_bytes = stride if stride > 0 else width * 2
    needed = row_bytes * height
    if len(data) < needed:
        raise ValueError(f"depth frame is {len(data)} bytes, expected at least {needed}")
    if row_bytes == width * 2:
        depth = np.frombuffer(data[:needed], dtype="<u2").reshape(height, width).copy()
    else:
        rows = [
            np.frombuffer(data[y * row_bytes : y * row_bytes + width * 2], dtype="<u2").copy()
            for y in range(height)
        ]
        depth = np.stack(rows)
    if pixel_format == DEPTH_100_UM:
        depth //= np.uint16(10)
    return depth


def depth_to_points(
    depth_mm: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    min_depth_mm: int,
    max_depth_mm: int,
    stride: int,
) -> np.ndarray:
    """Project a depth map to an Nx3 point cloud in millimeters.

    The camera frame is +X right, +Y down, and +Z forward.
    """
    if stride < 1:
        raise ValueError("stride must be at least 1")
    sampled = depth_mm[::stride, ::stride]
    rows, cols = np.indices(sampled.shape)
    z = sampled.astype(np.float32)
    valid = z > 0
    if min_depth_mm > 0:
        valid &= z >= min_depth_mm
    if max_depth_mm > 0:
        valid &= z <= max_depth_mm
    z = z[valid]
    u = cols[valid].astype(np.float32) * stride
    v = rows[valid].astype(np.float32) * stride
    x = (u - cx) * z / np.float32(fx)
    y = (v - cy) * z / np.float32(fy)
    return np.column_stack((x, y, z)).astype(np.float32, copy=False)


def encode_pcd(points_mm: np.ndarray) -> bytes:
    """Encode an Nx3 point cloud.

    ``points_mm`` is in millimeters. Viam's PCD reader treats file coordinates as
    meters and multiplies them by 1000, so the file stores meters.
    """
    count = 0 if points_mm.size == 0 else int(points_mm.shape[0])
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION .7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        "DATA binary\n"
    ).encode("ascii")
    if count == 0:
        return header
    meters = np.ascontiguousarray(points_mm, dtype="<f4") / np.float32(1000.0)
    return header + meters.tobytes()


def ir_frame_to_jpeg(data: bytes, width: int, height: int, stride: int, pixel_format: int) -> bytes:
    gray = _gray_image(data, width, height, stride, pixel_format)
    return _jpeg(gray)


def color_frame_to_jpeg(data: bytes, width: int, height: int, stride: int, pixel_format: int) -> bytes:
    if pixel_format == JPEG:
        return data
    if pixel_format == RGB888:
        row_bytes = stride if stride > 0 else width * 3
        if row_bytes == width * 3:
            rgb = np.frombuffer(data, dtype=np.uint8, count=width * height * 3).reshape(height, width, 3)
        else:
            rows = []
            for y in range(height):
                start = y * row_bytes
                rows.append(np.frombuffer(data[start : start + width * 3], dtype=np.uint8).reshape(width, 3))
            rgb = np.stack(rows)
        return _jpeg(rgb)
    if pixel_format == YUV422:
        return _jpeg(_uyvy_to_rgb(data, width, height))
    if pixel_format == YUYV:
        return _jpeg(_yuyv_to_rgb(data, width, height))
    raise ValueError(f"unsupported color pixel format {pixel_format}")


def _gray_image(data: bytes, width: int, height: int, stride: int, pixel_format: int) -> np.ndarray:
    if pixel_format not in (GRAY8, GRAY16):
        pixel_format = GRAY16 if len(data) >= width * height * 2 else GRAY8
    if pixel_format == GRAY8:
        row_bytes = stride if stride > 0 else width
        if row_bytes == width:
            return np.frombuffer(data, dtype=np.uint8, count=width * height).reshape(height, width)
        rows = [np.frombuffer(data[y * row_bytes : y * row_bytes + width], dtype=np.uint8) for y in range(height)]
        return np.stack(rows)
    if pixel_format == GRAY16:
        row_bytes = stride if stride > 0 else width * 2
        if row_bytes == width * 2:
            gray16 = np.frombuffer(data, dtype="<u2", count=width * height).reshape(height, width)
        else:
            rows = [
                np.frombuffer(data[y * row_bytes : y * row_bytes + width * 2], dtype="<u2") for y in range(height)
            ]
            gray16 = np.stack(rows)
        peak = int(gray16.max()) if gray16.size else 1
        peak = max(peak, 1)
        return (gray16.astype(np.uint32) * 255 // peak).astype(np.uint8)
    raise ValueError(f"unsupported infrared pixel format {pixel_format}")


def _yuyv_to_rgb(data: bytes, width: int, height: int) -> np.ndarray:
    return _packed_422_to_rgb(data, width, height, y0=0, u=1, y1=2, v=3)


def _uyvy_to_rgb(data: bytes, width: int, height: int) -> np.ndarray:
    # OpenNI ONI_PIXEL_FORMAT_YUV422 stores each pixel pair as U Y0 V Y1.
    return _packed_422_to_rgb(data, width, height, y0=1, u=0, y1=3, v=2)


def _packed_422_to_rgb(data: bytes, width: int, height: int, y0: int, u: int, y1: int, v: int) -> np.ndarray:
    packed = np.frombuffer(data, dtype=np.uint8, count=width * height * 2).reshape(height, width // 2, 4)
    y0_plane = packed[:, :, y0].astype(np.float32)
    u_plane = packed[:, :, u].astype(np.float32) - 128.0
    y1_plane = packed[:, :, y1].astype(np.float32)
    v_plane = packed[:, :, v].astype(np.float32) - 128.0
    rgb = np.empty((height, width, 3), dtype=np.uint8)
    for offset, y in ((0, y0_plane), (1, y1_plane)):
        r = y + 1.402 * v_plane
        g = y - 0.344136 * u_plane - 0.714136 * v_plane
        b = y + 1.772 * u_plane
        rgb[:, offset::2, 0] = np.clip(r, 0, 255)
        rgb[:, offset::2, 1] = np.clip(g, 0, 255)
        rgb[:, offset::2, 2] = np.clip(b, 0, 255)
    return rgb


def _hue_to_rgb(hue: np.ndarray) -> np.ndarray:
    sector = hue / 60.0
    cross = np.rint((1.0 - np.abs(np.mod(sector, 2.0) - 1.0)) * 255.0)
    red = np.zeros(hue.shape, dtype=np.float32)
    green = np.zeros(hue.shape, dtype=np.float32)
    blue = np.zeros(hue.shape, dtype=np.float32)
    bands = (
        (sector < 1, 255.0, cross, 0.0),
        ((sector >= 1) & (sector < 2), cross, 255.0, 0.0),
        ((sector >= 2) & (sector < 3), 0.0, 255.0, cross),
        ((sector >= 3) & (sector < 4), 0.0, cross, 255.0),
        ((sector >= 4) & (sector < 5), cross, 0.0, 255.0),
        (sector >= 5, 255.0, 0.0, cross),
    )
    for mask, red_value, green_value, blue_value in bands:
        red = np.where(mask, red_value, red)
        green = np.where(mask, green_value, green)
        blue = np.where(mask, blue_value, blue)
    return np.stack((red, green, blue), axis=-1).astype(np.uint8)


def _jpeg(image: np.ndarray) -> bytes:
    from PIL import Image

    mode = "L" if image.ndim == 2 else "RGB"
    buffer = BytesIO()
    Image.fromarray(image, mode=mode).save(buffer, format="JPEG", quality=80)
    return buffer.getvalue()


def _as_radians(value: float, fallback_deg: float) -> float:
    if not math.isfinite(value) or value <= 0:
        value = math.radians(fallback_deg)
    elif value > math.pi:
        value = math.radians(value)
    return value


def _positive(attrs: dict, name: str, default: int) -> int:
    value = int(attrs.get(name, default))
    if value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _positive_or_zero(attrs: dict, name: str, default: int) -> int:
    value = int(attrs.get(name, default))
    if value < 0:
        raise ValueError(f"{name} must be zero or a positive integer")
    return value


def format_name(pixel_format: int) -> str:
    names = {
        DEPTH_1_MM: "depth_1_mm",
        DEPTH_100_UM: "depth_100_um",
        RGB888: "rgb888",
        YUV422: "yuv422",
        GRAY8: "gray8",
        GRAY16: "gray16",
        JPEG: "jpeg",
        YUYV: "yuyv",
    }
    return names.get(pixel_format, str(pixel_format))


def describe_modes(modes: Iterable[VideoMode]) -> str:
    return ", ".join(
        f"{mode.width}x{mode.height}@{mode.fps} {format_name(mode.pixel_format)}" for mode in modes
    )
