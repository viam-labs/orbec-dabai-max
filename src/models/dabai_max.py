"""Viam camera component for the Orbbec DaBai Max."""

from __future__ import annotations

import asyncio
import threading
from datetime import datetime, timezone
from typing import Any, ClassVar, Mapping, Optional, Sequence, Tuple

from google.protobuf.timestamp_pb2 import Timestamp
from typing_extensions import Self

from frames import (
    DEPTH_FORMATS,
    IR_FORMATS,
    choose_video_mode,
    depth_frame_to_mm,
    depth_to_color_jpeg,
    depth_to_points,
    describe_modes,
    depth_to_color_jpeg,
    encode_depth_map,
    encode_pcd,
    intrinsics_from_fov,
    ir_frame_to_jpeg,
    parse_attributes,
)
from openni2 import SENSOR_DEPTH, SENSOR_IR, OpenNI2
from viam.components.camera import Camera, IntrinsicParameters
from viam.media.video import CameraMimeType, NamedImage
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, ResourceName, ResponseMetadata
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict

_SENSOR_TYPES = {"depth": SENSOR_DEPTH, "ir": SENSOR_IR}
_PREFERRED_FORMATS = {"depth": DEPTH_FORMATS, "ir": IR_FORMATS}


class DabaiMax(Camera, EasyResource):
    """Depth, infrared, and point clouds from an Orbbec DaBai Max."""

    MODEL: ClassVar[Model] = Model(ModelFamily("viam-labs", "camera"), "dabai-max")

    def __init__(self, name: str):
        super().__init__(name)
        self._lock = asyncio.Lock()
        self._device_lock = threading.Lock()
        self._camera: OpenNI2 | None = None
        self._config = parse_attributes({})
        self._streams: tuple[str, ...] = ()
        self._intrinsics: tuple[float, float, float, float] | None = None
        self._depth_shape: tuple[int, int] = (self._config.height, self._config.width)

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        camera = super().new(config, dependencies)
        camera._config = parse_attributes(struct_to_dict(config.attributes))
        camera._open_device()
        return camera

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        parse_attributes(struct_to_dict(config.attributes))
        return [], []

    async def get_images(
        self,
        *,
        filter_source_names: Optional[Sequence[str]] = None,
        extra: Optional[dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[Sequence[NamedImage], ResponseMetadata]:
        del extra, kwargs
        available = [name for name in ("color", "ir", "depth") if name == "color" or name in self._streams]
        requested = [name for name in available if not filter_source_names or name in filter_source_names]
        timeout_ms = int((timeout if timeout is not None else 5.0) * 1000)
        loop = asyncio.get_running_loop()
        async with self._lock:
            frames = await loop.run_in_executor(None, self._capture_sync, requested, timeout_ms)
        images = [NamedImage(sensor, payload, mime) for sensor, payload, mime in frames]
        captured = Timestamp()
        captured.FromDatetime(datetime.now(timezone.utc))
        return images, ResponseMetadata(captured_at=captured)

    async def get_point_cloud(
        self,
        *,
        extra: Optional[dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[bytes, str]:
        del extra, kwargs
        if self._intrinsics is None:
            raise NotImplementedError("point clouds require the depth sensor")
        timeout_ms = int((timeout if timeout is not None else 5.0) * 1000)
        loop = asyncio.get_running_loop()
        async with self._lock:
            cloud = await loop.run_in_executor(None, self._point_cloud_sync, timeout_ms)
        return cloud, CameraMimeType.PCD

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs) -> Camera.Properties:
        del timeout, kwargs
        mime_types = [str(CameraMimeType.JPEG)]
        if "depth" in self._streams:
            mime_types.append(str(CameraMimeType.VIAM_RAW_DEPTH))
        properties = Camera.Properties(
            supports_pcd=self._intrinsics is not None,
            mime_types=mime_types,
            frame_rate=float(self._config.fps),
        )
        if self._intrinsics is not None:
            fx, fy, cx, cy = self._intrinsics
            height, width = self._depth_shape
            properties.intrinsic_parameters.CopyFrom(
                IntrinsicParameters(
                    width_px=width,
                    height_px=height,
                    focal_x_px=fx,
                    focal_y_px=fy,
                    center_x_px=cx,
                    center_y_px=cy,
                )
            )
        return properties

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        del timeout, kwargs
        if command.get("command") != "get_device_info":
            return {}
        info = dict(self._camera.info) if self._camera is not None else {}
        return {key: value for key, value in info.items()}

    async def get_geometries(
        self, *, extra: Optional[dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ) -> Sequence[Geometry]:
        del extra, timeout, kwargs
        return []

    def stop(self, *, extra: Optional[Mapping[str, Any]] = None, timeout: Optional[float] = None, **kwargs):
        """Release the camera. Viam calls this before rebuilding the resource."""
        del extra, timeout, kwargs
        with self._device_lock:
            self._close_device()

    async def close(self):
        async with self._lock:
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(None, self.stop)

    def _open_device(self) -> None:
        camera = OpenNI2(self._config.openni2_redist or None)
        try:
            camera.open_device(self._config.serial_number, self._config.device_uri)
            self.logger.info(
                "opened %s serial=%s uri=%s",
                camera.info.get("name", "OpenNI camera"),
                camera.info.get("serial_number", ""),
                camera.info.get("uri", ""),
            )
            self._warn_if_unexpected_device(camera.info.get("name", ""))
            started: list[str] = []
            for sensor in ("ir", "depth"):
                try:
                    self._start_sensor(camera, sensor)
                except Exception as err:
                    if sensor == "depth":
                        raise
                    self.logger.warning("skipping %s: %s", sensor, err)
                    continue
                started.append(sensor)
            # The control tab displays the first JPEG. Lead with a false-color
            # view of the depth map, since this camera has no RGB sensor.
            if "depth" in started:
                started.insert(0, "color")
            self._streams = tuple(started)
            if "depth" in self._streams:
                hfov, vfov = camera.field_of_view(SENSOR_DEPTH)
                height, width = self._depth_shape
                self._intrinsics = intrinsics_from_fov(width, height, hfov, vfov)
                self.logger.info(
                    "depth intrinsics fx=%.2f fy=%.2f cx=%.2f cy=%.2f from fov h=%s v=%s",
                    *self._intrinsics,
                    hfov,
                    vfov,
                )
            self._camera = camera
        except Exception:
            camera.close()
            raise

    def _start_sensor(self, camera: OpenNI2, sensor: str) -> None:
        sensor_type = _SENSOR_TYPES[sensor]
        modes = camera.sensor_modes(sensor_type)
        if not modes:
            camera_name = camera.info.get("name", "camera")
            raise RuntimeError(
                f"{camera_name} has no {sensor} sensor. DaBai Max publishes depth and infrared; "
                "it does not support depth-to-color alignment."
            )
        width, height, fps = self._mode_request(sensor)
        mode = choose_video_mode(modes, width, height, fps, _PREFERRED_FORMATS[sensor])
        if (mode.width, mode.height, mode.fps) != (width, height, fps):
            self.logger.warning(
                "%s mode %sx%s@%s is not available; using %sx%s@%s. Available: %s",
                sensor,
                width,
                height,
                fps,
                mode.width,
                mode.height,
                mode.fps,
                describe_modes(modes),
            )
        camera.start_stream(sensor_type, mode)
        if sensor == "depth":
            self._depth_shape = (mode.height, mode.width)
        self.logger.info("started %s at %sx%s@%s", sensor, mode.width, mode.height, mode.fps)

    def _mode_request(self, sensor: str) -> tuple[int, int, int]:
        if sensor == "ir":
            return self._config.ir_width, self._config.ir_height, self._config.ir_fps
        return self._config.width, self._config.height, self._config.fps

    def _capture_sync(self, sensors: Sequence[str], timeout_ms: int) -> list[tuple[str, bytes, CameraMimeType]]:
        if self._camera is None:
            raise RuntimeError("DaBai Max is not open")
        images: list[tuple[str, bytes, CameraMimeType]] = []
        with self._device_lock:
            depth = None
            if "color" in sensors or "depth" in sensors:
                frame = self._camera.read_frame(SENSOR_DEPTH, timeout_ms)
                depth = depth_frame_to_mm(frame.data, frame.width, frame.height, frame.stride, frame.pixel_format)
                self._depth_shape = depth.shape
            if "color" in sensors and depth is not None:
                images.append(("color", depth_to_color_jpeg(depth), CameraMimeType.JPEG))
            if "ir" in sensors:
                frame = self._camera.read_frame(SENSOR_IR, timeout_ms)
                images.append(
                    (
                        "ir",
                        ir_frame_to_jpeg(frame.data, frame.width, frame.height, frame.stride, frame.pixel_format),
                        CameraMimeType.JPEG,
                    )
                )
            if "depth" in sensors and depth is not None:
                images.append(("depth", encode_depth_map(depth), CameraMimeType.VIAM_RAW_DEPTH))
        return images

    def _point_cloud_sync(self, timeout_ms: int) -> bytes:
        if self._camera is None or self._intrinsics is None:
            raise RuntimeError("DaBai Max depth stream is not open")
        with self._device_lock:
            frame = self._camera.read_frame(SENSOR_DEPTH, timeout_ms)
            depth = depth_frame_to_mm(frame.data, frame.width, frame.height, frame.stride, frame.pixel_format)
        fx, fy, cx, cy = self._intrinsics
        points = depth_to_points(
            depth,
            fx,
            fy,
            cx,
            cy,
            self._config.min_depth_mm,
            self._config.max_depth_mm,
            self._config.point_cloud_stride,
        )
        return encode_pcd(points)

    def _close_device(self) -> None:
        if self._camera is not None:
            self._camera.close()
            self._camera = None
        self._streams = ()
        self._intrinsics = None

    def _warn_if_unexpected_device(self, name: str) -> None:
        lowered = name.lower()
        if "dabai" in lowered and "max" in lowered:
            return
        self.logger.warning(
            "opened %r, which does not look like a DaBai Max; depth defaults are 640x320 at 10 fps",
            name,
        )

