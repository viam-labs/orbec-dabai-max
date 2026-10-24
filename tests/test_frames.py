import struct
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from frames import (  # noqa: E402
    DEPTH_1_MM,
    DEPTH_100_UM,
    GRAY16,
    VideoMode,
    choose_video_mode,
    depth_frame_to_mm,
    depth_to_color_jpeg,
    depth_to_points,
    encode_depth_map,
    encode_pcd,
    intrinsics_from_fov,
    ir_frame_to_jpeg,
    parse_attributes,
)
from openni2 import OniFrame, library_candidates  # noqa: E402


def test_parse_defaults_match_dabai_max_depth():
    config = parse_attributes({})
    assert (config.width, config.height, config.fps) == (640, 320, 10)
    assert (config.ir_width, config.ir_height, config.ir_fps) == (640, 400, 10)
    assert config.min_depth_mm == 200
    assert config.max_depth_mm == 2500


def test_parse_rejects_inverted_range():
    with pytest.raises(ValueError, match="min_depth_mm"):
        parse_attributes({"min_depth_mm": 2000, "max_depth_mm": 100})


def test_choose_video_mode_prefers_exact_millimeter_depth():
    modes = [
        VideoMode(DEPTH_100_UM, 640, 320, 10),
        VideoMode(DEPTH_1_MM, 320, 160, 10),
        VideoMode(DEPTH_1_MM, 640, 320, 10),
    ]
    chosen = choose_video_mode(modes, 640, 320, 10, (DEPTH_1_MM, DEPTH_100_UM))
    assert chosen == VideoMode(DEPTH_1_MM, 640, 320, 10)


def test_choose_video_mode_falls_back_to_nearest_resolution():
    modes = [VideoMode(DEPTH_1_MM, 320, 160, 10)]
    chosen = choose_video_mode(modes, 640, 320, 10, (DEPTH_1_MM,))
    assert chosen.width == 320
    assert chosen.height == 160


def test_depth_encoding_round_trip_is_big_endian():
    depth = np.array([[1000, 0], [2500, 20]], dtype=np.uint16)
    encoded = encode_depth_map(depth)
    assert encoded[:8] == b"DEPTHMAP"
    width, height = struct.unpack(">QQ", encoded[8:24])
    assert (width, height) == (2, 2)
    pixels = np.frombuffer(encoded[24:], dtype=">u2").reshape(2, 2)
    np.testing.assert_array_equal(pixels, depth)


def test_depth_color_jpeg_is_bright_for_ordinary_millimeter_values():
    from io import BytesIO

    from PIL import Image

    depth = np.zeros((32, 32), dtype=np.uint16)
    depth[8:24, 8:24] = 1000
    encoded = depth_to_color_jpeg(depth)
    assert encoded[:2] == b"\xff\xd8"
    image = np.asarray(Image.open(BytesIO(encoded)))
    pixel = image[16, 16].astype(int)
    assert int(pixel.max()) > 200
    assert int(image[2, 2].max()) < 20


def test_depth_100_um_converts_to_millimeters():
    raw = np.array([[10000, 25000]], dtype="<u2").tobytes()
    depth = depth_frame_to_mm(raw, 2, 1, 4, DEPTH_100_UM)
    np.testing.assert_array_equal(depth, np.array([[1000, 2500]], dtype=np.uint16))


def test_point_cloud_uses_optical_frame_and_drops_out_of_range_pixels():
    depth = np.array(
        [
            [0, 100, 0],
            [1000, 1000, 3000],
            [0, 500, 0],
        ],
        dtype=np.uint16,
    )
    points = depth_to_points(
        depth, fx=100, fy=100, cx=1, cy=1, min_depth_mm=200, max_depth_mm=2500, stride=1
    )
    # 100 mm is below the minimum and 3000 mm is above the maximum.
    # The remaining pixels are one step left of center, the center, and one step down.
    expected = np.array(
        [
            [-10, 0, 1000],
            [0, 0, 1000],
            [0, 5, 500],
        ],
        dtype=np.float32,
    )
    np.testing.assert_allclose(points, expected)


def test_pcd_stores_meters_for_viam():
    points = np.array([[1000, 2000, 3000], [0, 0, 500]], dtype=np.float32)
    encoded = encode_pcd(points)
    header, _, payload = encoded.partition(b"DATA binary\n")
    assert b"VERSION .7\n" in header
    assert b"POINTS 2" in header
    assert b"WIDTH 2" in header
    decoded = np.frombuffer(payload, dtype="<f4").reshape(2, 3)
    np.testing.assert_allclose(
        decoded, np.array([[1, 2, 3], [0, 0, 0.5]], dtype=np.float32)
    )


def test_intrinsics_accept_radians_or_degrees():
    radians = intrinsics_from_fov(640, 320, np.deg2rad(110), np.deg2rad(71))
    degrees = intrinsics_from_fov(640, 320, 110, 71)
    assert radians[0] == pytest.approx(degrees[0])
    assert radians[0] > 0
    assert radians[2] == pytest.approx(320)


def test_missing_fov_uses_dabai_max_datasheet():
    fx, fy, _, _ = intrinsics_from_fov(640, 320, 0, 0)
    expected_fx, expected_fy, _, _ = intrinsics_from_fov(640, 320, 110, 71)
    assert fx == pytest.approx(expected_fx)
    assert fy == pytest.approx(expected_fy)


def test_ir_gray16_jpeg_is_scaled():
    gray = np.array([[0, 1023], [512, 256]], dtype="<u2")
    encoded = ir_frame_to_jpeg(gray.tobytes(), 2, 2, 4, GRAY16)
    assert encoded[:2] == b"\xff\xd8"


def test_library_candidates_include_the_autoinstalled_sdk(tmp_path, monkeypatch):
    redist = tmp_path / "sdk" / "Redist"
    redist.mkdir(parents=True)
    install_root = tmp_path / "openni"
    install_root.mkdir()
    (install_root / "redist.path").write_text(f"{redist}\n", encoding="utf-8")
    monkeypatch.setenv("VIAM_MODULE_DATA", str(tmp_path))
    monkeypatch.delenv("OPENNI2_REDIST", raising=False)
    candidates = library_candidates(None)
    assert redist / "libOpenNI2.so" in candidates


def test_openni_frame_layout_matches_64_bit_abi():
    assert OniFrame.data.offset == 8
    assert OniFrame.timestamp.offset == 24
    assert OniFrame.width.offset == 36
    assert OniFrame.stride.offset == 72
