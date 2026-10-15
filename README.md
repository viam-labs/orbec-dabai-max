# DaBai Max

Viam camera module for the [Orbbec DaBai Max](https://www.orbbec.com.cn/index/Product/info.html?cate=38&id=56), a wide-angle stereo structured-light depth camera. It implements `rdk:component:camera` and returns a depth map and a point cloud.

The DaBai Max in current firmware uses the OpenNI protocol. Viam's `viam:orbbec` module talks to Orbbec SDK v2, which does not support this camera until Orbbec ships a UVC firmware update. This module uses OpenNI 2 instead.

Depth-to-color alignment is not supported on this camera. The module does not try to colorize the point cloud.

## Model viam-labs:camera:dabai-max

The depth stream uses Orbbec's DaBai Max profile, 640×320 at 10 fps. The working range filter is 0.2 m to 2.5 m.

No camera attributes are required. The module opens the depth stream and builds the point cloud from it.

`get_images` source names, in the order they are returned:

| Source | MIME type | Picture |
| --- | --- | --- |
| `color` | `image/jpeg` | False-color depth at full brightness. Near is warm and far is cool. This is the picture in the control tab. |
| `depth` | `image/vnd.viam.dep` | Raw depth in millimeters. |

The DaBai Max (`2bc5:069a`) has no RGB sensor. Sending the raw depth map as the preview JPEG looks black, because a few thousand millimeters is a tiny fraction of a 16-bit range. `color` stretches the measured depths across the full color range instead.

Point clouds use a camera frame where +X is right, +Y is down, and +Z is forward. Distances are millimeters inside Viam.

| Attribute | Type | Default | Description |
| --- | --- | --- | --- |
| `serial_number` | string | | Open this camera when more than one OpenNI device is connected. |
| `device_uri` | string | | OpenNI device URI. Overrides `serial_number`. |
| `openni2_redist` | string | `$OPENNI2_REDIST` | Directory that contains `libOpenNI2.so` and `OpenNI2/Drivers`. |
| `width`, `height`, `fps` | int | `640`, `320`, `10` | Requested depth mode. The nearest mode the camera reports is used. |
| `min_depth_mm`, `max_depth_mm` | int | `200`, `2500` | Point-cloud range filter. `0` disables that side of the filter. If every pixel falls outside the range, the cloud uses every measured pixel. |
| `point_cloud_stride` | int | `1` | Sample every Nth depth pixel when building a point cloud. |

`do_command` accepts `{"command": "get_device_info"}` and returns the OpenNI name, vendor, URI, and serial number.

## Machine setup

Add the module to the machine and configure a camera with model `viam-labs:camera:dabai-max`. On first run, `setup.sh` installs the Python packages, `libusb`, Orbbec OpenNI SDK 2.3.0.86, and the udev rule. The machine needs passwordless `sudo` for the package and udev steps. Replug the camera after that first install.

Set `openni2_redist` only when the SDK is already installed somewhere else. The module is built for `linux/amd64` and `linux/arm64`.

## Local checks

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt pytest
.venv/bin/python -m pytest
```

These tests cover frame encoding and video-mode selection. They do not open a camera.
