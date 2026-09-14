# udp_camera_ros

ROS 2 lifecycle node that receives H.264/RTP over UDP and publishes
`sensor_msgs/CompressedImage` (default) or raw `Image` via `image_transport`,
plus `sensor_msgs/CameraInfo`.

Decode uses **native GStreamer appsink** (no OpenCV `VideoCapture` / `cv_bridge`
on the hot path). Prefer hardware H.264 when available.

## Layout

```
include/udp_camera_ros/   public headers (Camera, UdpStream, calib helpers)
src/
  camera_node.cpp         lifecycle node entry
  camera.cpp              param → UdpStreamConfig adapter
  camera_info.cpp         YAML / UCAL1 calib parsing
  stream/                 GStreamer receive / publish / meta
```

## Launch

```bash
ros2 launch udp_camera_ros camera_stream.launch.py
```

Override parameters with your own YAML (topics, port, calibration):

```bash
ros2 launch udp_camera_ros camera_stream.launch.py \
  params_file:=/path/to/params.yaml \
  port:=5000
```

### CameraInfo source (`calib_source`)

| Value | Behavior |
|-------|----------|
| `auto` (default) | Use `calib_file` if set as fallback; prefer UCAL1 UDP meta when it arrives |
| `stream` | Intrinsics **only** from sideband meta (`meta_port`, default `port+1`) |
| `file` | Intrinsics only from `calib_file` |

Pi sender (`uv run python run_stream.py` in
`scripts/drone/rpi_gstreamer_camera`) emits UCAL1 JSON on `META_PORT`
(default `GS_PORT+1`) via `rpi-gstreamer-camera meta-send`, already scaled to
stream resolution and matching `UNDISTORT`. Settings live in `config/.env`. Stream meta already carries correct K/D for the live image
(`camera_info_from_calib`); file calib still uses `stream_undistorted` when
loading YAML.

### `calib_file`

Used for `file` / `auto` fallback. Accepts:

- a basename under `share/udp_camera_ros/config/`
- an absolute filesystem path
- a `package://pkg_name/relative/path.yaml` URL

### `publish_raw`

| Value | Behavior |
|-------|----------|
| `false` (default) | GStreamer `I420 → jpegenc` → `image_topic/compressed` + `camera_info` |
| `true` | Decode to BGR → `image_transport` CameraPublisher (raw + compressed plugins) |

`jpeg_quality` (1–100, default 80) applies to the GStreamer `jpegenc` path.

### `h264_decoder`

| Value | Behavior |
|-------|----------|
| `auto` (default) | Try `vah264dec` → `nvh264dec` → `avdec_h264` |
| `avdec_h264` | Force software (libav) |
| `vah264dec` | Force VA-API |
| `nvh264dec` | Force NVIDIA |

## Higher-resolution testing

```bash
# Pi
WIDTH=1536 HEIGHT=864 BITRATE=16000 uv run python run_stream.py
# (from scripts/drone/rpi_gstreamer_camera; or set values in config/.env)

# GS — calib rides on the meta port automatically
ros2 launch drone_bringup gs_bringup.launch.py
```

## Dependencies

See `apt_packages.txt` for GStreamer host packages (`libav` for SW decode;
VA-API / NVIDIA plugins optional for HW).
