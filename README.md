# udp_camera_ros

ROS 2 lifecycle node that receives H.264/RTP over UDP and republishes via
`image_transport` (raw + compressed) plus `sensor_msgs/CameraInfo`.

Decode uses **native GStreamer appsink** (no OpenCV `VideoCapture` / `cv_bridge`
on the hot path). Prefer hardware H.264 when available.

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

Pi sender (`camera_stream.sh`) emits UCAL1 JSON on `META_PORT` (default `GS_PORT+1`)
via `calib_util.py meta-send`, already scaled to stream resolution and matching
`UNDISTORT`.

### `calib_file`

Used for `file` / `auto` fallback. Accepts:

- a basename under `share/udp_camera_ros/config/`
- an absolute filesystem path
- a `package://pkg_name/relative/path.yaml` URL

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
WIDTH=1536 HEIGHT=864 BITRATE=16000 ./scripts/drone/camera_stream.sh

# GS — calib rides on the meta port automatically
ros2 launch drone_bringup gs_bringup.launch.py
```

## Dependencies

See `apt_packages.txt` for GStreamer host packages (`libav` for SW decode;
VA-API / NVIDIA plugins optional for HW).
