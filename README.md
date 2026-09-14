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

### `calib_file`

Accepts:

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

Receiver is sized for full IMX708 modes (e.g. 1536×864). Raise the sender
without changing this package’s defaults, for example on the Pi:

```bash
WIDTH=1536 HEIGHT=864 BITRATE=16000 ./scripts/drone/camera_stream.sh
```

Then confirm CameraInfo principal point is near image center and that
lifecycle deactivate/activate recovers after a Wi-Fi gap.

## Dependencies

See `apt_packages.txt` for GStreamer host packages (`libav` for SW decode;
VA-API / NVIDIA plugins optional for HW).
