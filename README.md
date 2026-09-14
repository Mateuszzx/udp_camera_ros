# udp_camera_ros

ROS 2 lifecycle node that receives H.264/RTP over UDP and republishes via
`image_transport` (raw + compressed) plus `sensor_msgs/CameraInfo`.

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

`calib_file` accepts:

- a basename under `share/udp_camera_ros/config/`
- an absolute filesystem path
- a `package://pkg_name/relative/path.yaml` URL

## Dependencies

See `apt_packages.txt` for GStreamer / OpenCV host packages.
