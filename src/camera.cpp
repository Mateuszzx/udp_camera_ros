/**
 * @file camera.cpp
 * @brief @ref udp_camera_ros::Camera adapter implementation (params → UdpStream).
 */

#include "udp_camera_ros/camera.hpp"

#include <algorithm>
#include <stdexcept>

#include "udp_camera_ros/camera_info.hpp"

namespace udp_camera_ros
{

Camera::Camera(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: node_(std::move(node))
{
}

void Camera::configure()
{
  const auto frame_id = node_->get_parameter("frame_id").as_string();
  const auto calib_source = node_->get_parameter("calib_source").as_string();
  const auto calib_file = node_->get_parameter("calib_file").as_string();
  bool stream_undistorted = node_->get_parameter("stream_undistorted").as_bool();

  UdpStreamConfig cfg;
  cfg.port = static_cast<int>(node_->get_parameter("port").as_int());
  const int meta_port_param = static_cast<int>(node_->get_parameter("meta_port").as_int());
  // meta_port < 0 → default to video port + 1; 0 disables sideband.
  cfg.meta_port = meta_port_param < 0 ? cfg.port + 1 : meta_port_param;
  cfg.image_topic = node_->get_parameter("image_topic").as_string();
  cfg.frame_id = frame_id;
  cfg.calib_source = calib_source;
  cfg.buffer_size = static_cast<int>(node_->get_parameter("udp_buffer_size").as_int());
  cfg.queue_size = std::max(1, static_cast<int>(node_->get_parameter("queue_size").as_int()));
  cfg.qos_reliability = node_->get_parameter("qos_reliability").as_string();
  cfg.jitter_latency_ms = static_cast<int>(node_->get_parameter("jitter_latency_ms").as_int());
  cfg.stall_timeout_ms = static_cast<int>(node_->get_parameter("stall_timeout_ms").as_int());
  cfg.reconnect_delay_ms = static_cast<int>(node_->get_parameter("reconnect_delay_ms").as_int());
  cfg.read_timeout_ms = static_cast<int>(node_->get_parameter("read_timeout_ms").as_int());
  cfg.h264_decoder = node_->get_parameter("h264_decoder").as_string();

  const bool want_file =
    calib_source == "file" ||
    (calib_source == "auto" && !calib_file.empty());
  if (want_file) {
    if (calib_file.empty()) {
      throw std::runtime_error("calib_source requires calib_file");
    }
    const std::string calib_path = resolve_calib_path(calib_file);
    cfg.camera_info = build_camera_info(
      load_calib(calib_path), frame_id, stream_undistorted);
    cfg.have_camera_info = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "CameraInfo from file %s (%ux%u, undistorted=%s)",
      calib_path.c_str(), cfg.camera_info.width, cfg.camera_info.height,
      stream_undistorted ? "true" : "false");
  } else if (calib_source == "stream") {
    RCLCPP_INFO(
      node_->get_logger(),
      "CameraInfo from stream meta only (UDP port %d)", cfg.meta_port);
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "CameraInfo: waiting for stream meta (UDP port %d); no file fallback",
      cfg.meta_port);
  }

  stream_ = std::make_unique<UdpStream>(node_->get_logger());
  stream_->configure(cfg);
}

void Camera::start()
{
  if (!stream_) {
    throw std::runtime_error("start() called before configure()");
  }
  stream_->start();
}

void Camera::stop()
{
  if (stream_) {
    stream_->stop();
  }
}

void Camera::cleanup()
{
  if (stream_) {
    stream_->cleanup();
    stream_.reset();
  }
}

}  // namespace udp_camera_ros
