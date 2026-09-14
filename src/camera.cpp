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
  const auto calib_file = node_->get_parameter("calib_file").as_string();
  const auto frame_id = node_->get_parameter("frame_id").as_string();
  const bool stream_undistorted = node_->get_parameter("stream_undistorted").as_bool();

  const std::string calib_path = resolve_calib_path(calib_file);
  auto camera_info = build_camera_info(load_calib(calib_path), frame_id, stream_undistorted);

  RCLCPP_INFO(
    node_->get_logger(),
    "CameraInfo from %s (%ux%u, undistorted=%s)",
    calib_path.c_str(), camera_info.width, camera_info.height,
    stream_undistorted ? "true" : "false");

  UdpStreamConfig cfg;
  cfg.port = static_cast<int>(node_->get_parameter("port").as_int());
  cfg.image_topic = node_->get_parameter("image_topic").as_string();
  cfg.frame_id = frame_id;
  cfg.camera_info = camera_info;
  cfg.buffer_size = static_cast<int>(node_->get_parameter("udp_buffer_size").as_int());
  cfg.queue_size = std::max(1, static_cast<int>(node_->get_parameter("queue_size").as_int()));
  cfg.qos_reliability = node_->get_parameter("qos_reliability").as_string();
  cfg.jitter_latency_ms = static_cast<int>(node_->get_parameter("jitter_latency_ms").as_int());
  cfg.stall_timeout_ms = static_cast<int>(node_->get_parameter("stall_timeout_ms").as_int());
  cfg.reconnect_delay_ms = static_cast<int>(node_->get_parameter("reconnect_delay_ms").as_int());
  cfg.read_timeout_ms = static_cast<int>(node_->get_parameter("read_timeout_ms").as_int());

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
