/**
 * @file udp_stream.cpp
 * @brief Lifecycle / publisher setup for @ref udp_camera_ros::UdpStream.
 */

#include "udp_camera_ros/udp_stream.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include <sys/socket.h>

#include "pipeline.hpp"
#include "gst_utils.hpp"

namespace udp_camera_ros
{

void UdpStreamConfig::validate() const
{
  if (qos_reliability != "reliable" && qos_reliability != "best_effort") {
    throw std::runtime_error(
            "qos_reliability must be reliable|best_effort (got '" + qos_reliability + "')");
  }
  if (port <= 0 || port > 65535) {
    throw std::runtime_error("invalid UDP port: " + std::to_string(port));
  }
  if (queue_size < 1) {
    throw std::runtime_error("queue_size must be >= 1");
  }
  if (jitter_latency_ms < 0) {
    throw std::runtime_error("jitter_latency_ms must be >= 0");
  }
  if (stall_timeout_ms < 1) {
    throw std::runtime_error("stall_timeout_ms must be >= 1");
  }
  if (reconnect_delay_ms < 0) {
    throw std::runtime_error("reconnect_delay_ms must be >= 0");
  }
  if (read_timeout_ms < 1) {
    throw std::runtime_error("read_timeout_ms must be >= 1");
  }
  if (h264_decoder != "auto" && h264_decoder != "avdec_h264" &&
    h264_decoder != "vah264dec" && h264_decoder != "nvh264dec")
  {
    throw std::runtime_error(
            "h264_decoder must be auto|avdec_h264|vah264dec|nvh264dec (got '" +
            h264_decoder + "')");
  }
  if (calib_source != "auto" && calib_source != "stream" && calib_source != "file") {
    throw std::runtime_error(
            "calib_source must be auto|stream|file (got '" + calib_source + "')");
  }
  if (meta_port < 0 || meta_port > 65535) {
    throw std::runtime_error("invalid meta_port: " + std::to_string(meta_port));
  }
  if (calib_source == "file" && !have_camera_info) {
    throw std::runtime_error("calib_source=file requires a loaded calib_file");
  }
  if (calib_source == "stream" && meta_port == 0) {
    throw std::runtime_error("calib_source=stream requires meta_port > 0");
  }
  if (!have_camera_info && meta_port == 0) {
    throw std::runtime_error(
            "no CameraInfo source: set calib_file and/or meta_port > 0");
  }
  if (jpeg_quality < 1 || jpeg_quality > 100) {
    throw std::runtime_error("jpeg_quality must be 1..100");
  }
}

UdpStream::UdpStream(rclcpp::Logger logger)
: logger_(std::move(logger))
{
}

UdpStream::~UdpStream()
{
  cleanup();
}

void UdpStream::configure(const UdpStreamConfig & cfg)
{
  cfg.validate();
  cfg_ = cfg;
  size_logged_ = false;
  first_frame_logged_ = false;
  meta_logged_ = false;
  active_decoder_.clear();
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    live_camera_info_ = cfg_.camera_info;
    have_camera_info_ = cfg_.have_camera_info;
    if (have_camera_info_) {
      live_camera_info_.header.frame_id = cfg_.frame_id;
    }
  }

  transport_node_ = std::make_shared<rclcpp::Node>(
    "camera_transport",
    rclcpp::NodeOptions().use_global_arguments(false));

  camera_info_topic_ = detail::camera_info_topic_for(cfg_.image_topic);
  rclcpp::QoS qos(cfg_.queue_size);
  if (cfg_.qos_reliability == "best_effort") {
    qos = rclcpp::SensorDataQoS().keep_last(cfg_.queue_size);
  }

  compressed_pub_.reset();
  info_pub_.reset();
  it_.reset();
  cam_pub_.shutdown();

  if (cfg_.publish_raw) {
    std::string param_base = cfg_.image_topic;
    if (!param_base.empty() && param_base.front() == '/') {
      param_base.erase(param_base.begin());
    }
    std::replace(param_base.begin(), param_base.end(), '/', '.');
    transport_node_->declare_parameter(
      param_base + ".enable_pub_plugins",
      std::vector<std::string>{
        "image_transport/raw",
        "image_transport/compressed",
      });
    it_ = std::make_shared<image_transport::ImageTransport>(transport_node_);
    rmw_qos_profile_t rmw_qos = cfg_.qos_reliability == "best_effort"
      ? rmw_qos_profile_sensor_data
      : rmw_qos_profile_default;
    rmw_qos.depth = static_cast<size_t>(cfg_.queue_size);
    cam_pub_ = image_transport::create_camera_publisher(
      transport_node_.get(), cfg_.image_topic, rmw_qos);
    RCLCPP_INFO(
      logger_,
      "Configured UDP 0.0.0.0:%d -> %s + /compressed (BGR path) ; info->%s",
      cfg_.port, cfg_.image_topic.c_str(), cam_pub_.getInfoTopic().c_str());
  } else {
    if (!detail::factory_exists("jpegenc")) {
      throw std::runtime_error("jpegenc not available (install gstreamer1.0-plugins-good)");
    }
    const std::string compressed_topic = cfg_.image_topic + "/compressed";
    compressed_pub_ = transport_node_->create_publisher<sensor_msgs::msg::CompressedImage>(
      compressed_topic, qos);
    info_pub_ = transport_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, qos);
    RCLCPP_INFO(
      logger_,
      "Configured UDP 0.0.0.0:%d -> %s (GStreamer I420→jpegenc q=%d) ; info->%s",
      cfg_.port, compressed_topic.c_str(), cfg_.jpeg_quality,
      camera_info_topic_.c_str());
  }

  configured_ = true;
  RCLCPP_INFO(
    logger_,
    "Calib source=%s meta_port=%d have_file_info=%s",
    cfg_.calib_source.c_str(), cfg_.meta_port,
    cfg_.have_camera_info ? "true" : "false");
  RCLCPP_INFO(
    logger_,
    "Start sender AFTER activate. Example: udpsink host=<this-host-ip> port=%d sync=false",
    cfg_.port);
  RCLCPP_INFO(
    logger_,
    "Stream recovery: qos=%s decoder=%s jitter=%dms stall=%dms reconnect=%dms read-timeout=%dms",
    cfg_.qos_reliability.c_str(), cfg_.h264_decoder.c_str(), cfg_.jitter_latency_ms,
    cfg_.stall_timeout_ms, cfg_.reconnect_delay_ms, cfg_.read_timeout_ms);
}

void UdpStream::start()
{
  if (!configured_) {
    throw std::runtime_error("start() called before configure()");
  }
  if (grab_thread_.joinable() || meta_thread_.joinable()) {
    throw std::runtime_error("start() called while already running");
  }

  stop_ = false;
  const bool want_meta =
    cfg_.meta_port > 0 &&
    (cfg_.calib_source == "stream" || cfg_.calib_source == "auto");
  if (want_meta) {
    meta_thread_ = std::thread([this]() {meta_loop();});
  }
  grab_thread_ = std::thread([this]() {grab_loop();});
  RCLCPP_INFO(
    logger_,
    "UDP grab thread started (waiting for H.264/RTP on port %d)...",
    cfg_.port);
}

void UdpStream::stop()
{
  stop_ = true;
  // Unblock try_pull_sample without destroying elements the worker still holds.
  flush_pipeline();
  if (meta_fd_ >= 0) {
    ::shutdown(meta_fd_, SHUT_RDWR);
  }

  if (grab_thread_.joinable()) {
    if (grab_thread_.get_id() != std::this_thread::get_id()) {
      grab_thread_.join();
    } else {
      grab_thread_.detach();
    }
  }
  if (meta_thread_.joinable()) {
    if (meta_thread_.get_id() != std::this_thread::get_id()) {
      meta_thread_.join();
    } else {
      meta_thread_.detach();
    }
  }

  close_pipeline();
}

void UdpStream::cleanup()
{
  stop();
  if (configured_) {
    cam_pub_.shutdown();
    compressed_pub_.reset();
    info_pub_.reset();
    it_.reset();
    transport_node_.reset();
    configured_ = false;
  }
}

}  // namespace udp_camera_ros
