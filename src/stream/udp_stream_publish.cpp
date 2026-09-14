/**
 * @file udp_stream_publish.cpp
 * @brief appsink sample → ROS Image / CompressedImage + CameraInfo.
 */

#include "udp_camera_ros/udp_stream.hpp"

#include <cstring>

#include <gst/gst.h>

#include "udp_camera_ros/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace udp_camera_ros
{

bool UdpStream::publish_sample(void * sample_ptr)
{
  if (!configured_ || !transport_node_ || !sample_ptr) {
    return false;
  }

  GstSample * sample = static_cast<GstSample *>(sample_ptr);
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  GstCaps * caps = gst_sample_get_caps(sample);
  if (!buffer || !caps) {
    return false;
  }

  const GstStructure * s = gst_caps_get_structure(caps, 0);
  int width = 0;
  int height = 0;
  if (!s ||
    !gst_structure_get_int(s, "width", &width) ||
    !gst_structure_get_int(s, "height", &height) ||
    width <= 0 || height <= 0)
  {
    return false;
  }

  sensor_msgs::msg::CameraInfo template_info;
  bool have_info = false;
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    have_info = have_camera_info_;
    if (have_info) {
      template_info = live_camera_info_;
    }
  }
  if (!have_info) {
    RCLCPP_WARN_THROTTLE(
      logger_, *transport_node_->get_clock(), 3000,
      "Waiting for CameraInfo (calib_source=%s meta_port=%d)",
      cfg_.calib_source.c_str(), cfg_.meta_port);
    return false;
  }

  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    return false;
  }

  const auto stamp = transport_node_->get_clock()->now();
  auto info = stamp_camera_info(template_info, stamp, width, height);
  info.header.frame_id = cfg_.frame_id;

  if (!cfg_.publish_raw) {
    // JPEG already produced by GStreamer jpegenc (I420 → JPEG).
    sensor_msgs::msg::CompressedImage jpg;
    jpg.header.stamp = stamp;
    jpg.header.frame_id = cfg_.frame_id;
    jpg.format = "jpeg";
    jpg.data.assign(map.data, map.data + map.size);
    gst_buffer_unmap(buffer, &map);

    if (!first_frame_logged_) {
      RCLCPP_INFO(
        logger_, "First JPEG frame: %dx%d (%zu bytes)",
        width, height, jpg.data.size());
      first_frame_logged_ = true;
    }
    log_camera_info_once(info);

    if (compressed_pub_) {
      compressed_pub_->publish(jpg);
    }
    if (info_pub_) {
      info_pub_->publish(info);
    }
    return true;
  }

  const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height) * 3u;
  if (map.size < expected) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *transport_node_->get_clock(), 5000,
      "Unexpected buffer size %zu (want >= %zu for %dx%d BGR)",
      map.size, expected, width, height);
    gst_buffer_unmap(buffer, &map);
    return false;
  }

  sensor_msgs::msg::Image image_msg;
  image_msg.header.stamp = stamp;
  image_msg.header.frame_id = cfg_.frame_id;
  image_msg.height = static_cast<uint32_t>(height);
  image_msg.width = static_cast<uint32_t>(width);
  image_msg.encoding = sensor_msgs::image_encodings::BGR8;
  image_msg.is_bigendian = false;
  image_msg.step = static_cast<uint32_t>(width * 3);
  image_msg.data.resize(expected);
  std::memcpy(image_msg.data.data(), map.data, expected);
  gst_buffer_unmap(buffer, &map);

  if (!first_frame_logged_) {
    RCLCPP_INFO(logger_, "First BGR frame: %dx%d", width, height);
    first_frame_logged_ = true;
  }
  log_camera_info_once(info);

  cam_pub_.publish(image_msg, info);
  return true;
}

void UdpStream::log_camera_info_once(const sensor_msgs::msg::CameraInfo & info)
{
  if (size_logged_) {
    return;
  }
  const double cx = info.k[2];
  const double cy = info.k[5];
  RCLCPP_INFO(
    logger_,
    "CameraInfo %dx%d  fx=%.1f fy=%.1f cx=%.1f cy=%.1f "
    "(principal should be near image center ~%.0f,%.0f)",
    info.width, info.height, info.k[0], info.k[4], cx, cy,
    0.5 * info.width, 0.5 * info.height);
  if (cx < 0.0 || cy < 0.0 ||
    cx > static_cast<double>(info.width) ||
    cy > static_cast<double>(info.height))
  {
    RCLCPP_ERROR(
      logger_,
      "Principal point (cx,cy)=(%.1f,%.1f) outside %ux%u image — "
      "calib resolution does not match the stream (landmarks will be offset)",
      cx, cy, info.width, info.height);
  }
  size_logged_ = true;
}

}  // namespace udp_camera_ros
