/**
 * @file udp_stream_meta.cpp
 * @brief UCAL1 sideband CameraInfo UDP listener.
 */

#include "udp_camera_ros/udp_stream.hpp"

#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "udp_camera_ros/camera_info.hpp"

namespace udp_camera_ros
{

void UdpStream::apply_stream_calib(const CalibData & calib)
{
  auto info = camera_info_from_calib(calib, cfg_.frame_id);
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    live_camera_info_ = info;
    have_camera_info_ = true;
  }
  size_logged_ = false;
  if (!meta_logged_) {
    RCLCPP_INFO(
      logger_,
      "CameraInfo from stream meta (%dx%d, model=%s)",
      info.width, info.height, info.distortion_model.c_str());
    meta_logged_ = true;
  }
}

void UdpStream::meta_loop()
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    RCLCPP_ERROR(logger_, "meta socket() failed");
    return;
  }
  meta_fd_ = fd;

  int yes = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(static_cast<uint16_t>(cfg_.meta_port));
  if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(logger_, "meta bind 0.0.0.0:%d failed", cfg_.meta_port);
    ::close(fd);
    meta_fd_ = -1;
    return;
  }

  // Make recv interruptible via shutdown() from stop().
  timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = 200000;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  RCLCPP_INFO(logger_, "Listening for UCAL1 CameraInfo on UDP port %d", cfg_.meta_port);

  std::vector<char> buf(65536);
  while (!stop_) {
    const ssize_t n = ::recv(fd, buf.data(), buf.size(), 0);
    if (n < 0) {
      continue;
    }
    if (n == 0) {
      continue;
    }
    try {
      const std::string payload(buf.data(), static_cast<size_t>(n));
      apply_stream_calib(parse_meta_payload(payload));
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        logger_, *transport_node_->get_clock(), 5000,
        "Ignoring bad meta packet: %s", e.what());
    }
  }

  ::close(fd);
  meta_fd_ = -1;
}

}  // namespace udp_camera_ros
