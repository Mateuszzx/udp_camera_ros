#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "udp_camera_ros/camera_info.hpp"

namespace udp_camera_ros
{

/**
 * @brief Runtime settings for the UDP H.264 to ROS image bridge.
 *
 * Port / encoding must match the H.264 RTP sender (payload type 96).
 * CameraInfo comes from a local file and/or UCAL1 sideband meta on @ref meta_port.
 */
struct UdpStreamConfig
{
  int port{5000};  ///< UDP listen port (bind 0.0.0.0).
  /// Sideband CameraInfo port; 0 = disable. Default callers use port+1.
  int meta_port{5001};
  std::string image_topic{"/camera/image_raw"};  ///< Base image_transport topic.
  std::string frame_id{"camera_optical_frame"};
  /// Initial CameraInfo (from file). May be empty when calib_source=stream.
  sensor_msgs::msg::CameraInfo camera_info;
  bool have_camera_info{false};
  /// file | stream | auto (prefer stream meta, keep file as fallback)
  std::string calib_source{"auto"};
  int buffer_size{212992};
  int queue_size{1};
  std::string qos_reliability{"best_effort"};
  int jitter_latency_ms{80};     ///< rtpjitterbuffer latency (Wi-Fi reorder).
  int stall_timeout_ms{1500};    ///< Reopen decoder after this long with no frame.
  int reconnect_delay_ms{400};   ///< Pause between pipeline open attempts.
  int read_timeout_ms{250};      ///< appsink pull timeout.
  /// auto | avdec_h264 | vah264dec | nvh264dec
  std::string h264_decoder{"auto"};
  /// When false (default), JPEG in GStreamer (I420→jpegenc); no raw Image.
  bool publish_raw{false};
  /// jpegenc quality when publish_raw is false (1–100).
  int jpeg_quality{80};

  /**
   * @brief Sanity-check port, queue_size, qos_reliability, and decoder.
   * @throws std::runtime_error if any field is invalid.
   */
  void validate() const;
};

/**
 * @brief Listen for H.264/RTP over UDP and republish images + CameraInfo.
 *
 * Default (`publish_raw=false`): decode → I420 → jpegenc → CompressedImage
 * on `image_topic/compressed` (skips BGR + OpenCV JPEG).
 *
 * With `publish_raw=true`: decode → BGR → image_transport CameraPublisher
 * (raw + compressed plugins).
 */
class UdpStream
{
public:
  explicit UdpStream(rclcpp::Logger logger);

  ~UdpStream();

  UdpStream(const UdpStream &) = delete;
  UdpStream & operator=(const UdpStream &) = delete;

  void configure(const UdpStreamConfig & cfg);
  void start();
  void stop();
  void cleanup();

private:
  struct Pipeline;

  std::vector<std::string> decoder_candidates() const;
  std::string build_pipeline(const std::string & decoder) const;
  std::unique_ptr<Pipeline> open_pipeline(const std::string & decoder);
  void flush_pipeline();
  void close_pipeline();
  void grab_loop();
  bool publish_sample(void * sample);
  void meta_loop();
  void apply_stream_calib(const CalibData & calib);
  void log_camera_info_once(const sensor_msgs::msg::CameraInfo & info);

  rclcpp::Logger logger_;
  UdpStreamConfig cfg_;
  bool configured_{false};
  bool size_logged_{false};
  bool first_frame_logged_{false};
  bool meta_logged_{false};
  std::string active_decoder_;
  std::string camera_info_topic_;

  rclcpp::Node::SharedPtr transport_node_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher cam_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;

  std::unique_ptr<Pipeline> pipeline_;
  std::mutex pipeline_mutex_;
  std::mutex camera_info_mutex_;
  sensor_msgs::msg::CameraInfo live_camera_info_;
  bool have_camera_info_{false};
  std::atomic<bool> stop_{true};
  std::thread grab_thread_;
  std::thread meta_thread_;
  int meta_fd_{-1};
};

}  // namespace udp_camera_ros
