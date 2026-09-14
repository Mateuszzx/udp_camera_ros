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
  /// When false (default), only advertise /compressed (no raw Image topic).
  bool publish_raw{false};

  /**
   * @brief Sanity-check port, queue_size, qos_reliability, and decoder.
   * @throws std::runtime_error if any field is invalid.
   */
  void validate() const;
};

/**
 * @brief Listen for H.264/RTP over UDP and republish via image_transport.
 *
 * Flow:
 * 1. @ref configure — create transport node, advertise CameraPublisher
 *    (raw + compressed + CameraInfo).
 * 2. @ref start — spawn grab thread; GStreamer PLAYING + appsink pull runs
 *    on the worker so lifecycle activate returns immediately.
 * 3. Grab loop maps BGR buffers into sensor_msgs/Image and publishes.
 *    On stall / ERROR / EOS the pipeline is torn down and rebuilt.
 *
 * Pipeline: udpsrc → rtpjitterbuffer → rtph264depay → h264parse →
 * decoder → videoconvert → appsink (drop=true, max-buffers=1).
 */
class UdpStream
{
public:
  /**
   * @brief Construct a stream helper that logs through @p logger.
   * @param logger Typically the lifecycle node's logger.
   */
  explicit UdpStream(rclcpp::Logger logger);

  ~UdpStream();

  UdpStream(const UdpStream &) = delete;
  UdpStream & operator=(const UdpStream &) = delete;

  /**
   * @brief Create the image_transport publishers (no UDP I/O yet).
   *
   * Whitelists only `raw` and `compressed` plugins so ffmpeg / depth /
   * theora topics are not advertised.
   *
   * @param cfg Validated stream configuration (copied).
   * @throws std::runtime_error if @ref UdpStreamConfig::validate fails.
   */
  void configure(const UdpStreamConfig & cfg);

  /**
   * @brief Start the background grab thread.
   *
   * Returns immediately; pipeline PLAYING / first-packet wait runs on the
   * worker so lifecycle activate is not blocked.
   *
   * @throws std::runtime_error if not configured or already running.
   */
  void start();

  /**
   * @brief Signal the grab thread to exit, set pipeline to NULL, and join.
   */
  void stop();

  /**
   * @brief @ref stop plus shutdown of CameraPublisher / transport node.
   */
  void cleanup();

private:
  struct Pipeline;

  /**
   * @brief List of decoder element names to try for @ref UdpStreamConfig::h264_decoder.
   * @throws std::runtime_error if none of the candidates exist on this host.
   */
  std::vector<std::string> decoder_candidates() const;

  /**
   * @brief Build the GStreamer pipeline launch string for @p decoder.
   */
  std::string build_pipeline(const std::string & decoder) const;

  /**
   * @brief Create, PLAY, and return a pipeline; nullptr on failure.
   */
  std::unique_ptr<Pipeline> open_pipeline(const std::string & decoder);

  /** @brief Set pipeline to NULL (unblocks appsink pull) without destroying it. */
  void flush_pipeline();

  /** @brief Destroy @ref pipeline_ (caller must not be pulling). */
  void close_pipeline();

  /**
   * @brief Worker: open pipeline, pull samples, republish until @ref stop_.
   *
   * Rebuilds after stall, ERROR, EOS, or failed open.
   */
  void grab_loop();

  /**
   * @brief Map one BGR GstSample into Image + CameraInfo and publish.
   * @return false if the sample could not be mapped / wrong format.
   */
  bool publish_sample(void * sample);

  /** @brief Background UDP listener for UCAL1 CameraInfo sideband packets. */
  void meta_loop();

  /** @brief Apply parsed stream meta under @ref camera_info_mutex_. */
  void apply_stream_calib(const CalibData & calib);

  rclcpp::Logger logger_;
  UdpStreamConfig cfg_;
  bool configured_{false};
  bool size_logged_{false};
  bool first_frame_logged_{false};
  bool meta_logged_{false};
  std::string active_decoder_;

  /** Separate node so image_transport publishers do not clash with lifecycle rosout. */
  rclcpp::Node::SharedPtr transport_node_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher cam_pub_;

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
