#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/videoio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace udp_camera_ros
{

/**
 * @brief Runtime settings for the UDP H.264 to ROS image bridge.
 *
 * Port / encoding must match the H.264 RTP sender (payload type 96).
 * CameraInfo is loaded from a local calibration file.
 */
struct UdpStreamConfig
{
  int port{5000};  ///< UDP listen port (bind 0.0.0.0).
  std::string image_topic{"/camera/image_raw"};  ///< Base image_transport topic.
  std::string frame_id{"camera_optical_frame"};
  sensor_msgs::msg::CameraInfo camera_info;
  int buffer_size{212992};
  int queue_size{1};
  std::string qos_reliability{"best_effort"};
  int jitter_latency_ms{80};     ///< rtpjitterbuffer latency (Wi-Fi reorder).
  int stall_timeout_ms{1500};    ///< Reopen decoder after this long with no frame.
  int reconnect_delay_ms{400};   ///< Pause between pipeline open attempts.
  int read_timeout_ms{250};      ///< OpenCV GStreamer pull timeout.

  /**
   * @brief Sanity-check port, queue_size, and qos_reliability.
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
 * 2. @ref start — spawn grab thread; OpenCV `VideoCapture` open blocks until
 *    the first RTP packet (activate itself returns immediately).
 * 3. Grab loop decodes BGR frames and publishes Image + CameraInfo.
 *    If the decoder stalls (Wi-Fi gap / lost IDR) the pipeline is reopened.
 *
 * Matches the drone GStreamer sender: udpsrc → depay → avdec_h264 → appsink.
 */
class UdpStream
{
public:
  /**
   * @brief Construct a stream helper that logs through @p logger.
   * @param logger Typically the lifecycle node's logger.
   */
  explicit UdpStream(rclcpp::Logger logger);

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
   * Returns immediately; GStreamer `udpsrc` open runs on the worker thread
   * so lifecycle activate is not blocked waiting for the first packet.
   *
   * @throws std::runtime_error if not configured or already running.
   */
  void start();

  /**
   * @brief Signal the grab thread to exit and join it.
   *
   * Releases `VideoCapture` from this thread so a blocked `open()`/`read()`
   * can unblock when the sender is gone or deactivate is requested.
   */
  void stop();

  /**
   * @brief @ref stop plus shutdown of CameraPublisher / transport node.
   */
  void cleanup();

private:
  /**
   * @brief Build the OpenCV CAP_GSTREAMER pipeline string for @ref cfg_.
   * @return Pipeline ending in BGR appsink (drop=true, max-buffers=1).
   */
  std::string build_pipeline() const;

  /**
   * @brief Worker: open pipeline (may block), then read + publish until @ref stop_.
   *
   * On open failure, empty reads, or a stall longer than @ref
   * UdpStreamConfig::stall_timeout_ms the pipeline is released and reopened
   * so a lost IDR / dead decoder does not freeze the ROS image topic.
   */
  void grab_loop();

  /**
   * @brief Release @ref cap_ so a blocked @c read() / @c open() can return.
   *
   * Safe to call from @ref stop or the grab thread. Concurrent with @c read()
   * is intentional — GStreamer unblocks @c appsink pull when the pipeline
   * goes to NULL.
   */
  void release_capture();

  /**
   * @brief Convert one BGR frame to Image + stamped CameraInfo and publish.
   * @param frame OpenCV BGR8 Mat from appsink.
   */
  void publish_frame(const cv::Mat & frame);

  rclcpp::Logger logger_;
  UdpStreamConfig cfg_;
  bool configured_{false};

  /** Separate node so image_transport publishers do not clash with lifecycle rosout. */
  rclcpp::Node::SharedPtr transport_node_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher cam_pub_;

  cv::VideoCapture cap_;
  std::mutex cap_mutex_;          ///< Guards @ref cap_ across grab / stop.
  std::atomic<bool> stop_{true};  ///< Set true to end @ref grab_loop.
  std::thread grab_thread_;
};

}  // namespace udp_camera_ros
