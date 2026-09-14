/**
 * @file udp_stream.cpp
 * @brief @ref udp_camera_ros::UdpStream implementation.
 */

#include "udp_camera_ros/udp_stream.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "udp_camera_ros/camera_info.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/image_encodings.hpp"

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
}

UdpStream::UdpStream(rclcpp::Logger logger)
: logger_(std::move(logger))
{
}

/**
 * Build the OpenCV CAP_GSTREAMER pipeline string.
 *
 * Each `!` is a GStreamer link. Left -> right = data flow:
 *
 *   udpsrc          listen for RTP on cfg_.port (all interfaces)
 *   rtpjitterbuffer reorder buffer; drop late packets (Wi-Fi jitter)
 *   rtph264depay    RTP -> H.264 NALs; wait-for-keyframe after loss
 *   h264parse       insert SPS/PPS before each IDR (decoder resync)
 *   avdec_h264      software H.264 decode (needs gstreamer1.0-libav)
 *   videoconvert    to BGR for OpenCV / cv_bridge
 *   appsink         hand frames to OpenCV (keep only newest: drop -> max-buffers=1)
 *
 * Important: udpsrc makes VideoCapture::open() block until the first UDP
 * packet arrives. That open runs inside grab_loop() on a background thread
 * so lifecycle activate() can return immediately.
 */
std::string UdpStream::build_pipeline() const
{
  return
    "udpsrc address=0.0.0.0 port=" + std::to_string(cfg_.port) +
    " buffer-size=" + std::to_string(cfg_.buffer_size) +
    " reuse=true"
    " caps=\"application/x-rtp,media=(string)video,clock-rate=(int)90000,"
    "encoding-name=(string)H264,payload=(int)96\" ! "
    "rtpjitterbuffer latency=" + std::to_string(cfg_.jitter_latency_ms) +
    " drop-on-latency=true do-lost=true ! "
    "rtph264depay wait-for-keyframe=true ! "
    "h264parse config-interval=-1 ! "
    "avdec_h264 ! "
    "videoconvert ! video/x-raw,format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false";
}

/**
 * Prepare ROS publishers. Does NOT open the UDP socket yet.
 *
 * Why a second node ("camera_transport")?
 *   image_transport needs an rclcpp::Node. Reusing the lifecycle node name
 *   would collide on /rosout, so we use a dedicated helper node.
 *
 * Why enable_pub_plugins?
 *   By default every installed transport plugin advertises a topic
 *   (ffmpeg, compressedDepth, ...). We only want raw + JPEG compressed.
 */
void UdpStream::configure(const UdpStreamConfig & cfg)
{
  cfg.validate();
  cfg_ = cfg;

  // Helper node for image_transport (not spun - publishing does not need spin).
  transport_node_ = std::make_shared<rclcpp::Node>(
    "camera_transport",
    rclcpp::NodeOptions().use_global_arguments(false));

  // image_transport turns "/camera/image_raw" into param
  // "camera.image_raw.enable_pub_plugins".
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

  // CameraPublisher = Image (all enabled transports) + sibling CameraInfo.
  // Best-effort matches ArUco / HSV / YOLO (SensorDataQoS) and skips DDS
  // retransmission of stale live video.
  it_ = std::make_shared<image_transport::ImageTransport>(transport_node_);
  rmw_qos_profile_t qos = cfg_.qos_reliability == "best_effort"
    ? rmw_qos_profile_sensor_data
    : rmw_qos_profile_default;
  qos.depth = static_cast<size_t>(cfg_.queue_size);
  cam_pub_ = image_transport::create_camera_publisher(
    transport_node_.get(), cfg_.image_topic, qos);

  configured_ = true;
  RCLCPP_INFO(
    logger_,
    "Configured UDP listen 0.0.0.0:%d -> %s + /compressed ; info->%s",
    cfg_.port, cfg_.image_topic.c_str(), cam_pub_.getInfoTopic().c_str());
  RCLCPP_INFO(
    logger_,
    "Start sender AFTER activate. Example: udpsink host=<this-host-ip> port=%d sync=false -> %s",
    cfg_.port, cfg_.image_topic.c_str());
  RCLCPP_INFO(
    logger_,
    "Stream recovery: qos=%s jitter=%dms stall=%dms reconnect=%dms read-timeout=%dms",
    cfg_.qos_reliability.c_str(), cfg_.jitter_latency_ms,
    cfg_.stall_timeout_ms, cfg_.reconnect_delay_ms, cfg_.read_timeout_ms);
}

/**
 * Kick off the background receiver. Returns right away.
 *
 * The actual GStreamer open + frame loop lives in grab_loop().
 * Call this from lifecycle on_activate().
 */
void UdpStream::start()
{
  if (!configured_) {
    throw std::runtime_error("start() called before configure()");
  }
  if (grab_thread_.joinable()) {
    throw std::runtime_error("start() called while already running");
  }

  stop_ = false;
  grab_thread_ = std::thread([this]() {grab_loop();});
  RCLCPP_INFO(
    logger_,
    "UDP grab thread started (waiting for H.264/RTP on port %d)...",
    cfg_.port);
}

/**
 * Ask grab_loop() to exit and wait for it.
 *
 * Trick: if VideoCapture is stuck in open()/read() waiting for UDP, we
 * release() it from this thread so GStreamer unblocks and the worker can
 * notice stop_ == true.
 */
void UdpStream::release_capture()
{
  // No lock around release(): grab_loop may be blocked in open()/read() and
  // GStreamer unblocks appsink when the pipeline is torn down.
  if (cap_.isOpened()) {
    cap_.release();
  }
}

void UdpStream::stop()
{
  stop_ = true;
  release_capture();

  if (grab_thread_.joinable()) {
    // Never join ourselves (would deadlock if somehow called from grab thread).
    if (grab_thread_.get_id() != std::this_thread::get_id()) {
      grab_thread_.join();
    } else {
      grab_thread_.detach();
    }
  }

  release_capture();
}

/** stop() + tear down image_transport so configure() can run again. */
void UdpStream::cleanup()
{
  stop();
  cam_pub_.shutdown();
  it_.reset();
  transport_node_.reset();
  configured_ = false;
}

/**
 * Worker thread body.
 *
 * Outer loop reopens the pipeline after a failed open, a dead decoder, or a
 * stall (no frame for stall_timeout_ms). Inner loop reads until stop_ or stall.
 *
 * Phase 1 - open:  VideoCapture(pipeline) blocks until first RTP packet
 *                  (or until stop() releases the capture).
 * Phase 2 - loop:  read BGR frames and publish until stop_ or stall.
 */
void UdpStream::grab_loop()
{
  const std::string pipeline = build_pipeline();
  rclcpp::Clock clock(RCL_ROS_TIME);
  int attempt = 0;

  while (!stop_) {
    ++attempt;
    if (attempt == 1) {
      RCLCPP_INFO(logger_, "Opening GStreamer pipeline (blocks until first UDP packet)...");
    } else {
      RCLCPP_WARN(logger_, "Reopening GStreamer pipeline (attempt %d)...", attempt);
    }
    RCLCPP_DEBUG(logger_, "Pipeline: %s", pipeline.c_str());

    cv::VideoCapture cap;
    if (!cap.open(pipeline, cv::CAP_GSTREAMER)) {
      if (stop_) {
        return;
      }
      RCLCPP_ERROR(
        logger_, "GStreamer open failed — retry in %d ms", cfg_.reconnect_delay_ms);
      std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.reconnect_delay_ms));
      continue;
    }

    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    // Unblock read() so a Wi-Fi gap can be detected instead of hanging forever.
    cap.set(cv::CAP_PROP_READ_TIMEOUT_MSEC, cfg_.read_timeout_ms);

    {
      std::lock_guard<std::mutex> lock(cap_mutex_);
      cap_ = std::move(cap);
    }
    RCLCPP_INFO(logger_, "GStreamer pipeline open - reading frames");

    bool waiting_logged = false;
    std::atomic<bool> got_frame{false};
    auto last_frame_at = std::chrono::steady_clock::now();
    std::atomic<bool> session_live{true};
    std::atomic<long> last_frame_ms{0};
    const auto session_start = last_frame_at;

    // Unblock a hung read() if OpenCV ignores READ_TIMEOUT_MSEC.
    std::thread watchdog([this, &session_live, &last_frame_ms, &got_frame, session_start]() {
      while (session_live.load() && !stop_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!got_frame.load()) {
          continue;
        }
        const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - session_start).count();
        const auto age = now_ms - last_frame_ms.load();
        if (age >= cfg_.stall_timeout_ms) {
          RCLCPP_WARN(
            logger_, "Watchdog: no video for %ld ms — restarting decoder",
            static_cast<long>(age));
          release_capture();
          break;
        }
      }
    });

    while (!stop_ && cap_.isOpened()) {
      cv::Mat frame;
      const bool ok = cap_.read(frame);

      if (ok && !frame.empty()) {
        if (!got_frame.exchange(true)) {
          RCLCPP_INFO(
            logger_, "First frame received: %dx%d", frame.cols, frame.rows);
        }
        waiting_logged = false;
        last_frame_at = std::chrono::steady_clock::now();
        last_frame_ms.store(
          std::chrono::duration_cast<std::chrono::milliseconds>(
            last_frame_at - session_start).count());
        publish_frame(frame);
        continue;
      }

      if (stop_ || !cap_.isOpened()) {
        break;
      }

      const auto stalled_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_frame_at).count();
      if (got_frame.load() && stalled_ms >= cfg_.stall_timeout_ms) {
        RCLCPP_WARN(
          logger_,
          "No video for %ld ms — restarting decoder",
          static_cast<long>(stalled_ms));
        break;
      }

      if (!waiting_logged) {
        RCLCPP_WARN(logger_, "Waiting for incoming H.264/RTP packets...");
        waiting_logged = true;
      } else {
        RCLCPP_WARN_THROTTLE(
          logger_, clock, 3000, "Waiting for incoming video packets...");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    session_live = false;
    if (watchdog.joinable()) {
      watchdog.join();
    }
    release_capture();
    if (!stop_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.reconnect_delay_ms));
    }
  }
}

/**
 * One decoded BGR Mat -> sensor_msgs/Image + CameraInfo on cam_pub_.
 *
 * CameraInfo intrinsics come from local calib YAML (cfg_.camera_info);
 * only the stamp / size are updated per frame. Video never carries calib.
 */
void UdpStream::publish_frame(const cv::Mat & frame)
{
  if (!configured_ || !transport_node_) {
    return;
  }

  std_msgs::msg::Header header;
  header.stamp = transport_node_->get_clock()->now();
  header.frame_id = cfg_.frame_id;

  // OpenCV BGR8 -> ROS Image (plugins may also emit /compressed on demand).
  cv_bridge::CvImage cv_img(header, sensor_msgs::image_encodings::BGR8, frame);
  auto image_msg = cv_img.toImageMsg();

  auto info = stamp_camera_info(cfg_.camera_info, header.stamp, frame.cols, frame.rows);
  info.header.frame_id = cfg_.frame_id;

  static bool size_logged = false;
  if (!size_logged) {
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
    size_logged = true;
  }

  cam_pub_.publish(*image_msg, info);
}

}  // namespace udp_camera_ros
