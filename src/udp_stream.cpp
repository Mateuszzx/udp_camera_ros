/**
 * @file udp_stream.cpp
 * @brief Native GStreamer appsink implementation of @ref udp_camera_ros::UdpStream.
 */

#include "udp_camera_ros/udp_stream.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include "udp_camera_ros/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace udp_camera_ros
{
namespace
{

std::once_flag g_gst_once;

void ensure_gst_init()
{
  std::call_once(g_gst_once, []() {
    GError * err = nullptr;
    if (!gst_init_check(nullptr, nullptr, &err)) {
      const std::string msg = err ? err->message : "gst_init_check failed";
      if (err) {
        g_error_free(err);
      }
      throw std::runtime_error(msg);
    }
  });
}

bool factory_exists(const char * name)
{
  GstElementFactory * factory = gst_element_factory_find(name);
  if (!factory) {
    return false;
  }
  gst_object_unref(factory);
  return true;
}

}  // namespace

struct UdpStream::Pipeline
{
  GstElement * pipeline{nullptr};
  GstElement * appsink{nullptr};

  ~Pipeline()
  {
    if (pipeline) {
      gst_element_set_state(pipeline, GST_STATE_NULL);
    }
    if (appsink) {
      gst_object_unref(appsink);
      appsink = nullptr;
    }
    if (pipeline) {
      gst_object_unref(pipeline);
      pipeline = nullptr;
    }
  }
};

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
}

UdpStream::UdpStream(rclcpp::Logger logger)
: logger_(std::move(logger))
{
}

UdpStream::~UdpStream()
{
  cleanup();
}

std::vector<std::string> UdpStream::decoder_candidates() const
{
  ensure_gst_init();

  const std::vector<std::string> preferred = [&]() {
    if (cfg_.h264_decoder == "auto") {
      return std::vector<std::string>{"vah264dec", "nvh264dec", "avdec_h264"};
    }
    return std::vector<std::string>{cfg_.h264_decoder};
  }();

  std::vector<std::string> available;
  for (const auto & name : preferred) {
    if (factory_exists(name.c_str())) {
      available.push_back(name);
    } else {
      RCLCPP_WARN(logger_, "H.264 decoder '%s' not available on this host", name.c_str());
    }
  }
  if (available.empty()) {
    throw std::runtime_error(
            "no usable H.264 decoder (wanted '" + cfg_.h264_decoder + "')");
  }
  return available;
}

std::string UdpStream::build_pipeline(const std::string & decoder) const
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
    "h264parse config-interval=-1 ! " +
    decoder + " ! "
    "videoconvert ! video/x-raw,format=BGR ! "
    "appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true";
}

std::unique_ptr<UdpStream::Pipeline> UdpStream::open_pipeline(const std::string & decoder)
{
  ensure_gst_init();

  const std::string launch = build_pipeline(decoder);
  RCLCPP_DEBUG(logger_, "Pipeline: %s", launch.c_str());

  GError * err = nullptr;
  GstElement * pipeline = gst_parse_launch(launch.c_str(), &err);
  if (!pipeline) {
    const std::string msg = err ? err->message : "gst_parse_launch failed";
    if (err) {
      g_error_free(err);
    }
    RCLCPP_ERROR(logger_, "Failed to create pipeline: %s", msg.c_str());
    return nullptr;
  }
  if (err) {
    RCLCPP_WARN(logger_, "Pipeline parse warning: %s", err->message);
    g_error_free(err);
  }

  GstElement * appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
  if (!appsink) {
    RCLCPP_ERROR(logger_, "Pipeline missing appsink named 'sink'");
    gst_object_unref(pipeline);
    return nullptr;
  }

  // Pull mode: we drive the clock via try_pull_sample timeouts.
  gst_app_sink_set_max_buffers(GST_APP_SINK(appsink), 1);
  gst_app_sink_set_drop(GST_APP_SINK(appsink), TRUE);

  const GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(logger_, "Failed to set pipeline to PLAYING");
    gst_object_unref(appsink);
    gst_object_unref(pipeline);
    return nullptr;
  }

  auto out = std::make_unique<Pipeline>();
  out->pipeline = pipeline;
  out->appsink = appsink;
  return out;
}

void UdpStream::flush_pipeline()
{
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  if (pipeline_ && pipeline_->pipeline) {
    gst_element_set_state(pipeline_->pipeline, GST_STATE_NULL);
  }
}

void UdpStream::close_pipeline()
{
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  pipeline_.reset();
}

void UdpStream::configure(const UdpStreamConfig & cfg)
{
  cfg.validate();
  cfg_ = cfg;
  size_logged_ = false;
  first_frame_logged_ = false;
  active_decoder_.clear();

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

void UdpStream::stop()
{
  stop_ = true;
  // Unblock try_pull_sample without destroying elements the worker still holds.
  flush_pipeline();

  if (grab_thread_.joinable()) {
    if (grab_thread_.get_id() != std::this_thread::get_id()) {
      grab_thread_.join();
    } else {
      grab_thread_.detach();
    }
  }

  close_pipeline();
}

void UdpStream::cleanup()
{
  stop();
  if (configured_) {
    cam_pub_.shutdown();
    it_.reset();
    transport_node_.reset();
    configured_ = false;
  }
}

void UdpStream::grab_loop()
{
  int attempt = 0;

  while (!stop_) {
    ++attempt;

    std::vector<std::string> candidates;
    try {
      candidates = decoder_candidates();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "%s — retry in %d ms", e.what(), cfg_.reconnect_delay_ms);
      std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.reconnect_delay_ms));
      continue;
    }

    if (attempt == 1) {
      RCLCPP_INFO(logger_, "Opening GStreamer pipeline (waits for first UDP packet)...");
    } else {
      RCLCPP_WARN(logger_, "Reopening GStreamer pipeline (attempt %d)...", attempt);
    }

    std::unique_ptr<Pipeline> pipeline;
    std::string decoder;
    for (const auto & name : candidates) {
      pipeline = open_pipeline(name);
      if (pipeline) {
        decoder = name;
        break;
      }
      RCLCPP_WARN(logger_, "Decoder '%s' failed to start — trying next", name.c_str());
    }

    if (!pipeline) {
      if (stop_) {
        return;
      }
      RCLCPP_ERROR(
        logger_, "GStreamer open failed for all decoders — retry in %d ms",
        cfg_.reconnect_delay_ms);
      std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.reconnect_delay_ms));
      continue;
    }

    if (active_decoder_ != decoder) {
      active_decoder_ = decoder;
      RCLCPP_INFO(logger_, "Using H.264 decoder: %s", decoder.c_str());
    }

    {
      std::lock_guard<std::mutex> lock(pipeline_mutex_);
      pipeline_ = std::move(pipeline);
    }
    RCLCPP_INFO(logger_, "GStreamer pipeline PLAYING — pulling frames");

    bool waiting_logged = false;
    auto last_frame_at = std::chrono::steady_clock::now();
    bool got_frame = false;
    const guint64 pull_timeout_ns =
      static_cast<guint64>(cfg_.read_timeout_ms) * GST_MSECOND;

    bool need_reopen = false;
    while (!stop_ && !need_reopen) {
      GstElement * appsink = nullptr;
      GstElement * pipe = nullptr;
      {
        std::lock_guard<std::mutex> lock(pipeline_mutex_);
        if (!pipeline_) {
          break;
        }
        appsink = pipeline_->appsink;
        pipe = pipeline_->pipeline;
      }

      GstBus * bus = gst_element_get_bus(pipe);
      while (true) {
        GstMessage * msg = gst_bus_pop_filtered(
          bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        if (!msg) {
          break;
        }
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
          GError * err = nullptr;
          gchar * dbg = nullptr;
          gst_message_parse_error(msg, &err, &dbg);
          RCLCPP_ERROR(
            logger_, "GStreamer error: %s (%s)",
            err ? err->message : "unknown", dbg ? dbg : "");
          if (err) {
            g_error_free(err);
          }
          g_free(dbg);
          need_reopen = true;
        } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
          RCLCPP_WARN(logger_, "GStreamer EOS — reopening");
          need_reopen = true;
        }
        gst_message_unref(msg);
      }
      gst_object_unref(bus);
      if (need_reopen) {
        break;
      }

      GstSample * sample = gst_app_sink_try_pull_sample(
        GST_APP_SINK(appsink), pull_timeout_ns);

      if (sample) {
        const bool ok = publish_sample(sample);
        gst_sample_unref(sample);
        if (ok) {
          got_frame = true;
          waiting_logged = false;
          last_frame_at = std::chrono::steady_clock::now();
        }
        continue;
      }

      if (stop_) {
        break;
      }

      // NULL sample: timeout, EOS, or flushed sink after close_pipeline().
      if (gst_app_sink_is_eos(GST_APP_SINK(appsink))) {
        RCLCPP_WARN(logger_, "appsink EOS — reopening");
        break;
      }

      const auto stalled_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_frame_at).count();
      if (got_frame && stalled_ms >= cfg_.stall_timeout_ms) {
        RCLCPP_WARN(
          logger_,
          "No video for %ld ms — restarting decoder",
          static_cast<long>(stalled_ms));
        break;
      }

      if (!waiting_logged) {
        RCLCPP_WARN(logger_, "Waiting for incoming H.264/RTP packets...");
        waiting_logged = true;
      }
    }

    close_pipeline();
    if (!stop_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.reconnect_delay_ms));
    }
  }
}

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

  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    return false;
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
  image_msg.header.stamp = transport_node_->get_clock()->now();
  image_msg.header.frame_id = cfg_.frame_id;
  image_msg.height = static_cast<uint32_t>(height);
  image_msg.width = static_cast<uint32_t>(width);
  image_msg.encoding = sensor_msgs::image_encodings::BGR8;
  image_msg.is_bigendian = false;
  image_msg.step = static_cast<uint32_t>(width * 3);
  image_msg.data.resize(expected);
  std::memcpy(image_msg.data.data(), map.data, expected);
  gst_buffer_unmap(buffer, &map);

  auto info = stamp_camera_info(
    cfg_.camera_info, image_msg.header.stamp, width, height);
  info.header.frame_id = cfg_.frame_id;

  if (!first_frame_logged_) {
    RCLCPP_INFO(logger_, "First frame received: %dx%d", width, height);
    first_frame_logged_ = true;
  }

  if (!size_logged_) {
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

  cam_pub_.publish(image_msg, info);
  return true;
}

}  // namespace udp_camera_ros
