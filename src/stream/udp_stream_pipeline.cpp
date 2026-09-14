/**
 * @file udp_stream_pipeline.cpp
 * @brief GStreamer pipeline open / recover / grab loop.
 */

#include "udp_camera_ros/udp_stream.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include "pipeline.hpp"
#include "gst_utils.hpp"

namespace udp_camera_ros
{

std::vector<std::string> UdpStream::decoder_candidates() const
{
  detail::ensure_gst_init();

  const std::vector<std::string> preferred = [&]() {
    if (cfg_.h264_decoder == "auto") {
      return std::vector<std::string>{"vah264dec", "nvh264dec", "avdec_h264"};
    }
    return std::vector<std::string>{cfg_.h264_decoder};
  }();

  std::vector<std::string> available;
  for (const auto & name : preferred) {
    if (detail::factory_exists(name.c_str())) {
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
  std::string pipe =
    "udpsrc address=0.0.0.0 port=" + std::to_string(cfg_.port) +
    " buffer-size=" + std::to_string(cfg_.buffer_size) +
    " reuse=true"
    " caps=\"application/x-rtp,media=(string)video,clock-rate=(int)90000,"
    "encoding-name=(string)H264,payload=(int)96\" ! "
    "rtpjitterbuffer latency=" + std::to_string(cfg_.jitter_latency_ms) +
    " drop-on-latency=true do-lost=true ! "
    "rtph264depay wait-for-keyframe=true ! "
    "h264parse config-interval=-1 ! " +
    decoder + " ! ";

  if (cfg_.publish_raw) {
    // Full BGR for image_transport raw (+ optional compressed plugin).
    pipe +=
      "videoconvert ! video/x-raw,format=BGR ! "
      "appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true";
  } else {
    // HW/SW decode often yields NV12; jpegenc wants I420 — cheaper than BGR,
    // and skips OpenCV JPEG in the ROS compressed plugin.
    pipe +=
      "videoconvert ! video/x-raw,format=I420 ! "
      "jpegenc quality=" + std::to_string(cfg_.jpeg_quality) + " ! "
      "appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true";
  }
  return pipe;
}

std::unique_ptr<UdpStream::Pipeline> UdpStream::open_pipeline(const std::string & decoder)
{
  detail::ensure_gst_init();

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

}  // namespace udp_camera_ros
