#pragma once

#include <gst/gst.h>

#include "udp_camera_ros/udp_stream.hpp"

namespace udp_camera_ros
{

/** Owns a PLAYING GStreamer pipeline + appsink (pull mode). */
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

}  // namespace udp_camera_ros
