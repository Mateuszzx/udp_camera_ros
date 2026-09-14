#pragma once

#include <string>

namespace udp_camera_ros
{
namespace detail
{

/** One-time gst_init_check; throws on failure. */
void ensure_gst_init();

/** True if a GStreamer element factory with @p name exists. */
bool factory_exists(const char * name);

/**
 * Map image_transport base topic to CameraInfo topic.
 * `/ns/image_raw` → `/ns/camera_info`.
 */
std::string camera_info_topic_for(const std::string & image_topic);

}  // namespace detail
}  // namespace udp_camera_ros
