#include "gst_utils.hpp"

#include <mutex>
#include <stdexcept>
#include <string_view>

#include <gst/gst.h>

namespace udp_camera_ros
{
namespace detail
{
namespace
{

std::once_flag g_gst_once;

}  // namespace

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

std::string camera_info_topic_for(const std::string & image_topic)
{
  constexpr std::string_view kRaw = "image_raw";
  if (image_topic.size() >= kRaw.size() &&
    image_topic.compare(image_topic.size() - kRaw.size(), kRaw.size(), kRaw) == 0)
  {
    return image_topic.substr(0, image_topic.size() - kRaw.size()) + "camera_info";
  }
  const auto slash = image_topic.find_last_of('/');
  if (slash == std::string::npos) {
    return "camera_info";
  }
  return image_topic.substr(0, slash + 1) + "camera_info";
}

}  // namespace detail
}  // namespace udp_camera_ros
