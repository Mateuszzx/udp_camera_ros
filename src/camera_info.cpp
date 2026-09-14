/**
 * @file camera_info.cpp
 * @brief Calibration YAML loading and CameraInfo helpers (see camera_info.hpp).
 */

#include "udp_camera_ros/camera_info.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

namespace udp_camera_ros
{
namespace
{

/**
 * @brief Read a matrix block's `data:` sequence from a calib YAML document.
 * @param doc Root YAML node.
 * @param key Key such as `camera_matrix` or `projection_matrix`.
 * @return Flat row-major coefficient vector.
 * @throws std::runtime_error if the key or `data` field is missing.
 */
std::vector<double> matrix_data(const YAML::Node & doc, const char * key)
{
  const auto block = doc[key];
  if (!block || !block["data"]) {
    throw std::runtime_error(std::string("missing matrix '") + key + "'");
  }
  std::vector<double> out;
  for (const auto & v : block["data"]) {
    out.push_back(v.as<double>());
  }
  return out;
}

}  // namespace


std::string resolve_calib_path(const std::string & path_or_name)
{
  if (path_or_name.empty()) {
    throw std::runtime_error("calib_file is empty");
  }
  if (path_or_name.front() == '/') {
    return path_or_name;
  }

  constexpr std::string_view kPackagePrefix = "package://";
  if (path_or_name.compare(0, kPackagePrefix.size(), kPackagePrefix) == 0) {
    const auto rest = path_or_name.substr(kPackagePrefix.size());
    const auto slash = rest.find('/');
    if (slash == std::string::npos || slash == 0) {
      throw std::runtime_error("invalid package:// URL: " + path_or_name);
    }
    const auto pkg = rest.substr(0, slash);
    const auto rel = rest.substr(slash + 1);
    return ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
  }

  return ament_index_cpp::get_package_share_directory("udp_camera_ros") +
         "/config/" + path_or_name;
}


CalibData load_calib(const std::string & path)
{
  const YAML::Node doc = YAML::LoadFile(path);
  CalibData calib;
  calib.width = doc["image_width"].as<int>(0);
  calib.height = doc["image_height"].as<int>(0);
  if (calib.width <= 0 || calib.height <= 0) {
    throw std::runtime_error("image_width / image_height missing or invalid");
  }
  calib.camera_name = doc["camera_name"].as<std::string>("");
  calib.distortion_model = doc["distortion_model"].as<std::string>("plumb_bob");
  calib.K = matrix_data(doc, "camera_matrix");
  calib.D = matrix_data(doc, "distortion_coefficients");
  calib.R = matrix_data(doc, "rectification_matrix");
  calib.P = matrix_data(doc, "projection_matrix");
  if (calib.K.size() != 9 || calib.R.size() != 9 || calib.P.size() != 12) {
    throw std::runtime_error("unexpected K/R/P sizes in calibration file");
  }
  return calib;
}

namespace
{

std::vector<double> json_array(const YAML::Node & node, const char * key, size_t expect)
{
  const auto arr = node[key];
  if (!arr || !arr.IsSequence()) {
    throw std::runtime_error(std::string("meta missing array '") + key + "'");
  }
  std::vector<double> out;
  out.reserve(arr.size());
  for (const auto & v : arr) {
    out.push_back(v.as<double>());
  }
  if (expect != 0 && out.size() != expect) {
    throw std::runtime_error(
            std::string("meta '") + key + "' size " + std::to_string(out.size()) +
            " (expected " + std::to_string(expect) + ")");
  }
  return out;
}

}  // namespace

CalibData parse_meta_payload(
  const std::string & payload,
  bool * stream_undistorted_out)
{
  constexpr std::string_view kMagic = "UCAL1\n";
  if (payload.size() < kMagic.size() ||
    payload.compare(0, kMagic.size(), kMagic) != 0)
  {
    throw std::runtime_error("meta packet missing UCAL1 magic");
  }

  const YAML::Node doc = YAML::Load(payload.substr(kMagic.size()));
  CalibData calib;
  calib.width = doc["width"].as<int>(0);
  calib.height = doc["height"].as<int>(0);
  if (calib.width <= 0 || calib.height <= 0) {
    throw std::runtime_error("meta width/height missing or invalid");
  }
  calib.camera_name = doc["camera_name"].as<std::string>("");
  calib.distortion_model = doc["distortion_model"].as<std::string>("plumb_bob");
  calib.K = json_array(doc, "K", 9);
  calib.D = json_array(doc, "D", 0);
  calib.R = json_array(doc, "R", 9);
  calib.P = json_array(doc, "P", 12);
  if (calib.D.size() < 4) {
    throw std::runtime_error("meta D too short");
  }
  if (stream_undistorted_out && doc["stream_undistorted"]) {
    *stream_undistorted_out = doc["stream_undistorted"].as<bool>();
  }
  return calib;
}

sensor_msgs::msg::CameraInfo camera_info_from_calib(
  const CalibData & calib,
  const std::string & frame_id)
{
  sensor_msgs::msg::CameraInfo msg;
  msg.header.frame_id = frame_id;
  msg.width = static_cast<uint32_t>(calib.width);
  msg.height = static_cast<uint32_t>(calib.height);
  msg.distortion_model = calib.distortion_model;
  std::copy_n(calib.K.begin(), 9, msg.k.begin());
  msg.d = calib.D;
  std::copy_n(calib.R.begin(), 9, msg.r.begin());
  std::copy_n(calib.P.begin(), 12, msg.p.begin());
  return msg;
}

sensor_msgs::msg::CameraInfo build_camera_info(
  const CalibData & calib,
  const std::string & frame_id,
  bool stream_undistorted)
{
  sensor_msgs::msg::CameraInfo msg;
  msg.header.frame_id = frame_id;
  msg.width = static_cast<uint32_t>(calib.width);
  msg.height = static_cast<uint32_t>(calib.height);
  msg.distortion_model = calib.distortion_model;
  std::copy_n(calib.R.begin(), 9, msg.r.begin());
  std::copy_n(calib.P.begin(), 12, msg.p.begin());

  if (stream_undistorted) {
    const auto & p = calib.P;
    msg.k = {p[0], p[1], p[2], p[4], p[5], p[6], p[8], p[9], p[10]};
    msg.d.assign(std::max<size_t>(calib.D.size(), 5), 0.0);
  } else {
    std::copy_n(calib.K.begin(), 9, msg.k.begin());
    msg.d = calib.D;
  }
  return msg;
}

sensor_msgs::msg::CameraInfo stamp_camera_info(
  const sensor_msgs::msg::CameraInfo & template_info,
  const builtin_interfaces::msg::Time & stamp,
  int width,
  int height)
{
  sensor_msgs::msg::CameraInfo info = template_info;
  info.header.stamp = stamp;

  const int cal_w = static_cast<int>(template_info.width);
  const int cal_h = static_cast<int>(template_info.height);
  // Scale fx,fy,cx,cy when the decoded frame size ≠ calib size.
  // Without this, a 1536×864 K on a 768×432 image puts (cx,cy) near the
  // bottom-right corner — landmarks appear constantly offset.
  if (cal_w > 0 && cal_h > 0 && (width != cal_w || height != cal_h)) {
    const double sx = static_cast<double>(width) / static_cast<double>(cal_w);
    const double sy = static_cast<double>(height) / static_cast<double>(cal_h);
    info.k[0] *= sx;  // fx
    info.k[2] *= sx;  // cx
    info.k[4] *= sy;  // fy
    info.k[5] *= sy;  // cy
    info.p[0] *= sx;  // fx
    info.p[2] *= sx;  // cx
    info.p[3] *= sx;  // Tx
    info.p[5] *= sy;  // fy
    info.p[6] *= sy;  // cy
    info.p[7] *= sy;  // Ty
  }

  info.width = static_cast<uint32_t>(width);
  info.height = static_cast<uint32_t>(height);
  return info;
}

}  // namespace udp_camera_ros
