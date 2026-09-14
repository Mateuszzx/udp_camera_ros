#pragma once

#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace udp_camera_ros
{

/**
 * @brief Intrinsics / extrinsics loaded from a ROS camera calibration YAML.
 *
 * Layout matches `camera_calibration` / `camera_info_manager` files:
 * `camera_matrix` (K 3x3), `distortion_coefficients` (D),
 * `rectification_matrix` (R 3x3), `projection_matrix` (P 3x4).
 */
struct CalibData
{
  int width{0};   ///< Calibrated image width [px].
  int height{0};  ///< Calibrated image height [px].
  std::string camera_name;
  std::string distortion_model{"plumb_bob"};
  std::vector<double> K;  ///< Row-major 3x3 camera matrix (9 elems).
  std::vector<double> D;  ///< Distortion coefficients.
  std::vector<double> R;  ///< Row-major 3x3 rectification matrix (9 elems).
  std::vector<double> P;  ///< Row-major 3x4 projection matrix (12 elems).
};

/**
 * @brief Resolve a calibration file path.
 *
 * Accepts:
 * - absolute filesystem path (`/...`)
 * - `package://pkg_name/relative/path.yaml`
 * - basename under `share/udp_camera_ros/config/`
 *
 * @param path_or_name Path, package URL, or config basename.
 * @return Absolute filesystem path to the calibration file.
 * @throws std::runtime_error if the path cannot be resolved.
 */
std::string resolve_calib_path(const std::string & path_or_name);

/**
 * @brief Parse a camera calibration YAML into @ref CalibData.
 * @param path Absolute path to the YAML file.
 * @return Populated calibration data.
 * @throws std::runtime_error on missing/invalid fields or matrix sizes.
 */
CalibData load_calib(const std::string & path);

/**
 * @brief Parse a UCAL1 sideband UDP payload into @ref CalibData.
 *
 * Payload: magic `UCAL1\n` + JSON object with width/height/K/D/R/P
 * (and optional stream_undistorted). JSON is accepted via yaml-cpp.
 *
 * @param payload Raw UDP datagram bytes.
 * @param stream_undistorted_out Optional; set from JSON when present.
 * @return Parsed calibration at stream resolution.
 * @throws std::runtime_error on bad magic / JSON / sizes.
 */
CalibData parse_meta_payload(
  const std::string & payload,
  bool * stream_undistorted_out = nullptr);

/**
 * @brief Build CameraInfo directly from flat K/D/R/P (stream meta path).
 *
 * Unlike @ref build_camera_info, does not re-derive K from P — the sender
 * already applied undistort / scale for the live resolution.
 */
sensor_msgs::msg::CameraInfo camera_info_from_calib(
  const CalibData & calib,
  const std::string & frame_id);

/**
 * @brief Build a reusable CameraInfo template from file calibration.
 *
 * When @p stream_undistorted is true (sender already remaps with OpenCV),
 * published K is taken from P's left 3x3 and D is zeroed so consumers do
 * not undistort twice.
 *
 * @param calib Loaded calibration.
 * @param frame_id TF frame for `header.frame_id`.
 * @param stream_undistorted True if the UDP video is already undistorted.
 * @return CameraInfo without a stamp (stamp per frame via @ref stamp_camera_info).
 */
sensor_msgs::msg::CameraInfo build_camera_info(
  const CalibData & calib,
  const std::string & frame_id,
  bool stream_undistorted);

/**
 * @brief Copy a CameraInfo template and stamp it for the current frame.
 * @param template_info Intrinsics from @ref build_camera_info.
 * @param stamp ROS time to put on `header.stamp`.
 * @param width Actual frame width (may differ from calib if scaled).
 * @param height Actual frame height.
 * @return Stamped CameraInfo ready to publish with the Image.
 */
sensor_msgs::msg::CameraInfo stamp_camera_info(
  const sensor_msgs::msg::CameraInfo & template_info,
  const builtin_interfaces::msg::Time & stamp,
  int width,
  int height);

}  // namespace udp_camera_ros
