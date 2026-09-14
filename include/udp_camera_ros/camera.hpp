#pragma once

#include <memory>
#include <string>

#include "udp_camera_ros/udp_stream.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace udp_camera_ros
{

/**
 * @brief Thin adapter between the lifecycle node and @ref UdpStream.
 *
 * Reads ROS parameters and the local calibration YAML, builds a
 * @ref UdpStreamConfig, then forwards configure / start / stop / cleanup
 * to the UDP receiver. Keeps lifecycle callbacks free of GStreamer details.
 */
class Camera
{
public:
  /**
   * @brief Construct an adapter bound to a lifecycle node.
   * @param node Shared pointer to the owning `camera_node` (for params + logging).
   */
  explicit Camera(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief Load calib + params and configure the underlying @ref UdpStream.
   *
   * Does not open the UDP socket yet — that happens in @ref start so
   * lifecycle configure stays non-blocking.
   */
  void configure();

  /**
   * @brief Start receiving UDP H.264 and publishing ROS images.
   * @throws std::runtime_error if called before @ref configure.
   */
  void start();

  /** @brief Stop the grab thread and release the VideoCapture. */
  void stop();

  /**
   * @brief Stop streaming and tear down publishers / transport node.
   *
   * Safe to call multiple times; leaves the adapter ready for a new
   * @ref configure after the lifecycle returns to unconfigured.
   */
  void cleanup();

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::unique_ptr<UdpStream> stream_;
};

}  // namespace udp_camera_ros
