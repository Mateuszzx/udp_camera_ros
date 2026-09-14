/**
 * @file camera_node.cpp
 * @brief Lifecycle ROS 2 node entry point for the ground-station camera bridge.
 *
 * States:
 * - configure → load params/calib, advertise topics (@ref udp_camera_ros::Camera)
 * - activate  → start UDP grab thread
 * - deactivate / cleanup / shutdown → stop and release resources
 */

#include <memory>

#include "udp_camera_ros/camera.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace udp_camera_ros
{

/**
 * @brief Lifecycle node that owns a @ref Camera adapter.
 *
 * Declares parameters used by @ref Camera::configure. Autostart transitions
 * are driven from `camera_stream.launch.py`.
 */
class CameraNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Construct `camera_node` and declare stream / calib parameters.
   * @param options Standard rclcpp node options (remaps, params file, …).
   */
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("camera_node", options)
  {
    declare_parameter("port", 5000);
    declare_parameter("meta_port", -1);  // -1 → port+1; 0 disables
    declare_parameter("image_topic", "/camera/image_raw");
    declare_parameter("frame_id", "camera_optical_frame");
    declare_parameter("calib_source", "auto");  // auto | stream | file
    declare_parameter("calib_file", "");  // optional fallback / file mode
    declare_parameter("stream_undistorted", false);
    declare_parameter("udp_buffer_size", 212992);
    declare_parameter("queue_size", 1);
    declare_parameter("qos_reliability", "best_effort");
    declare_parameter("jitter_latency_ms", 80);
    declare_parameter("stall_timeout_ms", 1500);
    declare_parameter("reconnect_delay_ms", 400);
    declare_parameter("read_timeout_ms", 250);
    declare_parameter("h264_decoder", "auto");
    declare_parameter("publish_raw", false);

    RCLCPP_INFO(get_logger(), "Lifecycle: unconfigured");
  }

  /**
   * @brief unconfigured to inactive: build @ref Camera and configure publishers.
   * @return SUCCESS, or FAILURE if calib/params/configure throw.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "[lifecycle] configure  (from %s)", state.label().c_str());
    try {
      camera_ = std::make_unique<Camera>(
        std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(shared_from_this()));
      camera_->configure();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "configure failed: %s", e.what());
      if (camera_) {
        camera_->cleanup();
        camera_.reset();
      }
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "[lifecycle] inactive");
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief inactive to active: start UDP receive / publish thread.
   * @return SUCCESS, or FAILURE if start throws.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "[lifecycle] activate   (from %s)", state.label().c_str());
    try {
      camera_->start();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "activate failed: %s", e.what());
      camera_->stop();
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "[lifecycle] active");
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief active to inactive: stop the grab thread, keep publishers up.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "[lifecycle] deactivate (from %s)", state.label().c_str());
    if (camera_) {
      camera_->stop();
    }
    RCLCPP_INFO(get_logger(), "[lifecycle] inactive");
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief inactive to unconfigured: tear down Camera / publishers.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "[lifecycle] cleanup    (from %s)", state.label().c_str());
    if (camera_) {
      camera_->cleanup();
      camera_.reset();
    }
    RCLCPP_INFO(get_logger(), "[lifecycle] unconfigured");
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief Any state to finalized: force cleanup on process shutdown.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "[lifecycle] shutdown   (from %s)", state.label().c_str());
    if (camera_) {
      camera_->cleanup();
      camera_.reset();
    }
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief Error processing state to cleaned: release resources and recover to cleaned state.
   */
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_ERROR(get_logger(), "[lifecycle] error      (in %s)", state.label().c_str());
    if (camera_) {
      camera_->cleanup();
      camera_.reset();
    }
    return CallbackReturn::SUCCESS;
  }

private:
  std::unique_ptr<Camera> camera_;
};

}  // namespace udp_camera_ros

/**
 * @brief Process entry: init ROS, spin the lifecycle node until shutdown.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<udp_camera_ros::CameraNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
