#ifndef CAMERA_HPP_
#define CAMERA_HPP_

// ros
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// #include "as2_core/core_functions.hpp"
#include "as2_core/node.hpp"

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

#define RGB_TOPIC "camera1/image_raw"
#define DEPTH_TOPIC "depht"
#define SEGMENT_TOPIC "segmentation"
#define OPTFLOW_TOPIC "optical_flow"

class Camera : public as2::Node, std::enable_shared_from_this<Camera> {
 public:
  Camera();
  ~Camera() { delete image_transport_ptr_; };
  void setup();
  void run();
  std::shared_ptr<rclcpp::Node> getSelfPtr();

 private:
  image_transport::ImageTransport* image_transport_ptr_ = nullptr;
  /** Publishers **/

  image_transport::Publisher rgb_pub_;
  image_transport::Publisher depth_pub_;
  image_transport::Publisher segmentation_pub_;
  image_transport::Publisher opticalflow_pub_;

  // unity quadrotor
  std::shared_ptr<flightlib::Quadrotor> quad_ptr_;
  // define quadsize scale (for uniy visualization only)
  flightlib::QuadState quad_state_;

  // Flightmare(Unity3D)
  std::shared_ptr<flightlib::UnityBridge> unity_bridge_ptr_;
  flightlib::SceneID scene_id_;
  // flightlib::SceneID scene_id_;{flightlib::UnityScene::WAREHOUSE};
  bool unity_ready_{false};

  // camera
  std::shared_ptr<flightlib::RGBCamera> rgb_camera_;
  flightlib::FrameID frame_id_;
};

#endif