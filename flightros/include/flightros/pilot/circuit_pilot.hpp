#ifndef CIRCUIT_PILOT_HPP_
#define CIRCUIT_PILOT_HPP_

#include <Eigen/Dense>
#include <memory>

// ros
#include <cv_bridge/cv_bridge.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "as2_core/node.hpp"

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/aruco_gate.hpp"
#include "flightlib/objects/dynamic_gate.hpp"
#include "flightlib/objects/object_base.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/objects/static_gate.hpp"
#include "flightlib/objects/static_object.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

// camera
#include <image_transport/image_transport.hpp>

#include "flightlib/bridges/unity_message_types.hpp"

#define STATE_TOPIC "self_localization/odom"
#define RGB_TOPIC "camera1/image_raw"
// #define DEPTH_TOPIC "depht"
// #define SEGMENT_TOPIC "segmentation"
// #define OPTFLOW_TOPIC "optical_flow"

class CircuitPilot : public as2::Node {
 public:
  CircuitPilot();
  ~CircuitPilot() { delete image_transport_ptr_; };
  void setup();
  void run();
  std::shared_ptr<rclcpp::Node> getSelfPtr();

  // callbacks
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  bool setUnity(const bool render);
  bool connectUnity(void);

 private:
  // subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_state_est_;

  // unity quadrotor
  std::shared_ptr<flightlib::Quadrotor> quad_ptr_;
  std::shared_ptr<flightlib::RGBCamera> rgb_camera_;
  std::shared_ptr<ArucoGate> gate1;
  std::shared_ptr<flightlib::StaticGate> gate2;
  std::shared_ptr<flightlib::StaticGate> gate3;
  std::shared_ptr<flightlib::StaticGate> gate4;
  std::shared_ptr<flightlib::StaticGate> gate5;
  std::shared_ptr<flightlib::StaticGate> gate6;
  std::shared_ptr<flightlib::StaticGate> gate7;
  std::shared_ptr<flightlib::StaticGate> gate8;
  std::shared_ptr<flightlib::StaticGate> gate9;
  flightlib::QuadState quad_state_;

  // Flightmare(Unity3D)
  std::shared_ptr<flightlib::UnityBridge> unity_bridge_ptr_;
  flightlib::SceneID scene_id_{flightlib::UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  flightlib::RenderMessage_t unity_output_;
  uint16_t receive_id_{0};

  // camera
  flightlib::FrameID frame_id_;
  image_transport::ImageTransport* image_transport_ptr_ = nullptr;
  image_transport::Publisher rgb_pub_;
};

#endif  // FLIGHT_PILOT_HPP_