#ifndef FLIGHT_PILOT_HPP_
#define FLIGHT_PILOT_HPP_

#include <Eigen/Dense>
#include <memory>
#include <vector>
// #include <math>

// ros
#include <cv_bridge/cv_bridge.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>

#include "as2_core/node.hpp"
#include "as2_core/names/topics.hpp"

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

// camera
#include <image_transport/image_transport.hpp>

#include "flightlib/bridges/unity_message_types.hpp"

#define RGB_TOPIC "camera2/image_raw"
// #define DEPTH_TOPIC "depht"
// #define SEGMENT_TOPIC "segmentation"
// #define OPTFLOW_TOPIC "optical_flow"

class FlightPilot : public as2::Node {
 public:
  FlightPilot();
  ~FlightPilot() { delete image_transport_ptr_; };
  void setup();
  void run();
  virtual std::shared_ptr<rclcpp::Node> getSelfPtr();

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
  flightlib::QuadState quad_state_;

  // model
  std::string model_;

  // initial pose
  std::vector<double> pose_0_;  // {x, y, z, yaw}

  // Flightmare(Unity3D)
  std::shared_ptr<flightlib::UnityBridge> unity_bridge_ptr_;
  flightlib::SceneID scene_id_{flightlib::UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  flightlib::RenderMessage_t unity_output_;

  // camera
  std::vector<double> cam_pose_;
  std::vector<double> cam_orient_;
  flightlib::FrameID frame_id_;
  image_transport::ImageTransport* image_transport_ptr_ = nullptr;
  image_transport::Publisher rgb_pub_;
};

#endif  // FLIGHT_PILOT_HPP_