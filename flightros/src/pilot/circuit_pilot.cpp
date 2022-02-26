#include "flightros/pilot/circuit_pilot.hpp"

CircuitPilot::CircuitPilot()
  : as2::Node("CircuitPilot"),
    scene_id_(flightlib::UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0) {
  sub_state_est_ = this->create_subscription<nav_msgs::msg::Odometry>(
    STATE_TOPIC, 1,
    std::bind(&CircuitPilot::poseCallback, this, std::placeholders::_1));

  // Quad initialization
  quad_ptr_ = std::make_shared<flightlib::Quadrotor>();
  // Add mono camera
  rgb_camera_ = std::make_shared<flightlib::RGBCamera>();

  std::string object_id1 = "unity_gate1";
  std::string prefab_id1 = "aruco_gate2";
  gate1 = std::make_shared<ArucoGate>(object_id1, prefab_id1);
  gate1->setPosition(Eigen::Vector3f(-5.0, 5.0, 1.5));
  gate1->setQuaternion(flightlib::Quaternion(0.0, 0.0, 0.0, 1.0));
  std::string object_id2 = "unity_gate2";
  std::string prefab_id2 = "aruco_gate2";
  gate2 = std::make_shared<flightlib::StaticGate>(object_id2, prefab_id2);
  gate2->setPosition(Eigen::Vector3f(5.0, 10.0, 2.0));
  gate2->setQuaternion(flightlib::Quaternion(0.0, 0.0, 0.0, 1.0));
  std::string object_id3 = "unity_gate3";
  std::string prefab_id3 = "aruco_gate2";
  gate3 = std::make_shared<flightlib::StaticGate>(object_id3, prefab_id3);
  gate3->setPosition(Eigen::Vector3f(-5.0, 10.0, 3.0));
  gate3->setQuaternion(flightlib::Quaternion(0.0, 0.0, 0.0, 1.0));
  std::string object_id4 = "unity_gate4";
  std::string prefab_id4 = "aruco_gate2";
  gate4 = std::make_shared<flightlib::StaticGate>(object_id4, prefab_id4);
  gate4->setPosition(Eigen::Vector3f(5.0, 5.0, 2.5));
  gate4->setQuaternion(flightlib::Quaternion(0.0, 0.0, 0.0, 0.1));
  // std::string object_id5 = "unity_gate5";
  // std::string prefab_id5 = "aruco_gate2";
  // gate5 = std::make_shared<flightlib::StaticGate>(object_id5, prefab_id5);
  // gate5->setPosition(Eigen::Vector3f(17, -4, 0));
  // gate5->setQuaternion(flightlib::Quaternion(0.7071068, 0.0, 0.0,
  // 0.7071068)); std::string object_id6 = "unity_gate6"; std::string prefab_id6
  // = "aruco_gate2"; gate6 =
  // std::make_shared<flightlib::StaticGate>(object_id6, prefab_id6);
  // gate6->setPosition(Eigen::Vector3f(16, -6, 0));
  // gate6->setQuaternion(flightlib::Quaternion(0.9239, 0.0, 0.0, 0.3827));
  // std::string object_id7 = "unity_gate7";
  // std::string prefab_id7 = "aruco_gate2";
  // gate7 = std::make_shared<flightlib::StaticGate>(object_id7, prefab_id7);
  // gate7->setPosition(Eigen::Vector3f(14, -8, 0));
  // gate7->setQuaternion(flightlib::Quaternion(0.0, 0.0, 0.0, 0.0));
  // std::string object_id8 = "unity_gate8";
  // std::string prefab_id8 = "aruco_gate2";
  // gate8 = std::make_shared<flightlib::StaticGate>(object_id8, prefab_id8);
  // gate8->setPosition(Eigen::Vector3f(10, -8, 0));
  // gate8->setQuaternion(flightlib::Quaternion(0.0, 0.0, 0.0, 0.0));
  // std::string object_id9 = "unity_gate9";
  // std::string prefab_id9 = "aruco_gate2";
  // gate9 = std::make_shared<flightlib::StaticGate>(object_id9, prefab_id9);
  // gate9->setPosition(Eigen::Vector3f(6, -8, 0));
  // gate9->setQuaternion(flightlib::Quaternion(0.0, 0.0, 0.0, 0.0));
}

void CircuitPilot::setup() {
  // camera
  image_transport_ptr_ =
    new image_transport::ImageTransport(this->getSelfPtr());
  image_transport::ImageTransport& image_transport_ = *image_transport_ptr_;

  rgb_pub_ = image_transport_.advertise(RGB_TOPIC, 1);
  frame_id_ = 0;

  flightlib::Vector<3> B_r_BC(0.0, 0.0, 0.3);
  flightlib::Matrix<3, 3> R_BC =
    flightlib::Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();

  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(720);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  quad_ptr_->addRGBCamera(rgb_camera_);

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);

  // connect unity
  unity_render_ = true;  // FIXME
  setUnity(unity_render_);
  connectUnity();
}

void CircuitPilot::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Position
  quad_state_.x[flightlib::QuadState::POSX] = (float)msg->pose.pose.position.x;
  quad_state_.x[flightlib::QuadState::POSY] = (float)msg->pose.pose.position.y;
  quad_state_.x[flightlib::QuadState::POSZ] = (float)msg->pose.pose.position.z;
  // Orientation
  quad_state_.x[flightlib::QuadState::ATTW] =
    (float)msg->pose.pose.orientation.w;
  quad_state_.x[flightlib::QuadState::ATTX] =
    (float)msg->pose.pose.orientation.x;
  quad_state_.x[flightlib::QuadState::ATTY] =
    (float)msg->pose.pose.orientation.y;
  quad_state_.x[flightlib::QuadState::ATTZ] =
    (float)msg->pose.pose.orientation.z;

  quad_ptr_->setState(quad_state_);

  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();

    if (quad_ptr_->getCollision()) {
      RCLCPP_INFO(this->get_logger(), "COLLISION");
    }
  }
}

bool CircuitPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = flightlib::UnityBridge::getInstance();
    unity_bridge_ptr_->addStaticObject(gate1);
    unity_bridge_ptr_->addStaticObject(gate2);
    unity_bridge_ptr_->addStaticObject(gate3);
    unity_bridge_ptr_->addStaticObject(gate4);
    // unity_bridge_ptr_->addStaticObject(gate5);
    // unity_bridge_ptr_->addStaticObject(gate6);
    // unity_bridge_ptr_->addStaticObject(gate7);
    // unity_bridge_ptr_->addStaticObject(gate8);
    // unity_bridge_ptr_->addStaticObject(gate9);
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    RCLCPP_INFO(this->get_logger(), "Unity Bridge is created.");
  }
  return true;
}

bool CircuitPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

std::shared_ptr<rclcpp::Node> CircuitPilot::getSelfPtr() {
  // auto base = enable_shared_from_this<Camera>::shared_from_this();
  // std::shared_ptr<rclcpp::Node> derived =
  //   std::dynamic_pointer_cast<rclcpp::Node>(base);
  // return derived;
  static std::shared_ptr<rclcpp::Node> ptr =
    std::make_shared<rclcpp::Node>("camera_test");
  return ptr;
}

void CircuitPilot::run() {
  // camera
  if (!unity_ready_) {
    RCLCPP_ERROR(this->get_logger(), "Can't connect with Unity!");
    return;
  }

  quad_state_.x[flightlib::QuadState::POSZ] += 0.1;

  quad_ptr_->setState(quad_state_);

  // unity_bridge_ptr_->getRender(frame_id_);
  // unity_bridge_ptr_->handleOutput();

  cv::Mat img;
  rclcpp::Time timestamp = this->get_clock()->now();

  rgb_camera_->getRGBImage(img);
  sensor_msgs::msg::Image::SharedPtr rgb_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
  rgb_msg->header.stamp = timestamp;
  rgb_pub_.publish(rgb_msg);

  // rgb_camera_->getDepthMap(img);
  // sensor_msgs::msg::Image::SharedPtr depth_msg =
  //   cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", img).toImageMsg();
  // depth_msg->header.stamp = timestamp;
  // depth_pub_.publish(depth_msg);

  // rgb_camera_->getSegmentation(img);
  // sensor_msgs::msg::Image::SharedPtr segmentation_msg =
  //   cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
  // segmentation_msg->header.stamp = timestamp;
  // segmentation_pub_.publish(segmentation_msg);

  // rgb_camera_->getOpticalFlow(img);
  // sensor_msgs::msg::Image::SharedPtr opticflow_msg =
  //   cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
  // opticflow_msg->header.stamp = timestamp;
  // opticalflow_pub_.publish(opticflow_msg);

  frame_id_ += 1;
}