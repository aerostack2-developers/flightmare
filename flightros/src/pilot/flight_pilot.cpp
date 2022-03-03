#include "flightros/pilot/flight_pilot.hpp"

FlightPilot::FlightPilot()
  : as2::Node("FlightPilot"),
    scene_id_(flightlib::UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0) {
  sub_state_est_ = this->create_subscription<nav_msgs::msg::Odometry>(
    STATE_TOPIC, 1,
    std::bind(&FlightPilot::poseCallback, this, std::placeholders::_1));

  // Quad initialization
  quad_ptr_ = std::make_shared<flightlib::Quadrotor>();
  // Add mono camera
  rgb_camera_ = std::make_shared<flightlib::RGBCamera>();
}

void FlightPilot::setup() {
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
  // rgb_camera_->setRelPose(B_r_BC, R_BC);
  quad_ptr_->addRGBCamera(rgb_camera_);

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);

  // connect unity
  unity_render_ = true;  // FIXME
  setUnity(unity_render_);
  connectUnity();
}

void FlightPilot::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = flightlib::UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    RCLCPP_INFO(this->get_logger(), "Unity Bridge is created.");
  }
  return true;
}

bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

std::shared_ptr<rclcpp::Node> FlightPilot::getSelfPtr() {
  // auto base = enable_shared_from_this<Camera>::shared_from_this();
  // std::shared_ptr<rclcpp::Node> derived =
  //   std::dynamic_pointer_cast<rclcpp::Node>(base);
  // return derived;
  // FIXME: Do not create a new Node
  auto ptr = std::make_shared<rclcpp::Node>("camera_test");
  return ptr;
}

void FlightPilot::run() {
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