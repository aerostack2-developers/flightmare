#include "flightros/pilot/flight_pilot.hpp"

FlightPilot::FlightPilot(): as2::Node("FlightPilot")
{
  this->declare_parameter<bool>("render", true);
  this->get_parameter("render", unity_render_);

  this->declare_parameter<int>("scene_id", flightlib::UnityScene::ARUCO);
  this->get_parameter("scene_id", scene_id_);
  
  this->declare_parameter<std::vector<double>>("drone.pose", {0.0, 0.0, 0.0, 0.0});
  this->get_parameter("drone.pose", pose_0_);

  this->declare_parameter<std::vector<double>>("drone.cam.pose", {0.0, 0.0, 0.3});
  this->get_parameter("drone.cam.pose", cam_pose_);

  this->declare_parameter<std::vector<double>>("drone.cam.orientation", {0.0, 0.0, 0.0});
  this->get_parameter("drone.cam.orientation", cam_orient_);

  this->declare_parameter<std::string>("pub_port", "10253");
  this->get_parameter("pub_port", p_port);

  this->declare_parameter<std::string>("sub_port", "10254");
  this->get_parameter("sub_port", s_port);

  sub_state_est_ = this->create_subscription<nav_msgs::msg::Odometry>(
    as2_names::topics::sensor_measurements::odom, 
    as2_names::topics::sensor_measurements::qos,
    std::bind(&FlightPilot::poseCallback, this, std::placeholders::_1));

  // Quad initialization
  quad_ptr_ = std::make_shared<flightlib::Quadrotor>();
  // Add mono camera
  rgb_camera_ = std::make_shared<flightlib::RGBCamera>();
}

void FlightPilot::setup() {
    std::string object_id = "unity_gate1";
    std::string prefab_id = "";
    if (p_port == "10253"){
      prefab_id = "aruco_gate2";
    }
    else{
      prefab_id = "aruco_gate_green";
    }
    gate = std::make_shared<flightlib::StaticGate>(object_id, prefab_id);
    gate->setPosition(Eigen::Vector3f(0, 0, 0));
    gate->setQuaternion(
    flightlib::Quaternion(0.0, 0.0, 0.0, 1.0));
  tf2::Quaternion cam_quad;
  cam_quad.setRPY(-cam_orient_[2], cam_orient_[0], -cam_orient_[1]);
  cam_quad = cam_quad.normalize();

  flightlib::Vector<3> B_r_BC(cam_pose_[0], cam_pose_[1], cam_pose_[2]);
  flightlib::Matrix<3, 3> R_BC =
  flightlib::Quaternion(0.0, 0.0, 0.0, 1.0).toRotationMatrix();
  /*flightlib::Matrix<3, 3> R_BC = flightlib::Quaternion(cam_quad.getX(),
                                                       cam_quad.getY(),
                                                       cam_quad.getZ(),
                                                       cam_quad.getW()).toRotationMatrix();*/

  image_transport_ptr_ =
    new image_transport::ImageTransport(this->getSelfPtr());
  image_transport::ImageTransport& image_transport_ = *image_transport_ptr_;

  if (p_port == "10253"){
    rgb_pub_ = image_transport_.advertise(RGB_TOPIC, 1);
  }
  else{
    rgb_pub_ = image_transport_.advertise(RGB_TOPIC_2, 1);
  }
  
  frame_id_ = 0;

  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(720);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  quad_ptr_->addRGBCamera(rgb_camera_);

  // initialization
  quad_state_.setZero();
  quad_state_.x[flightlib::QuadState::POSX] = pose_0_[0];
  quad_state_.x[flightlib::QuadState::POSY] = pose_0_[1];
  quad_state_.x[flightlib::QuadState::POSZ] = pose_0_[2];
  quad_ptr_->reset(quad_state_);

  // connect unity
  setUnity(unity_render_);
  connectUnity();
}

void FlightPilot::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Position
  
  quad_state_.x[flightlib::QuadState::POSX] = (float)msg->pose.pose.position.x + pose_0_[0];
  quad_state_.x[flightlib::QuadState::POSY] = (float)msg->pose.pose.position.y + pose_0_[1];
  quad_state_.x[flightlib::QuadState::POSZ] = (float)msg->pose.pose.position.z + pose_0_[2];
  // Orientation
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setX(msg->pose.pose.orientation.x);
  tf_quaternion.setY(msg->pose.pose.orientation.y);
  tf_quaternion.setZ(msg->pose.pose.orientation.z);
  tf_quaternion.setW(msg->pose.pose.orientation.w);

  tf2::Matrix3x3 rotation_matrix(tf_quaternion);
  double roll, pitch, yaw;
  rotation_matrix.getRPY(roll, pitch, yaw);
  tf_quaternion.setRPY(pitch, -roll, (yaw + M_PI_2));

  // Orientation
  quad_state_.x[flightlib::QuadState::ATTX] = (float)tf_quaternion.x();
  quad_state_.x[flightlib::QuadState::ATTY] = (float)tf_quaternion.y();
  quad_state_.x[flightlib::QuadState::ATTZ] = (float)tf_quaternion.z();
  quad_state_.x[flightlib::QuadState::ATTW] = (float)tf_quaternion.w();

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
    unity_bridge_ptr_ = flightlib::UnityBridge::getInstance(p_port, s_port);
    unity_bridge_ptr_->addStaticObject(gate);
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
  return this->shared_from_this();
}

void FlightPilot::run() {
  
  if (image_transport_ptr_ != nullptr ) { 
    
    // camera
    if (!unity_ready_) {
      RCLCPP_ERROR(this->get_logger(), "Can't connect with Unity!");
      return;
    }

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
}