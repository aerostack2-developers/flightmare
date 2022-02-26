#include "flightros/camera/camera.hpp"

Camera::Camera() : as2::Node("Camera") {
  // std::enable_shared_from_this<Camera>(),
  //   image_transport_(this->getSelfPtr()) {
  // // image_transport_ =
  // //
  // image_transport::ImageTransport(std::make_shared<rclcpp::Node>("Camera2"));
  // // unity quadrotor
  // quad_ptr_ = std::make_shared<flightlib::Quadrotor>();
  // rgb_camera_ = std::make_shared<flightlib::RGBCamera>();

  // rgb_pub_ = image_transport_.advertise(RGB_TOPIC, 1);
  // depth_pub_ = image_transport_.advertise(DEPTH_TOPIC, 1);
  // segmentation_pub_ = image_transport_.advertise(SEGMENT_TOPIC, 1);
  // opticalflow_pub_ = image_transport_.advertise(OPTFLOW_TOPIC, 1);

  // rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(RGB_TOPIC, 1);
  // depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(DEPTH_TOPIC,
  // 1); segmentation_pub_ =
  //   this->create_publisher<sensor_msgs::msg::Image>(SEGMENT_TOPIC, 1);
  // opticalflow_pub_ =
  //   this->create_publisher<sensor_msgs::msg::Image>(OPTFLOW_TOPIC, 1);
  quad_ptr_ = std::make_shared<flightlib::Quadrotor>();
  rgb_camera_ = std::make_shared<flightlib::RGBCamera>();

  // define quadsize scale (for unity visualization only)
  Eigen::Matrix<float, 3, 1> quad_size(0.5, 0.5, 0.5);
  quad_ptr_->setSize(quad_size);

  // Flightmare(Unity3D)
  unity_bridge_ptr_ = flightlib::UnityBridge::getInstance();

  // Flightmare
  flightlib::Vector<3> B_r_BC(0.0, 0.0, 0.3);
  flightlib::Matrix<3, 3> R_BC =
    flightlib::Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  // RCLCPP_INFO(this->get_logger(), R_BC); // FIXME
  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(640);
  rgb_camera_->setHeight(360);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->setPostProcesscing(
    std::vector<bool>{true, true, true});  // depth, segmentation, optical flow
  quad_ptr_->addRGBCamera(rgb_camera_);
}

std::shared_ptr<rclcpp::Node> Camera::getSelfPtr() {
  // auto base = enable_shared_from_this<Camera>::shared_from_this();
  // std::shared_ptr<rclcpp::Node> derived =
  //   std::dynamic_pointer_cast<rclcpp::Node>(base);
  // return derived;
  static std::shared_ptr<rclcpp::Node> ptr =
    std::make_shared<rclcpp::Node>("camera_test");
  return ptr;
}

void Camera::setup() {
  image_transport_ptr_ =
    new image_transport::ImageTransport(this->getSelfPtr());
  image_transport::ImageTransport& image_transport_ = *image_transport_ptr_;
  // image_transport_ =
  //   image_transport::ImageTransport(std::make_shared<rclcpp::Node>("Camera2"));
  // unity quadrotor

  rgb_pub_ = image_transport_.advertise(RGB_TOPIC, 1);
  depth_pub_ = image_transport_.advertise(DEPTH_TOPIC, 1);
  segmentation_pub_ = image_transport_.advertise(SEGMENT_TOPIC, 1);
  opticalflow_pub_ = image_transport_.advertise(OPTFLOW_TOPIC, 1);


  scene_id_ = flightlib::UnityScene::WAREHOUSE;

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);

  // connect unity
  unity_bridge_ptr_->addQuadrotor(quad_ptr_);
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);

  frame_id_ = 0;
}

void Camera::run() {
  if (!unity_ready_) {
    RCLCPP_ERROR(this->get_logger(), "Can't connect with Unity!");
    return;
  }

  quad_state_.x[flightlib::QuadState::POSZ] += 0.1;  // Fly up!

  quad_ptr_->setState(quad_state_);

  unity_bridge_ptr_->getRender(frame_id_);
  unity_bridge_ptr_->handleOutput();

  cv::Mat img;

  rclcpp::Time timestamp = this->get_clock()->now();

  rgb_camera_->getRGBImage(img);
  sensor_msgs::msg::Image::SharedPtr rgb_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
  rgb_msg->header.stamp = timestamp;
  rgb_pub_.publish(rgb_msg);

  rgb_camera_->getDepthMap(img);
  sensor_msgs::msg::Image::SharedPtr depth_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", img).toImageMsg();
  depth_msg->header.stamp = timestamp;
  depth_pub_.publish(depth_msg);

  rgb_camera_->getSegmentation(img);
  sensor_msgs::msg::Image::SharedPtr segmentation_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
  segmentation_msg->header.stamp = timestamp;
  segmentation_pub_.publish(segmentation_msg);

  rgb_camera_->getOpticalFlow(img);
  sensor_msgs::msg::Image::SharedPtr opticflow_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
  opticflow_msg->header.stamp = timestamp;
  opticalflow_pub_.publish(opticflow_msg);

  frame_id_ += 1;
}

// // using namespace flightlib;
// int main(int argc, char *argv[]) {
//   // initialize ROS
//   // ros::init(argc, argv, "camera_example");
//   // ros::NodeHandle nh("");
//   // ros::NodeHandle pnh("~");
//   // ros::Rate(50.0);

//   // publisher
//   // image_transport::Publisher rgb_pub;
//   // image_transport::Publisher depth_pub;
//   // image_transport::Publisher segmentation_pub;
//   // image_transport::Publisher opticalflow_pub;

//   // // unity quadrotor
//   // std::shared_ptr<Quadrotor> quad_ptr = std::make_shared<Quadrotor>();
//   // // define quadsize scale (for unity visualization only)
//   // Vector<3> quad_size(0.5, 0.5, 0.5);
//   // quad_ptr->setSize(quad_size);
//   // QuadState quad_state;

//   //
//   // std::shared_ptr<RGBCamera> rgb_camera = std::make_shared<RGBCamera>();

//   // // Flightmare(Unity3D)
//   // std::shared_ptr<UnityBridge> unity_bridge_ptr =
//   UnityBridge::getInstance();
//   // SceneID scene_id{UnityScene::WAREHOUSE};
//   // bool unity_ready{false};

//   // // initialize publishers
//   // image_transport::ImageTransport it(pnh);
//   // rgb_pub = it.advertise("/rgb", 1);
//   // depth_pub = it.advertise("/depth", 1);
//   // segmentation_pub = it.advertise("/segmentation", 1);
//   // opticalflow_pub = it.advertise("/opticalflow", 1);

//   // // Flightmare
//   // Vector<3> B_r_BC(0.0, 0.0, 0.3);
//   // Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
//   // std::cout << R_BC << std::endl;
//   // rgb_camera->setFOV(90);
//   // rgb_camera->setWidth(640);
//   // rgb_camera->setHeight(360);
//   // rgb_camera->setRelPose(B_r_BC, R_BC);
//   // rgb_camera->setPostProcesscing(
//   //   std::vector<bool>{true, true, true});  // depth, segmentation, optical
//   //   flow
//   // quad_ptr->addRGBCamera(rgb_camera);

//   // // initialization
//   // quad_state.setZero();
//   // quad_ptr->reset(quad_state);

//   // // connect unity
//   // unity_bridge_ptr->addQuadrotor(quad_ptr);
//   // unity_ready = unity_bridge_ptr->connectUnity(scene_id);

//   FrameID frame_id = 0;
//   while (ros::ok() && unity_ready) {
//     quad_state.x[QS::POSZ] += 0.1;

//     quad_ptr->setState(quad_state);

//     unity_bridge_ptr->getRender(frame_id);
//     unity_bridge_ptr->handleOutput();

//     cv::Mat img;

//     ros::Time timestamp = ros::Time::now();

//     rgb_camera->getRGBImage(img);
//     sensor_msgs::ImagePtr rgb_msg =
//       cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
//     rgb_msg->header.stamp = timestamp;
//     rgb_pub.publish(rgb_msg);

//     rgb_camera->getDepthMap(img);
//     sensor_msgs::ImagePtr depth_msg =
//       cv_bridge::CvImage(std_msgs::Header(), "32FC1", img).toImageMsg();
//     depth_msg->header.stamp = timestamp;
//     depth_pub.publish(depth_msg);

//     rgb_camera->getSegmentation(img);
//     sensor_msgs::ImagePtr segmentation_msg =
//       cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
//     segmentation_msg->header.stamp = timestamp;
//     segmentation_pub.publish(segmentation_msg);

//     rgb_camera->getOpticalFlow(img);
//     sensor_msgs::ImagePtr opticflow_msg =
//       cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
//     opticflow_msg->header.stamp = timestamp;
//     opticalflow_pub.publish(opticflow_msg);

//     frame_id += 1;
//   }

//   return 0;
// }
