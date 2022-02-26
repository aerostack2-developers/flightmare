#include "as2_core/core_functions.hpp"
#include "flightros/camera/camera.hpp"

int main(int argc, char *argv[]) {
  // initialize ROS2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Camera>();
  node->preset_loop_frequency(50);  // Node frequency for run and callbacks
  node->setup();
  // Node with only callbacks
  // as2::spinLoop(node);
  // Node with run
  as2::spinLoop(node, std::bind(&Camera::run, node));

  rclcpp::shutdown();
  return 0;
}