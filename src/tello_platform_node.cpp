#include "tello_platform.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto tello_node = std::make_shared<TelloPlatform>();
  tello_node->preset_loop_frequency(300);

  as2::spinLoop(tello_node);

  rclcpp::shutdown();
  return 0;
}
