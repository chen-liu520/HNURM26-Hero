#include "LIVMapper.h"
#include <csignal>
#include <atomic>

static std::atomic<bool> g_shutdown_requested(false);

void signalHandler(int sig)
{
  (void)sig;
  std::cout << "\n[Signal Handler] Shutdown requested (Ctrl+C), finishing current processing..." << std::endl;
  g_shutdown_requested.store(true);
  // Don't call rclcpp::shutdown() here, let main() handle it gracefully
}

int main(int argc, char **argv)
{
  // Set up custom signal handler for SIGINT (Ctrl+C)
  std::signal(SIGINT, signalHandler);
  
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr nh;
  image_transport::ImageTransport it_(nh);
  LIVMapper mapper(nh, "laserMapping", options);
  mapper.initializeSubscribersAndPublishers(nh, it_);
  mapper.run(nh, g_shutdown_requested);
  
  std::cout << "[Main] Shutting down ROS2..." << std::endl;
  rclcpp::shutdown();
  return 0;
}
