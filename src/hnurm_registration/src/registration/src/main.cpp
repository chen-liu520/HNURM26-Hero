#include "registration/registration_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // 使用 NodeOptions 创建节点
  rclcpp::NodeOptions options;
  auto node = std::make_shared<hnurm::RelocationNode>(options);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// 回调单独一个线程执行
/*
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 1. 创建节点
    auto node = std::make_shared<RelocationNode>(rclcpp::NodeOptions());

    // 2. 创建AsyncSpinner（自动处理线程）
    // 参数：线程数（0=使用CPU核心数）
    rclcpp::executors::AsyncSpinner spinner(0);
    spinner.attach_node(node);

    // 3. 启动异步执行器（非阻塞）
    spinner.start();

    // 4. 主线程继续执行自己的逻辑（与回调并行）
    while (rclcpp::ok()) {
        // 主线程的工作：如监控、日志、其他任务
        RCLCPP_INFO(node->get_logger(), "Main thread is running...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    rclcpp::shutdown();
    return 0;
}
*/