#include "hnurm_uart/uart_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
    auto node = std::make_shared<hnurm::UartNode>(options);
    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
}