/**
 * @file trigger_hero_node.cpp
 * @brief 键盘触发 Hero 重定位模式
 * 
 * 功能：
 * - 等待用户按回车键
 * - 发送服务请求到 /trigger_hero_relocation
 * - 显示响应结果
 * 
 * 使用方法：
 *   ros2 run trigger_registration trigger_hero_node
 * 
 * 按回车键触发 Hero 模式
 * 按 Ctrl+C 退出
 */

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class TriggerHeroNode : public rclcpp::Node
{
public:
    TriggerHeroNode() : Node("trigger_hero_node")
    {
        // 创建服务客户端
        client_ = this->create_client<std_srvs::srv::Trigger>("trigger_hero_relocation");
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "  Hero 重定位触发器已启动");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "等待服务 /trigger_hero_relocation ...");
        
        // 等待服务可用
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "被打断，退出");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "服务不可用，等待中...");
        }
        
        RCLCPP_INFO(this->get_logger(), "服务已连接！");
        RCLCPP_INFO(this->get_logger(), "使用说明：");
        RCLCPP_INFO(this->get_logger(), "  [回车键] - 触发 Hero 高精度重定位");
        RCLCPP_INFO(this->get_logger(), "  [Ctrl+C] - 退出程序");
        RCLCPP_INFO(this->get_logger(), "等待输入...");
        
        // 启动键盘监听线程
        keyboard_thread_ = std::thread(&TriggerHeroNode::keyboardLoop, this);
    }
    
    ~TriggerHeroNode()
    {
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }

private:
    void keyboardLoop()
    {
        std::string line;
        while (rclcpp::ok()) {
            // 读取一行输入（等待回车）
            std::getline(std::cin, line);
            
            if (!rclcpp::ok()) {
                break;
            }
            
            // 发送触发请求
            triggerHero();
            
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            RCLCPP_INFO(this->get_logger(), "等待下一次输入...");
        }
    }
    
    void triggerHero()
    {
        RCLCPP_INFO(this->get_logger(), "[触发] 发送 Hero 模式请求...");
        
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        
        // 异步发送请求
        auto future = client_->async_send_request(request);
        
        // 等待响应（最多5秒）
        auto status = future.wait_for(5s);
        
        if (status == std::future_status::ready) {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "[成功] %s", response->message.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "[失败] %s", response->message.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "[异常] 服务调用失败: %s", e.what());
            }
        } else if (status == std::future_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "[超时] 服务响应超时（5秒）");
        }
    }

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    std::thread keyboard_thread_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TriggerHeroNode>();
    
    // 使用 spin 保持节点运行
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
