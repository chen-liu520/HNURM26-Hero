#pragma once
#include "hnurm_uart/Serialcodec.h"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mutex>
namespace hnurm
{
struct init_imu
{
    bool is_init;
    float yaw;
    float pitch;
};
class UartNode : public rclcpp::Node
{
public:
    using UartNodePtr = std::shared_ptr<UartNode>;

    UartNode(const rclcpp::NodeOptions &options)
        : rclcpp::Node("hnurm_uart_node", options),
          logger(get_logger()),
          serial_codec_(nullptr)
    {
    }

    ~UartNode()
    {
        delete serial_codec_;
    }

    UartNode(const UartNode &)            = delete;
    UartNode &operator=(const UartNode &) = delete;
    UartNode(UartNode &&)                 = delete;
    UartNode &operator=(UartNode &&)      = delete;
    void      run();
    init_imu init_imu_{false,0.0,0.0};

private:
    void sub_callback(hnurm_interfaces::msg::VisionSendData::SharedPtr msg);
    void decision_sub_callback(hnurm_interfaces::msg::VisionSendData::SharedPtr msg);
    void sub_twist_callback(geometry_msgs::msg::Twist::SharedPtr msg);
    void timer_callback();
    void re_launch();
    void back_target_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void special_area_callback(const std_msgs::msg::Bool::SharedPtr msg);


private:
    rclcpp::CallbackGroup::SharedPtr callback_group1_;
    rclcpp::CallbackGroup::SharedPtr callback_group2_;

    rclcpp::Subscription<hnurm_interfaces::msg::VisionSendData>::SharedPtr sub_;
    rclcpp::Subscription<hnurm_interfaces::msg::VisionSendData>::SharedPtr decision_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr             sub_twist_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr back_target_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr in_special_area_sub_;

    rclcpp::Publisher<hnurm_interfaces::msg::VisionRecvData>::SharedPtr master_pub_;
    rclcpp::Publisher<hnurm_interfaces::msg::VisionRecvData>::SharedPtr slave_pub;

    std::thread uart_thread_;

    // tf broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Logger               logger;
    SerialCodec                 *serial_codec_;

    // topics
    std::string recv_topic_;
    std::string send_topic_;
    bool        use_control_id_ = false;

    bool        use_distribution_ = false;
    std::string master_ns_        = "left";
    std::string slave_ns_         = "right";
    std::string decision_send_topic_ = "/decision/vision_send_data";
    std::string back_target_ = "/back_target";
    bool is_in_special_area = false;

    std::mutex decision_cache_mutex_;
    uint8_t cached_gesture_ = 0;
    bool has_cached_gesture_ = false;
    

    float control_id_ = 0;

    bool stop_flag_ = false;
    int  error_cnt_ = 0;

protected:
    UartNodePtr shared_from_this()
    {
        return std::static_pointer_cast<UartNode>(rclcpp::Node::shared_from_this());
    }
};
}
