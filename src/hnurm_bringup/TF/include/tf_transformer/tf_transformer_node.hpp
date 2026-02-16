#pragma once

#include "nav_msgs/msg/odometry.hpp"
#include "hnurm_interfaces/msg/vision_recv_data.hpp"
#include "std_msgs/msg/float32.hpp"

#include <vector>
#include <tf2/LinearMath/Quaternion.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/vector3.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/static_transform_broadcaster.h"

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


#include "sensor_msgs/msg/point_cloud2.hpp"

namespace hnurm
{
    
class TfTransformer : public rclcpp::Node
{
public:
    explicit TfTransformer(const rclcpp::NodeOptions &options);  // explict：防止隐式转换

    ~TfTransformer()
    {
        RCLCPP_INFO(get_logger(), "TfTransformer destroyed");
    }

    TfTransformer(const TfTransformer &)            = delete; // 禁用拷贝构造函数
    TfTransformer &operator=(const TfTransformer &) = delete; // 禁用赋值运算符
    TfTransformer(TfTransformer &&)                 = delete; // 禁用移动构造函数
    TfTransformer &operator=(TfTransformer &&)      = delete; // 禁用移动赋值运算符
    // 传感器外参，包括平移(x,y,z)和旋转(roll,pitch,yaw)，类似变换矩阵
    struct extrinsic
    {
        float x,y,z,roll,pitch,yaw;
    };
    // 当前机器人的姿态角(roll,pitch,yaw)
    struct current_rpy
    {
        float roll,pitch,yaw;
    };
    // base_footprint到lidar
    extrinsic lidar_to_basefootprint_, joint_to_basefootprint_, gunpoint_to_joint_;
    
    // 节点会根据情况选择使用哪个位姿来设置 map 和 odom 坐标系之间的变换：
    bool is_initial_pose_get_ = false;

    int pointcloud_registrated_get_count_ = 0; // 拿到重定位待配准点云的次数

    bool is_regis_cloud_ready_ = false; // 重定位待配准点云是不是达到次数阈值

    bool is_static_finished_ = false; // 静态变换是否完成

    bool is_odom_ready_ = false; // 里程计是否完成变换

    bool is_relocation_ready_ = false; // 重定位是否完成变换
    // 目标点三维坐标，相对map系
    double target_x_, target_y_, target_z_;

    // 是否实时补偿pitch角度
    bool if_dynamic_pitch_ = true;

    std::string recv_topic_ = "recv"; // 接收视觉识别数据的话题名称

    std::string current_pointcloud_topic_ = "current_pointcloud"; // 当前点云话题名称

protected:
    std::shared_ptr<TfTransformer> shared_from_this()
    {
        return std::static_pointer_cast<TfTransformer>(std::shared_ptr<rclcpp::Node>(this));
    }
private:
    /* ******************订阅者****************** */
    //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // 订阅机器人的里程计数据，用于获取机器人的位置和姿态信息。
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_registrated_sub_; // 订阅重定位待配准点云，用于获取odom坐标系

    rclcpp::Subscription<hnurm_interfaces::msg::VisionRecvData>::SharedPtr recv_data_sub_; // 订阅视觉识别数据，用于获取云台pitch转角

    /*******************发布者 ***********************/
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PointStamped>> gun2target_vector_pub_; // 发布枪到目标点的向量

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_tf_;
    rclcpp::TimerBase::SharedPtr timer_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_; // F2变换缓冲区，存储和管理所有已广播的坐标变换信息
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // 监听ROS话题上发布的变换消息并将其存储到tf_buffer_中
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // 广播动态变化的坐标变换到/tf话题，发布随时间变化的变换关系（如机器人运动时的位姿）
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_; // 广播静态（不随时间变化）的坐标变换到/tf_static话题，发布固定的变换关系（如传感器安装位置，优化带宽使用，因为静态变换只需发布一次

    std::string downsampled_pcd_file_; // 存储下采样后的点云文件路径，作为map系的初始
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_downsampled_;// 存储下采样后的点云，作为map系的初始pose
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_registrated_;

    // 初始位姿猜测
    geometry_msgs::msg::TransformStamped initial_pose_guess_;

    // 重定位点云（用于获取frame_id）
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_registration_;

    // 变换odom -> base_link
    //void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void init_and_static_transform();
 
    // 三个静态
    void timer_callback();

    // 串口订阅回调
    void recv_data_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg);

    // 给出变换/cloud_registrated -> odom，的cloud_registrated原来的frame_id
    void pointcloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    extrinsic getTransformParams(const std::string & ns);

    // 计算距离
    void calculate_vector();

    // 检查重定位和里程计是否准备好
    void check_relocation_and_odom_ready();
};
}