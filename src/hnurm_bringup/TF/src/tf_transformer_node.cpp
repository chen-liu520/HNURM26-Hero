#include "tf_transformer/tf_transformer_node.hpp"
// #include <cv_bridge/cv_bridge.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <cmath>

using namespace std::chrono_literals;

namespace hnurm
{
    TfTransformer::TfTransformer(const rclcpp::NodeOptions &options)
        : Node("TfTransformer", options),
          global_map_downsampled_(new pcl::PointCloud<pcl::PointXYZ>), // 添加初始化
          pointcloud_registrated_(new pcl::PointCloud<pcl::PointXYZ>)  // 添加初始化
    {
        // Initialize the node
        RCLCPP_INFO(get_logger(), "TfTransformer is running");
        std::vector<std::string> frame_names = {
            "lidar_to_base", "joint_to_base", "gunpoint_to_joint"
        };
        // 存储变换外参

        /***************参数读取 start******************/
        std::map<std::string, extrinsic> transforms;

        // Get the transform parameters for each frame
        for (const auto &frame : frame_names)
        {
            transforms[frame] = getTransformParams(frame);
        }

        lidar_to_basefootprint_ = transforms["lidar_to_base"];
        joint_to_basefootprint_ = transforms["joint_to_base"];
        gunpoint_to_joint_ = transforms["gunpoint_to_joint"];

        // 读取下采样后的点云文件路径
        this->get_parameter("downsampled_pcd_file", downsampled_pcd_file_);

        // 目标点参数
        this->get_parameter("target_x", target_x_);
        this->get_parameter("target_y", target_y_);
        this->get_parameter("target_z", target_z_);

        this->get_parameter("if_dynamic_pitch", if_dynamic_pitch_);

        this->get_parameter("recv_topic", recv_topic_);

        RCLCPP_INFO(get_logger(), "Target point (map): %.2f, %.2f, %.2f", target_x_, target_y_, target_z_);

        RCLCPP_ERROR(get_logger(), "joint_to_base: x=%.3f, y=%.3f, z=%.3f",
                    joint_to_basefootprint_.x, joint_to_basefootprint_.y, joint_to_basefootprint_.z);
        RCLCPP_ERROR(get_logger(), "gunpoint_to_joint: x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f",
                    gunpoint_to_joint_.x, gunpoint_to_joint_.y, gunpoint_to_joint_.z,
                    gunpoint_to_joint_.roll, gunpoint_to_joint_.pitch, gunpoint_to_joint_.yaw);
        RCLCPP_ERROR(get_logger(), "recv_topic: %s", recv_topic_.c_str());

        /***************参数读取 end******************/

        // Initialize the TF2 buffer and listeners
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        recv_data_sub_ = this->create_subscription<hnurm_interfaces::msg::VisionRecvData>(
            recv_topic_, 
            rclcpp::SensorDataQoS(), std::bind(&TfTransformer::recv_data_callback, 
            this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init",
            rclcpp::SensorDataQoS(),
            std::bind(&TfTransformer::odom_callback, this, std::placeholders::_1));

        // 订阅实时点云
        pointcloud_registrated_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", rclcpp::SensorDataQoS(), std::bind(&TfTransformer::pointcloud_sub_callback, this, std::placeholders::_1));

        gun2target_vector_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/gun2target_vector", 10);

        timer_tf_ = this->create_wall_timer(110ms, std::bind(&TfTransformer::timer_callback, this));
    }

    void TfTransformer::init_and_static_transform()
    {
        geometry_msgs::msg::TransformStamped static_transform;

        // 定义一个四元数对象，用于存储旋转信息
        tf2::Quaternion q;

        // 3. 静态变换  odom->camera_init
        static_transform.header.frame_id = "odom"; // 使用默认值
        static_transform.child_frame_id = "camera_init";                                    
        static_transform.transform.translation.x = 0.0;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;
        static_transform.transform.rotation.x = 0.0;
        static_transform.transform.rotation.y = 0.0;
        static_transform.transform.rotation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;
        static_broadcaster_->sendTransform(static_transform);

        // 4. 静态变换  base_link -> basefootprint：外参传入
        static_transform.header.frame_id = "base_link";     // 父坐标系
        static_transform.child_frame_id = "base_footprint"; // 子坐标系
        static_transform.transform.translation.x = -lidar_to_basefootprint_.x;
        static_transform.transform.translation.y = -lidar_to_basefootprint_.y;
        static_transform.transform.translation.z = -lidar_to_basefootprint_.z;
        q.setRPY(0.0, 0.0, 0.0); // 旋转角度（单位：弧度）
        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y();
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();
        static_broadcaster_->sendTransform(static_transform);

        // 5. 静态变换  joint_link -> gunpoint：外参传入
        static_transform.header.frame_id = "joint_link";   // 父坐标系
        static_transform.child_frame_id = "gunpoint_link"; // 子坐标系
        static_transform.transform.translation.x = gunpoint_to_joint_.x;
        static_transform.transform.translation.y = gunpoint_to_joint_.y;
        static_transform.transform.translation.z = gunpoint_to_joint_.z;
        q.setRPY(gunpoint_to_joint_.roll, gunpoint_to_joint_.pitch, gunpoint_to_joint_.yaw); // 在这里进行z轴的-90度旋转
        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y();
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();
        static_broadcaster_->sendTransform(static_transform);

        is_static_finished_ = true;
    }

    // 传入namespace，把参数赋值给结构体变量，代表转换类型
    TfTransformer::extrinsic TfTransformer::getTransformParams(const std::string &ns)
    {
        extrinsic t;
        // get_parameter 函数用于获取当前节点的参数
        this->get_parameter(ns + ".x", t.x);
        this->get_parameter(ns + ".y", t.y);
        this->get_parameter(ns + ".z", t.z);
        this->get_parameter(ns + ".roll", t.roll);
        this->get_parameter(ns + ".pitch", t.pitch);
        this->get_parameter(ns + ".yaw", t.yaw);
        return t;
    }

    void TfTransformer::timer_callback()
    {
        /*tf tree
        down_cloud  camera_init-->aft_mapped                                extrinsic
            |           |                                                       |
            |           |                                                       |
           map  -->   odom  -->  base_link  -->  base_footprint --> joint_link --> gunpoint_link --> gunpoint_horizontal
                 |           |               |                   |              |                 |       
                 |           |               |                   |              |                 |
             relocation   odometry         static             recv:pitch      static           recv:pitch
        */
        /********************************************************************/
        /*******************************静态变换*******************************/
        /********************************************************************/
        if (!is_static_finished_ && is_regis_cloud_ready_)
        {
            init_and_static_transform();
        }

        check_relocation_and_odom_ready();

        calculate_vector();
    }

    void TfTransformer::pointcloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        
        if (pointcloud_registrated_get_count_ < 2 && !is_regis_cloud_ready_)
        {
            pointcloud_registrated_get_count_++;
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pointcloud_registrated_);
        is_regis_cloud_ready_ = true;
    }

    void TfTransformer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = "odom";     // 父坐标系（原来是 camera_init）
        transform.child_frame_id = "base_link"; // 子坐标系（原来是 aft_mapped）

        // 复制位置
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;

        // 复制姿态
        transform.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform);

        /*
        RCLCPP_INFO(this->get_logger(),
                    "发布 TF: odom -> base_link, 位置: %.2f, %.2f, %.2f",
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z);*/

        is_odom_ready_ = true;
    }

    void TfTransformer::recv_data_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg)
    {

        current_rpy rpy;
        rpy.roll = msg->roll;
        rpy.pitch = msg->pitch;
        rpy.yaw = msg->yaw;

        // 3. 创建 TransformStamped 消息
        geometry_msgs::msg::TransformStamped transformStamped;
        tf2::Quaternion q;

        auto now = this->get_clock()->now();
        // 4. 填充消息头
        transformStamped.header.stamp = now;
        transformStamped.header.frame_id = "base_footprint"; // 父坐标系（原来是 camera_init）
        transformStamped.child_frame_id = "joint_link";      // 子坐标系（原来是 aft_mapped）

        // 5. 复制位置（Odometry 的 pose 就是子坐标系在父坐标系中的位姿）
        transformStamped.transform.translation.x = joint_to_basefootprint_.x;
        transformStamped.transform.translation.y = joint_to_basefootprint_.y;
        transformStamped.transform.translation.z = joint_to_basefootprint_.z;

        // 6. 复制姿态（四元数）
        if (if_dynamic_pitch_)
            q.setRPY(0, -rpy.pitch * M_PI / 180.0, 0);
        else
            q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        // 7. 发布 TF
        tf_broadcaster_->sendTransform(transformStamped);

        // 8. 发布 TF: gunpoinit_link --> gunpoint_horizontal
        transformStamped.header.frame_id = "gunpoint_link";      // 父坐标系（原来是 camera_init）
        transformStamped.child_frame_id = "gunpoint_horizontal"; // 子坐标系（原来是 aft_mapped）
        try
        {
            geometry_msgs::msg::TransformStamped tf_gunpoint = tf_buffer_->lookupTransform(
                "base_footprint",           // target_frame
                "gunpoint_link",            // source_frame
                tf2::TimePointZero,         // 最新时间
                tf2::durationFromSec(0.1)); // 100ms超时

            // 发布 gunpoint_horizontal：同位置，但方向水平
            transformStamped.header.stamp = now;
            transformStamped.header.frame_id = "base_footprint";
            transformStamped.child_frame_id = "gunpoint_horizontal";
            // 位置从 lookup 结果获取
            transformStamped.transform.translation.x = tf_gunpoint.transform.translation.x;
            transformStamped.transform.translation.y = tf_gunpoint.transform.translation.y;
            transformStamped.transform.translation.z = tf_gunpoint.transform.translation.z;
            // 方向：水平（只有yaw，无pitch/roll）
            q.setRPY(0, 0, gunpoint_to_joint_.yaw);
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(transformStamped);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to lookup gunpoint_link: %s", ex.what());
        }
    }
    
    void TfTransformer::calculate_vector()
    {
        if (!is_static_finished_ || !is_odom_ready_ || !is_relocation_ready_)
        {
            return;
        }

        try
        {
            // 获取 gunpoint_link 相对于 map 的变换
            geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
                "gunpoint_horizontal",
                "map",
                tf2::TimePointZero,
                tf2::durationFromSec(0.1));

            /*
            // 目标点在 map 中的坐标
            geometry_msgs::msg::PointStamped target_map;
            target_map.header.stamp = tf.header.stamp; // 使用TF的时间戳保持一致
            target_map.header.frame_id = "map";
            target_map.point.x = target_x_;
            target_map.point.y = target_y_;
            target_map.point.z = target_z_;

            // 目标点在 gunpoint_link 中的坐标
            geometry_msgs::msg::PointStamped target_gunpoint;
            target_gunpoint.header.stamp = tf.header.stamp;
            target_gunpoint.header.frame_id = "gunpoint_horizontal";

            tf_buffer_->transform(
                target_map,
                target_gunpoint,
                "gunpoint_horizontal");
            */
            // 构建目标点在 map 中的坐标
            tf2::Vector3 target_in_map(target_x_, target_y_, target_z_);

            // 获取旋转和平移
            tf2::Transform transform;
            tf2::fromMsg(tf.transform, transform);

            // 变换到 gunpoint_horizontal 坐标系
            tf2::Vector3 target_in_gunpoint = transform * target_in_map;

            // 发布结果
            geometry_msgs::msg::PointStamped target_gunpoint;
            target_gunpoint.header.stamp = tf.header.stamp;
            target_gunpoint.header.frame_id = "gunpoint_horizontal";
            target_gunpoint.point.x = target_in_gunpoint.x();
            target_gunpoint.point.y = target_in_gunpoint.y();
            target_gunpoint.point.z = target_in_gunpoint.z();

            RCLCPP_INFO(this->get_logger(),
                        "\033[1;34mTarget in gunpoint_horizontal: x=%.3f, y=%.3f, z=%.3f\033[0m",
                        target_in_gunpoint.x(), target_in_gunpoint.y(), target_in_gunpoint.z());

            // 发布向 gunpoint_link 指向目标点的向量
            gun2target_vector_pub_->publish(target_gunpoint);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Could not get transform from 【map to gunpoint_horizontal】: %s", ex.what());
        }
    }

    void TfTransformer::check_relocation_and_odom_ready()
    {
        if (!is_relocation_ready_)
        {
            try
            {
                geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
                    "map",
                    "odom",
                    tf2::TimePointZero // 最新可用变换
                );
                is_relocation_ready_ = true;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Could not get transform from 【map to odom】: %s", ex.what());
            }
        }
        if (!is_odom_ready_)
        {
            try
            {
                geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
                    "odom",
                    "base_link",
                    tf2::TimePointZero // 最新可用变换
                );
                is_odom_ready_ = true;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Could not get transform from 【odom to base_link】: %s", ex.what());
            }
        }
    }

} // namespace hnurm

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<hnurm::TfTransformer>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();
}