#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "hnurm_interfaces/msg/vision_recv_data.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <atomic>

using namespace std::chrono_literals;

// 非阻塞键盘输入设置
void setNonBlockingInput(bool enable)
{
    static struct termios oldt, newt;
    if (enable)
    {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }
    else
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
}

class TfTestNode : public rclcpp::Node
{
public:
    TfTestNode() : Node("tf_test_node")
    {
        RCLCPP_INFO(get_logger(), "TF Test Node started");
        RCLCPP_INFO(get_logger(), "Controls:");
        RCLCPP_INFO(get_logger(), "  SPACE = pitch -10°");
        RCLCPP_INFO(get_logger(), "  ENTER = pitch +10°");
        RCLCPP_INFO(get_logger(), "  Q     = yaw -10° (左转)");
        RCLCPP_INFO(get_logger(), "  E     = yaw +10° (右转)");

        // 发布者
        vision_recv_pub_ = this->create_publisher<hnurm_interfaces::msg::VisionRecvData>("vision_recv_data", 10);
        //odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("current_pointcloud", 10);

        // TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // 发布静态TF: map -> odom (固定平移，用于区分)
        

        // 定时器：发布odom、VisionRecvData和点云
        timer_ = this->create_wall_timer(100ms, std::bind(&TfTestNode::timerCallback, this));

        // 启动键盘监听线程
        setNonBlockingInput(true);
        keyboard_thread_ = std::thread(&TfTestNode::keyboardLoop, this);
    }

    ~TfTestNode()
    {
        running_ = false;
        if (keyboard_thread_.joinable())
            keyboard_thread_.join();
        setNonBlockingInput(false);
    }

private:
    void publishMapToBaseLink()
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->get_clock()->now();
        tf.header.frame_id = "map";
        tf.child_frame_id = "base_link";
        
        // 固定平移
        tf.transform.translation.x = 1.0;  
        tf.transform.translation.y = 2.0;  
        tf.transform.translation.z = 0.0;
        
        // 应用 yaw (绕Z轴) 和 pitch (绕Y轴)
        double yaw_rad = current_yaw_.load() * M_PI / 180.0;
        
        tf2::Quaternion q_yaw;
        q_yaw.setRPY(0, 0, yaw_rad);       // yaw: 绕Z轴
        
        
        tf.transform.rotation.x = q_yaw.x();
        tf.transform.rotation.y = q_yaw.y();
        tf.transform.rotation.z = q_yaw.z();
        tf.transform.rotation.w = q_yaw.w();
        
        tf_broadcaster_->sendTransform(tf);
    }

    void publishEmptyPointCloud()
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->get_clock()->now();
        
        // 设置点云的基本字段
        cloud_msg.height = 1;
        cloud_msg.width = 0;  // 空点云，没有点
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;
        
        // 添加点字段 (x, y, z)
        sensor_msgs::msg::PointField field;
        field.name = "x";
        field.offset = 0;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        cloud_msg.fields.push_back(field);
        
        field.name = "y";
        field.offset = 4;
        cloud_msg.fields.push_back(field);
        
        field.name = "z";
        field.offset = 8;
        cloud_msg.fields.push_back(field);
        
        cloud_msg.point_step = 12;  // 3 * float32
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
        cloud_msg.data.resize(0);   // 空数据
        cloud_msg.header.frame_id = "aft_registered";

        pointcloud_pub_->publish(cloud_msg);
    }

    void timerCallback()
    {
        auto now = this->get_clock()->now();
        // 0. 发布动态TF: map -> base_link (应用yaw和pitch)
        publishMapToBaseLink();
        // 1. 发布空点云，用于触发TF节点的静态变换初始化
        publishEmptyPointCloud();

        // 2. 发布 VisionRecvData (替代原来的接口)
        hnurm_interfaces::msg::VisionRecvData recv_msg;
        recv_msg.header.stamp = now;
        recv_msg.header.frame_id = "test_frame";
        
        // 设置枚举值（使用默认值）
        recv_msg.self_color.data = 0;      // COLOR_NONE
        recv_msg.work_mode.data = 0;       // AUTO_SHOOT
        recv_msg.bullet_speed.data = 0;    // BULLE_SPEED_NONE
        recv_msg.gesture.data = 0;         // ATTACK
        
        // 设置RPY角度（pitch和yaw可变）
        recv_msg.roll = 0.0;
        recv_msg.pitch = current_pitch_;
        recv_msg.yaw = 0.0;
        
        // 其他字段设为0
        recv_msg.control_id = 0.0;
        recv_msg.game_progress = 0.0;
        recv_msg.current_hp = 0.0;
        recv_msg.current_base_hp = 0.0;
        recv_msg.allow_fire_amount = 0.0;
        recv_msg.current_outpost_hp = 0.0;
        recv_msg.current_enemy_base_hp = 0.0;
        recv_msg.current_enemy_outpost_hp = 0.0;
        recv_msg.hero_x = 0.0;
        recv_msg.hero_y = 0.0;
        recv_msg.cmd_x = 0.0;
        recv_msg.cmd_y = 0.0;
        
        vision_recv_pub_->publish(recv_msg);
    
    }

    void keyboardLoop()
    {
        while (running_ && rclcpp::ok())
        {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0)
            {
                if (c == ' ')  // 空格：pitch减小10度
                {
                    double new_pitch = current_pitch_.load() - 10.0;
                    current_pitch_.store(new_pitch);
                    RCLCPP_INFO(get_logger(), "SPACE pressed: pitch = %.1f°", new_pitch);
                }
                else if (c == '\n')  // 回车：pitch增加10度
                {
                    double new_pitch = current_pitch_.load() + 10.0;
                    current_pitch_.store(new_pitch);
                    RCLCPP_INFO(get_logger(), "ENTER pressed: pitch = %.1f°", new_pitch);
                }
                else if (c == 'q' || c == 'Q')  // Q：yaw减小10度（左转）
                {
                    double new_yaw = current_yaw_.load() - 10.0;
                    current_yaw_.store(new_yaw);
                    RCLCPP_INFO(get_logger(), "Q pressed: yaw = %.1f° (left)", new_yaw);
                }
                else if (c == 'e' || c == 'E')  // E：yaw增加10度（右转）
                {
                    double new_yaw = current_yaw_.load() + 10.0;
                    current_yaw_.store(new_yaw);
                    RCLCPP_INFO(get_logger(), "E pressed: yaw = %.1f° (right)", new_yaw);
                }
            }
            std::this_thread::sleep_for(50ms);
        }
    }

    // 成员变量
    rclcpp::Publisher<hnurm_interfaces::msg::VisionRecvData>::SharedPtr vision_recv_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::atomic<bool> running_{true};
    std::thread keyboard_thread_;
    std::atomic<double> current_pitch_{0.0};
    std::atomic<double> current_yaw_{0.0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
