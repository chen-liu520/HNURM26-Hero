#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <iostream>
#include <vector>
#include <deque>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>

// eigen libs
#include <Eigen/Geometry>
#include <Eigen/Core>

// small_gicp libs
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/benchmark/read_points.hpp>

// pcl libs
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

// teaserpp libs
#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <quatro/quatro_module.h>

#include <mutex>
#include <atomic>
#include <future>

namespace hnurm
{

    using QuatroPointType = pcl::PointXYZ;

    class RelocationNode : public rclcpp::Node
    {
    public:
        explicit RelocationNode(const rclcpp::NodeOptions &options);
        ~RelocationNode();
        RelocationNode(const RelocationNode &) = delete;
        RelocationNode &operator=(const RelocationNode &) = delete;
        RelocationNode(RelocationNode &&) = delete;

        RelocationNode &operator=(RelocationNode &&) = delete;

        bool is_yaw_ready_ = false;

    private:
        /***********************回调和工具函数声明 start************************/
        void pointcloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        // void recv_sub_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg);
        void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
        void load_pcd_map(const std::string &map_path);
        void set_gicp_handler();
        void update_deque_when_registration_thread_running(sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void timer_callback();
        void high_frequency_timer_callback();

        void trigger_hero_callback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        void yaw_ready_callback(const std_msgs::msg::Bool::SharedPtr msg);

        void small_gicp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr &current_sum_cloud_, std::string gicp_type);
        void reset();

        void accumulate_cloud_then_QUAandGICP(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int count_num);
        void accumulate_cloud_then_QUAandGICP_with_debug(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int count_num);

        void QUA_GICP_init_and_reset(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> protecting_vector, int count_num, int &gicp_counter);
        void QUA_GICP_init_and_reset_with_debug(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> protecting_vector, int count_num, int &gicp_counter);

        void GICP_tracking(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        /**********************回调和工具函数声明 end************************/

        // 【注释命名规范:建图得到的点云：全局点云 + 降采样/协方差】
        // 【注释命名规范:当前帧点云：当前帧点云 + 降采样/协方差】

        /***********************订阅者，发布者，定时器声明 start************************/
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_; // 订阅rviz发布的初始位姿
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr yaw_ready_sub_;                           // 订阅yaw准备好的信号

        // 服务端，接受请求，触发HERO模式
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hero_trigger_service_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_registered_pub_; // 发布配准后的点云
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;            // 发布降采样的全局点云
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;                        // 发布状态
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr relocation_ready_pub_;                // 发布重定位准备好信号，告诉TF可以计算向量
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_lidar_pub_;             // 发布原始点云，来自/livox/lidar
        rclcpp::TimerBase::SharedPtr timer_;                                                    // 发布原始点云定时器
        rclcpp::TimerBase::SharedPtr high_frequency_timer_;                                     // 高频定时器，用于发布tf和状态
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        /***********************订阅者，发布者，定时器声明 end************************/

        /*********************************成员变量 start*******************************/
        /*********1. 点云相关 ****************/
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;                        // 全局原始点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_downsampled_;            // 全局降采样点云
        pcl::PointCloud<pcl::PointCovariance>::Ptr global_map_PointCovariance_; // 全局点云协方差/全局带协方差的降采样点云

        pcl::PointCloud<pcl::PointXYZ>::Ptr current_accumulated_cloud_; // 原：source_cloud_ 当前积累点云（未降采样）
        // pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;             // 原：accumulte_cloud_ 当前帧点云（未降采样）
        // pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_downsampled_; // 当前帧点云（降采样）
        pcl::PointCloud<pcl::PointXYZ>::Ptr summary_downsampled_cloud_; // 积累的降采样点云

        pcl::PointCloud<pcl::PointCovariance>::Ptr source_cloud_PointCovariance_; // gicp配准原

        /*********2. 位姿相关 ***************/

        Eigen::Isometry3d initial_guess_ = Eigen::Isometry3d::Identity(); // 来自quatro++或者initial_pose_callback

        geometry_msgs::msg::TransformStamped transform; // map -> odom变换存放，标准类型的transform，用于发布tf

        /*************3. kd树相关（用于gicp） ***************/
        std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;

        sensor_msgs::msg::PointCloud2::SharedPtr cloud_; // 发布的降采样全局点云，发布者：pointcloud_pub_

        bool getInitialPose_ = false; // 有没有获得初始位姿

        Eigen::Isometry3d pre_result_ = Eigen::Isometry3d::Identity(); // 可能不会用，前一次的gicp配准结果，初始化为单位矩阵
        small_gicp::RegistrationResult result;                         // gicp配准结果

        /*************4. 算法参数相关*************/

        // small_gicp参数
        std::string pointcloud_sub_topic_;
        std::string yaw_ready_topic_;
        std::string trigger_hero_service_name_;
        std::string pcd_file_;
        std::string downsampled_pcd_file_;

        /*****************普通模式 gicp参数 start***************/
        int gicp_num_threads_;
        int gicp_num_neighbors_;
        float gicp_max_dist_sq_;
        float gicp_max_iterations_;                    // 最大迭代次数（默认通常是30 - 50）##收敛阈值
        float gicp_convergence_translation_tolerance_; // 平移收敛阈值（单位：m）
        float gicp_convergence_rotation_tolerance_;    // 旋转收敛阈值（单位：rad）

        float gicp_voxel_size_;
        float quatro_voxel_size_;
        /*****************普通模式 gicp参数 end***************/

        int gicp_failed_counter_ = 0;
        int tracking_frequency_divisor_; // 跟踪阶段每收到几帧进行一次gicp，用于控制gicp频率，控制资源消耗
        int gicp_run_counter_ = 0;       // gicp运行次数，配合上面的阈值，用于控制gicp频率
        int hero_gicp_counter_ = 0;      // 单独英雄部署模式gicp运行次数，达到阈值切换为NONE状态
        int hero_gicp_running_counter_threshold_; // 英雄部署模式gicp运行次数阈值

        /*****************英雄模式 gicp参数 start***************/
        int gicp2_num_threads_;
        int gicp2_num_neighbors_;
        float gicp2_max_dist_sq_;
        float gicp2_max_iterations_;                    // 最大迭代次数（默认通常是30 - 50）##收敛阈值
        float gicp2_convergence_translation_tolerance_; // 平移收敛阈值（单位：m）
        float gicp2_convergence_rotation_tolerance_;    // 旋转收敛阈值（单位：rad）

        float gicp2_voxel_size_;
        float quatro2_voxel_size_;
        /*****************英雄模式 gicp参数 end***************/

        float map_voxel_size_;

        // quatro++参数
        int m_rotation_max_iter_ = 100,
            m_num_max_corres_ = 200;
        double m_normal_radius_ = 0.02, m_fpfh_radius_ = 0.04, m_distance_threshold_ = 30.0;
        double m_noise_bound_ = 0.25, m_rotation_gnc_factor_ = 1.39, m_rotation_cost_thr_ = 0.0001;
        bool m_estimate_scale_ = false, m_use_optimized_matching_ = true;

        /*************5. 状态与关键参数 **************/
        bool test_cloud_registered_ = false; // 是否测试点云频率
        bool use_rviz_revise_ = false;       // 是否使用rviz给定初始位姿
        bool generate_downsampled_pcd_ = false;
        bool use_hero_individual_ = true; // 是否使用英雄部署模式
        

        enum class State
        {
            INIT,     // 初始积累阶段（5秒内）
            TRACKING, // 正常运行跟踪
            RESET,    // 重置状态
            HERO,     // 英雄部署模式
            NONE      // 无状态，这个阶段不进行任何操作
        };

        std::atomic<State> state_{State::INIT}; // 初始状态为初始积累阶段，原子类型，线程安全

        std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> track_slide_window_clouds_queue; // 跟踪阶段滑动窗口点云队列
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> init_current_clouds_vector;     // 初始位姿阶段点云向量

        int track_accumulation_counter_;     // 跟踪阶段点云积累次数,也是初次配准gicp取的帧数
        int init_accumulation_counter_;      //  初始位姿阶段点云积累次数
        int reset_accumulation_counter_;     // 重置阶段点云积累次数
        int hero_qua_accumulation_counter_;  // 英雄部署模式quatro积累点云数量
        int hero_gicp_accumulation_counter_; // 英雄部署模式gicp积累点云数量

        // 频率计算相关变量

        std::chrono::steady_clock::time_point last_callback_time_; // 上一次回调的时间
        bool is_first_callback_ = true;
        bool is_queue_full_ = false;

        /**************6. 多线程和锁*******************************/

        std::future<void> quatro_future_;
        std::atomic<bool> is_QUAandGICP_running_{false};
        std::mutex accumulation_mutex_; // 应该是 mutex

        /***************7. QUATRO++和GICP求解器************************ */

        small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration_;

        small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> super_registration_;

        std::shared_ptr<quatro<QuatroPointType>> m_quatro_handler = nullptr;

        /*********************************成员变量 end*******************************/

    protected:
        std::shared_ptr<RelocationNode> shared_from_this()
        {
            return std::static_pointer_cast<RelocationNode>(rclcpp::Node::shared_from_this());
        }
    };

}