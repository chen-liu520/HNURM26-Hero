# registration — Quatro++配合small_gicp点云重定位节点

## 一、功能概述和项目解释
- 订阅激光点云（`pointcloud_sub_topic`，默认 `/cloud_registered`），将当前帧点云与全局地图进行配准，估计当前位姿。
    - **为什么用这个话题？[详情见这里，非常重要，一定要看](#点云话题订阅)**
- 为什么使用quatro++和small_gicp两个配准算法？
    - 因为small_gicp对初始位姿猜测敏感，他需要一个相对精确的初始位姿猜测才能收敛，结果比较精准；而quatro++是一个全局配准算法，他不需要初始位姿猜测，结果比较粗糙
    - 因此，在第一次配准的时候，我们选择先用quatro++输出一个初始位姿，随后使用这个初始位姿进行small_gicp配准
    - 在第一次配准之后的所有阶段，我们每次使用small_gicp上一次的配准结果作为初始位姿猜测，进行small_gicp配准
        - 这里的要求就是，small_gicp配准频率必须快一点，避免机器人快读移动导致之前的位姿超出gicp的收敛域
        - 因此，我们设计了用点云回调函数作为配准的触发条件，每次收到新的点云时，跟踪阶段就进行一次small_gicp配准，这个1点云回调频率大致是10Hz
    - 当gicp配准失败后，我们会用quatro++重新进行一次全局配准，得到一个新的初始位姿猜测，随后用这个新的初始位姿猜测进行small_gicp配准
- 基于以上设计，我们使用一个状态机控制配准流程，状态机包括以下状态：
    - INIT：初始位姿配准阶段，quatro++进行全局配准得到初始猜测，传给small_gicp进行精细配准
    - TRACKING：跟踪阶段：连续使用small_gicp算法
    - RESET：当small_gicp配准失败后，会进入重置状态，用quatro++进行全局配准，得到一个新的初始位姿猜测，随后用这个新的初始位姿猜测进行small_gicp配准
- 支持从离线 PCD 地图加载（`pcd_file`），可按需自动生成/缓存降采样地图（`generate_downsampled_pcd` 与 `downsampled_pcd_file`）。
- mid360雷达单帧大概2万点，一般不太足够点云配准精度需求，因此不论quatro++还是small_gicp都需要收集多帧数据，所以**项目参数可以手动更改**：初始配准quatro++算法、gicp失败后的重置状态配准quatro++算法、small_gicp配准这三个部分的积累帧数。
    - 如果发现实时性不足，可以降低跟踪阶段积累次数，比如设置为2，就是每次配准只保留最新的2帧点云进行配准，再小就是1,每次只配准最新的一帧点云
- 上面提到：使用点云回调作为配准触发条件，但是gicp资源消耗可能比较大，10Hz也就是0.1s一次可能对低性能系统来说有点吃不消
    - 因此设计参数：`tracking_frequency_divisor`:跟踪阶段每收到几帧进行一次gicp，用于控制gicp频率，控制资源消耗，注意要大于等于1
- 参数：`test_cloud_registered_`用于测试点云发布频率
- 发布配准状态，可以用于决策判断，因为RESET应该让车急停的
- 参数`use_rviz_revise`：不使用quatro进行初始位姿猜测，直接用rviz中发布的位姿作为初始位姿猜测，后面持续运行gicp，reset再quatro
- small_gicp参数：
    - **num_threads**: 线程数，用于并行计算（降采样、协方差计算、GICP配准）
    - **num_neighbors**: 协方差估计时的近邻点数，影响法向量计算精度
    - **max_dist_sq**: 最大对应点距离平方（单位：m²），用于过滤离群点
    - **gicp_max_iterations**: GICP最大迭代次数，控制收敛上限
    - **gicp_convergence_tolerance**: GICP收敛阈值，误差变化小于此值认为收敛
    - **gicp_voxel_size**: 当前帧点云降采样体素大小（米），平衡精度与速度
    - **map_voxel_size**: 全局地图降采样体素大小（米）
    - **tracking_frequency_divisor**: 跟踪阶段GICP降频因子，每N帧执行一次配准
- quatro++参数：
    - **m_rotation_max_iter**: 旋转估计最大迭代次数
    - **m_num_max_corres**: 最大对应点数，用于FPFH特征匹配
    - **m_normal_radius**: 法向量估计半径（米）
    - **m_fpfh_radius**: FPFH特征计算半径（米）
    - **m_distance_threshold**: 对应点距离阈值（米），过滤错误匹配
    - **m_noise_bound**: 噪声边界（米），假设的点云噪声水平
    - **m_rotation_gnc_factor**: GNC（Graduated Non-Convexity）因子，控制旋转估计鲁棒性
    - **m_rotation_cost_thr**: 旋转估计收敛阈值
    - **m_estimate_scale**: 是否估计尺度（默认为false，适用于刚性配准）
    - **m_use_optimized_matching**: 是否使用优化的特征匹配算法
    - **quatro_voxel_size**: Quatro++降采样体素大小（米）




## 二、使用方法：
- rviz可视化启动+launch（可选）：
```bash
ros2 launch registration registration_rviz.launch.py
``` 
- 无rviz可视化的launch启动节点：
```bash
ros2 launch registration registration.launch.py
```
- rviz设置global options的fixed frame为==map,才能看到配准点云与原点云重和效果==


## 三、参数说明（可在 YAML 中设置）
### 详情见yaml文件注释


## 四、问题
1. 虽然已知/cloud_registered含义上就是odom，但是这里用的是点云坐标系，显示转化在哪里？



2. 多线程gicp配准
```cpp
// else
        // {
        //   std::vector<RegistrationResult> results;
        //   #pragma omp parallel for num_threads(4)
        //   for(const auto& guess_:guesses_)
        //   {
        //     result = registration.align(*global_map_PointCovariance_, *source_cloud_PointCovariance_, *target_tree_, guess_);
        //     RegistrationResult rs_;
        //     rs_.converged = result.converged;
        //     rs_.error = result.error;
        //     rs_.transformation = result.T_target_source;
        //     #pragma omp critical
        //     results.emplace_back(rs_);
        //   }
        //   auto best_result = std::min_element(results.begin(), results.end(),
        //   [](const auto& a, const auto& b) { return a.error < b.error; });
        //   pre_result_=best_result->transformation;
        //   // if(best_result->error>20.0&&!doFirstRegistration_)
        //   // {
        //   //   RCLCPP_INFO(get_logger(), "cannot do first registration,reset,result error:%f",best_result->error);
        //   //   reset();
        //   //   return;
        //   // }
        //   // else pre_result_=best_result->transformation;
        // }
```

## 五、问题解决（遇到不懂的可以找找我写没写）
1. /current_cloud_ 是成员变量，永远指向固定内存块，每次fromROSMsg()会在该内存块【固定的这块内存，地址永远不变】上写入新数据，导致旧数据被覆盖。
    - 也就是说，如果队列一直push_back(current_cloud_)，那么队列中所有成员都是同一个指针，【都】指向最新的点云数据【他们都相同】，实际上并没有累计点云。
    - **解决:每次接收到点云，新建智能指针的PCL对象存贮，这样内存里不同地址对应的是不同的点云对象，从而实现累计点云。**
    - 智能指针生命周期：不会随着当前作用域销毁而销毁，在本例子中：
        - 创建新对象：引用=1
        - push_back到队列/vector，引用计数 = 2（队列持有 + 局部变量持有）
        - 局部变量销毁，引用计数 = 1（队列仍持有）
        - 队列/vector销毁，或者pop_front()/pop_back()，引用计数 = 0，内存被释放
2. 点云累加：完全相同坐标的加法如何处理
    - 在PCL内部，加和不会判断重复，反正这里有一个点，就算有100个相同的点，在这个坐标位置就是100个点
    - **解决方法：降采样：解决相同的重复的，又简化计算**
    - 降采样：将点云降采样到一个固定的体素大小，这样相同坐标的点就会被合并，从而实现加和。
3. quatro时间太长，加入多线程
    - 防止quatro在主线程长时间运行阻塞，导致点云无法更新
4. 多线程引起线程安全问题
    - 使用is_QUAandGICP_running_隔绝主线程
    - 使用原子变量
5. 降采样和计算协方差是两个函数
6. 加入small_gicp两个新参数
7. 为什么有的点云变量使用hpp里面已经设定的，有的点云变量自己定义？
    - 因为hpp里面已经设定的点云变量是全局的，如果存在多线程，都使用公共的，就会有线程安全
    - 一开始我没考虑到多线程需求，后续再改，就出现了许多hpp里定义的变量没有使用的情况
    - 比如：
| 变量名 | 位置 | 说明 |
| --- | --- | --- |
| current_cloud_	 | hpp:115	| 声明但未在cpp中使用 |
| current_cloud_downsampled_	| hpp:116	| 声明但未在cpp中使用|
| use_timer_	| hpp:178	| 声明并读取参数，但逻辑未实现（pointcloud_sub_callback中if判断为空）|
| tf_listener_	| hpp:102	| 创建了但未调用任何lookupTransform | 
| tf_buffer_	| hpp:103	| 配合tf_listener_，同样未使用 |

## 七、函数解释
1. 构造函数
```cpp
RelocationNode()
├── 参数声明与读取 (declare_parameter)
│   ├── 点云相关: pointcloud_sub_topic_, pcd_file_, downsampled_pcd_file_
│   ├── 积累控制: init_accumulation_counter_, track_accumulation_counter_, reset_accumulation_counter_
│   ├── small_gicp参数: num_threads_, num_neighbors_, max_dist_sq_, gicp_max_iterations_, gicp_convergence_tolerance_
│   ├── 降采样参数: gicp_voxel_size_, quatro_voxel_size_, map_voxel_size_
│   └── 模式控制: test_cloud_registered_, use_rviz_revise_, use_timer_, tracking_frequency_divisor_
├── 初始化Quatro处理器 (m_quatro_handler)
│   └── 传入全部Quatro参数 (m_rotation_max_iter_, m_noise_bound_ 等)
├── 初始化TF工具
│   ├── tf_buffer_ ← get_clock()
│   ├── tf_listener_ ← tf_buffer_
│   └── tf_broadcaster_ ← *this
├── 初始化点云指针
│   ├── global_map_ (原始地图)
│   ├── global_map_downsampled_ (降采样地图)
│   ├── global_map_PointCovariance_ (带协方差的地图)
│   ├── current_accumulated_cloud_ (当前积累点云)
│   └── summary_downsampled_cloud_ (降采样后点云)
├── load_pcd_map(pcd_file_)  【关键调用】
│   ├── 分支A: generate_downsampled_pcd_ == true
│   │   ├── 加载原始点云 → global_map_
│   │   ├── 体素降采样 → global_map_downsampled_
│   │   └── 保存降采样PCD文件
│   └── 分支B: generate_downsampled_pcd_ == false (默认)
│       ├── 加载降采样点云 → global_map_downsampled_
│       ├── 计算协方差 → global_map_PointCovariance_
│       ├── 构建KD-Tree → target_tree_ 【供GICP使用】
│       └── 准备可视化点云 → cloud_
├── 创建ROS2通信实体
│   ├── pointcloud_sub_ → pointcloud_sub_callback()
│   ├── init_pose_sub_ → initial_pose_callback()
│   ├── pointcloud_pub_ → 发布全局地图
│   ├── pointcloud_registered_pub_ → 发布配准后点云
│   ├── status_pub_ → 发布状态
│   └── timer_ → timer_callback()
└── 预分配向量空间
    └── init_current_clouds_vector.resize(init_accumulation_counter_)
```

2. 点云回调
```cpp
pointcloud_sub_callback(msg)
├── 【调试模式检查】if (test_cloud_registered_)
│   └── 计算并打印点云发布间隔时间
│   └── 返回（不进行配准）
│
├── 【忙等待检查】if (is_QUAandGICP_running_.load())
│   └── 打印错误："正在初始化配准中，当前点云被丢弃"
│   └── 返回
│
└── 【正常处理流程】else (!is_QUAandGICP_running_)
    ├── 【分支1】state == INIT && !use_rviz_revise_
    │   └── accumulate_cloud_then_QUAandGICP_with_debug(msg, init_accumulation_counter_)
    │       └── 【异步】Quatro + GICP 初始化配准
    │
    ├── 【分支2】state == TRACKING
    │   └── GICP_tracking(msg)
    │       └── 【同步】滑动窗口 + GICP 跟踪配准
    │
    ├── 【分支3】state == RESET
    │   └── accumulate_cloud_then_QUAandGICP(msg, reset_accumulation_counter_)
    │       └── 【异步】Quatro + GICP 重置配准
    │
    └── 【分支4】else (即 use_rviz_revise_ == true)
        ├── if (!getInitialPose_)
        │   └── 打印："等待RViz给出初始位姿中..."
        │   └── 返回
        └── else (已获得初始位姿)
            └── GICP_tracking(msg)
                └── 【同步】GICP 跟踪配准（使用RViz给的初始位姿）
```

3.  Quatro+GICP初始化流程（INIT/RESET共用逻辑
```cpp
accumulate_cloud_then_QUAandGICP(msg, count_num)
├── 【点云积累阶段】
│   ├── 将msg转换为PCL点云 (new_cloud，新建智能指针)
│   ├── init_current_clouds_vector.push_back(new_cloud)
│   └── if (vector.size() < count_num) return;  // 积累不足，等待
├── 【触发Quatro配准】
│   ├── 累加所有点云: *summary_cloud += *cloud
│   ├── 体素降采样: summary_downsampled_cloud_ = voxelgrid_sampling_omp(...)
│   ├── 创建Quatro异步任务 (防止阻塞主线程)
│   │   └── std::async(std::launch::async, [&]() { ... })
│   └── 设置标志: is_QUAandGICP_running_ = true
│
└── 【异步线程：Quatro+GICP处理】
    ├── Quatro粗配准
    │   ├── m_quatro_handler->align(summary_downsampled_cloud_, global_map_downsampled_, if_valid_)
    │   └── 输出: output_tf_ (4x4矩阵)
    ├── 【结果处理分支】
    │   ├── 【成功分支】if (if_valid_)
    │   │   ├── 保存初始猜测: initial_guess_ = Eigen::Isometry3d(output_tf_)
    │   │   ├── 设置标志: getInitialPose_ = true
    │   │   ├── 取最近3帧点云准备精配准
    │   │   ├── 【调用GICP】relocalization(current_sum_cloud_for_gicp)
    │   │   │   └── 如成功，状态自动切换到TRACKING
    │   │   └── 清空积累向量: init_current_clouds_vector.clear()
    │   └── 【失败分支】
    │       ├── 打印警告: "Quatro 配准失败，清空积累"
    │       ├── 清空积累向量
    │       └── 重置标志: is_QUAandGICP_running_ = false
    └── 配准完成后: is_QUAandGICP_running_ = false
```
4. GICP跟踪流程（TRACKING状态）
```
relocalization(current_sum_cloud_)
├── 前置检查
│   └── if (!getInitialPose_) 返回错误
├── 【GICP配准配置】
│   ├── Registration<GICPFactor, ParallelReductionOMP> registration
│   ├── 设置线程数: registration.reduction.num_threads = num_threads_
│   ├── 设置最大对应点距离: registration.rejector.max_dist_sq = max_dist_sq_
│   ├── 设置最大迭代次数: registration.optimizer.max_iterations = gicp_max_iterations_
│   └── 设置收敛阈值: registration.criteria.translation_eps = gicp_convergence_tolerance_
├── 【源点云预处理】
│   ├── 体素降采样: voxelgrid_sampling_omp(...)
│   └── 计算协方差: estimate_covariances_omp(...)
│       └── 得到 source_cloud_PointCovariance_
├── 【分支配准策略】
│   ├── 【TRACKING状态】使用pre_result_作为初始猜测（连续跟踪）
│   │   └── result = registration.align(*global_map_PointCovariance_, *source_cloud_PointCovariance_, *target_tree_, pre_result_)
│   └── 【INIT/RESET状态】使用initial_guess_作为初始猜测（来自Quatro）
│       └── result = registration.align(*global_map_PointCovariance_, *source_cloud_PointCovariance_, *target_tree_, initial_guess_)
├── 【收敛失败处理】
│   └── if (!result.converged)
│       ├── 如INIT/RESET状态: 调用 reset(), 发布fallback TF, 返回
│       └── 如TRACKING状态: gicp_failed_counter_++, 返回
└── 【收敛成功处理】
    ├── 保存结果: pre_result_ = result.T_target_source
    ├── 发布TF变换 (map→odom)
    │   ├── transform.header.frame_id = "map"
    │   ├── transform.child_frame_id = current_sum_cloud_->header.frame_id
    │   └── tf_broadcaster_->sendTransform(transform)
    ├── 发布配准后点云 (pointcloud_registered_pub_)
    ├── 状态切换检查
    │   └── if (INIT || RESET) → state_ = TRACKING
    └── 重置失败计数器: gicp_failed_counter_ = 0
```

5. GICP精配准流程（TRACKING状态）
```
relocalization(current_sum_cloud_)
├── 前置检查
│   └── if (!getInitialPose_) 返回错误
├── 【GICP配准配置】
│   ├── Registration<GICPFactor, ParallelReductionOMP> registration
│   ├── 设置线程数: registration.reduction.num_threads = num_threads_
│   ├── 设置最大距离: registration.rejector.max_dist_sq = max_dist_sq_
│   └── 源点云降采样+协方差计算: source_cloud_PointCovariance_
├── 【分支配准策略】
│   ├── 【TRACKING状态】使用pre_result_作为初始猜测（连续跟踪）
│   │   └── result = registration.align(..., pre_result_)
│   └── 【INIT/RESET状态】使用initial_guess_作为初始猜测（来自Quatro）
│       └── result = registration.align(..., initial_guess_)
├── 【收敛失败处理】
│   └── if (!result.converged && 非TRACKING状态)
│       ├── 调用 reset()
│       ├── 发布初始猜测作为TF（让系统有参考）
│       └── 提前返回
└── 【收敛成功处理】
    ├── 保存结果: pre_result_ = result.T_target_source
    ├── 发布TF变换 (map→odom)
    │   ├── transform.header.frame_id = "map"
    │   ├── transform.child_frame_id = current_sum_cloud_->header.frame_id
    │   └── tf_broadcaster_->sendTransform(transform)
    ├── 状态切换检查
    │   └── if (INIT || RESET) → state_ = TRACKING
    └── 打印成功信息
```



## 八、数据流向
```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              输入层 (Input Layer)                                │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐                   │
│  │ /cloud_registered│  │ /initialpose    │  │ PCD地图文件      │                   │
│  │ (去畸变点云)      │  │ (RViz初始位姿)   │  │ (离线地图)       │                   │
│  └────────┬────────┘  └────────┬────────┘  └─────────────────┘                   │
└───────────┼───────────────────┼───────────────────────────────────────────────────┘
            │                   │
            ▼                   ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                      点云回调层 (pointcloud_sub_callback)                      │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │ 步骤1: 调试模式检查 (test_cloud_registered_)                           │  │
│  │          │                                                             │  │
│  │          ▼ true                                                        │  │
│  │    计算点云间隔 → 打印日志 → 结束                                      │  │
│  │          │ false                                                       │  │
│  │          ▼                                                             │  │
│  │ 步骤2: 配准任务检查 (is_QUAandGICP_running_)                           │  │
│  │          │                                                             │  │
│  │          ▼ true                                                        │  │
│  │    打印丢弃信息 → 结束                                                 │  │
│  │          │ false                                                       │  │
│  │          ▼                                                             │  │
│  │ 步骤3: 状态分发                                                        │  │
│  │    ┌──────────┬──────────┬─────────────────┐                           │  │
│  │    │          │          │                 │                           │  │
│  │    ▼          ▼          ▼                 ▼                           │  │
│  │   INIT     RESET    TRACKING      use_rviz_revise_                     │  │
│  │    │          │          │            │                                │  │
│  │    ▼          ▼          │            │                                │  │
│  │ accumulate*  accumulate* │            │                                │  │
│  │  (带debug)   (无debug)   │            │                                │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────────────┘
            │                   │              │
            ▼                   ▼              ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                    点云积累层 (accumulate_cloud_then_QUAandGICP*)              │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │ 流程:                                                                  │  │
│  │  1. pcl::fromROSMsg → current_cloud (新建智能指针)                     │  │
│  │  2. lock_guard → init_current_clouds_vector.push_back(current_cloud)   │  │
│  │  3. if (size >= count_num)                                             │  │
│  │       │                                                                │  │
│  │       ▼ true                                                           │  │
│  │     累加点云 → voxelgrid_sampling_omp → summary_downsampled_cloud_     │  │
│  │     is_QUAandGICP_running_ = true                                      │  │
│  │     protecting_vector = init_current_clouds_vector (拷贝)              │  │
│  │     init_current_clouds_vector.clear()                                 │  │
│  │     std::async(std::launch::async, QUA_GICP_init_and_reset*)           │  │
│  │       │                                                                │  │
│  │       └─────────────────────────────────────────────┐                  │  │
│  │                                                    ▼                   │  │
│  │                                          【新线程执行】                │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────────────┘
            │                                                        │
            │                              ┌─────────────────────────┘
            │                              ▼
            │            ┌──────────────────────────────────────────────┐
            │            │     Quatro+GICP层 (QUA_GICP_init_and_reset*)  │
            │            │                                               │
            │            │  ┌─────────────────────────────────────────┐  │
            │            │  │ 步骤1: Quatro++ 粗配准                  │  │
            │            │  │   m_quatro_handler->align(              │  │
            │            │  │     summary_downsampled_cloud_,         │  │
            │            │  │     global_map_downsampled_,            │  │
            │            │  │     if_valid_)                          │  │
            │            │  │        │                                │  │
            │            │  │        ▼ false                          │  │
            │            │  │   清空向量 → 设置is_QUAandGICP_running_=false │
            │            │  │        │ true                            │  │
            │            │  │        ▼                                │  │
            │            │  │ 步骤2: 提取最近track_accumulation_counter_帧 │
            │            │  │   current_sum_cloud_for_gicp (累加)     │  │
            │            │  │        │                                │  │
            │            │  │        ▼                                │  │
            │            │  │ 步骤3: GICP精配准 → relocalization()    │  │
            │            │  │        │                                │  │
            │            │  │        ▼                                │  │
            │            │  │ 步骤4: 清空向量 → is_QUAandGICP_running_=false │
            │            │  └─────────────────────────────────────────┘  │
            │            └──────────────────────────────────────────────┘
            │                              │
            └──────────────────────────────┤
                                           │
┌──────────────────────────────────────────┴───────────────────────────────────┐
│                       GICP跟踪层 (GICP_tracking)                               │
│                                                                                │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │ 流程:                                                                  │  │
│  │  1. pcl::fromROSMsg → current_cloud (新建智能指针)                     │  │
│  │  2. 滑动窗口管理 (deque)                                               │  │
│  │       if (!is_queue_full_): push_back → 检查是否满 → 未满则return      │  │
│  │       else: pop_front → push_back                                      │  │
│  │  3. 频率控制: gicp_run_counter_++                                      │  │
│  │     if (counter % tracking_frequency_divisor_ != 0) → return           │  │
│  │  4. 累加窗口内所有点云 → current_sum_cloud_for_gicp                    │  │
│  │  5. → relocalization(current_sum_cloud_for_gicp)                       │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────────────┘
                                           │
                                           ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                       GICP精配准层 (relocalization)                            │
│                                                                                │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │ 输入: current_sum_cloud_ (PCL点云指针)                                 │  │
│  │                                                                         │  │
│  │ 步骤1: 源点云预处理                                                     │  │
│  │   • voxelgrid_sampling_omp → source_cloud_PointCovariance_ (降采样)    │  │
│  │   • estimate_covariances_omp (计算协方差)                              │  │
│  │                                                                         │  │
│  │ 步骤2: 配置GICP配准器                                                   │  │
│  │   • Registration<GICPFactor, ParallelReductionOMP>                     │  │
│  │   • num_threads, max_dist_sq, gicp_max_iterations,                     │  │
│  │     gicp_convergence_tolerance                                         │  │
│  │                                                                         │  │
│  │ 步骤3: 执行配准 (分状态选择初始猜测)                                    │  │
│  │   • TRACKING状态: 使用 pre_result_ (上一次结果)                        │  │
│  │   • INIT/RESET状态: 使用 initial_guess_ (来自Quatro)                   │  │
│  │   • registration.align(global_map_PointCovariance_,                    │  │
│  │                      source_cloud_PointCovariance_,                    │  │
│  │                      target_tree_,                                     │  │
│  │                      initial_guess/pre_result)                         │  │
│  │                                                                         │  │
│  │ 步骤4: 结果处理                                                         │  │
│  │   ├─► 成功 (result.converged):                                         │  │
│  │   │     • pre_result_ = result.T_target_source                         │  │
│  │   │     • 发布TF (map→odom)                                            │  │
│  │   │     • 发布配准后点云 (/pointcloud_registered)                      │  │
│  │   │     • 状态切换: INIT/RESET → TRACKING                              │  │
│  │   │     • gicp_failed_counter_ = 0                                     │  │
│  │   │                                                                    │  │
│  │   └─► 失败:                                                            │  │
│  │         • INIT/RESET: reset() → 发布fallback TF → return               │  │
│  │         • TRACKING: gicp_failed_counter_++                             │  │
│  │           if (>1): reset() → 状态=RESET                                │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────────────┘
                                           │
                                           ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                           输出层 (Output Layer)                                │
│                                                                                │
│  ┌─────────────────┐  ┌─────────────────────────┐  ┌─────────────────────┐   │
│  │ TF变换          │  │ 配准后点云               │  │ 状态字符串           │   │
│  │ /tf (map→odom)  │  │ /pointcloud_registered  │  │ /registration_status│   │
│  └─────────────────┘  └─────────────────────────┘  └─────────────────────┘   │
│                                                                                │
└──────────────────────────────────────────────────────────────────────────────┘
```

## 九、函数调用关系
| 调用层数 | 函数名 | 调用者 | 调用函数 | 说明 |
| --- | --- | --- | --- | --- |
|1	| pointcloud_sub_callback	| ROS2订阅回调	| accumulate_cloud_then_QUAandGICP_with_debug / accumulate_cloud_then_QUAandGICP / GICP_tracking	| 点云回调入口，状态分发 |
|2	| accumulate_cloud_then_QUAandGICP_with_debug	| pointcloud_sub_callback	| QUA_GICP_init_and_reset_with_debug(异步)	| INIT状态，带调试信息 |
|2	| accumulate_cloud_then_QUAandGICP	| pointcloud_sub_callback	| QUA_GICP_init_and_reset(异步)	| RESET状态，无调试信息 |
|2	| GICP_tracking	| pointcloud_sub_callback	| relocalization	| TRACKING状态或RViz模式 |
|3	| QUA_GICP_init_and_reset_with_debug	| accumulate_cloud_then_QUAandGICP_with_debug(异步)	| relocalization	| Quatro粗配准 + GICP精配准(带计时)|
|3	| QUA_GICP_init_and_reset	| accumulate_cloud_then_QUAandGICP(异步)	| relocalization	| Quatro粗配准 + GICP精配准(无计时)|
|4	| relocalization	| GICP_tracking / QUA_GICP_init_and_reset*	| reset(失败时)	| GICP精配准核心函数 |
|公用	| reset	relocalization(GICP失败2次)	| 无	| 重置状态到RESET，清空队列|
|公用	| load_pcd_map	| 构造函数	| 无	| 加载PCD地图，构建KD-Tree |
|公用	| timer_callback	| 定时器	| 无	| 定时发布地图和状态 |

## 十、删除的函数publishTransform
```cpp
void RelocationNode::publishTransform(const Eigen::Matrix4d &transform_matrix)
    {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = current_accumulated_cloud_->header.frame_id;

        // 提取平移部分 (4x4矩阵的最后一列)
        transform.transform.translation.x = transform_matrix(0, 3);
        transform.transform.translation.y = transform_matrix(1, 3);
        transform.transform.translation.z = transform_matrix(2, 3);
        // 提取旋转部分 (3x3左上子矩阵)
        Eigen::Matrix3d rotation = transform_matrix.block<3, 3>(0, 0);
        Eigen::Quaterniond quat(rotation);
        transform.transform.rotation.x = quat.x();
        transform.transform.rotation.y = quat.y();
        transform.transform.rotation.z = quat.z();
        transform.transform.rotation.w = quat.w();

        tf_broadcaster_->sendTransform(transform);
    }
```




## 点云话题：/cloud_registered {#点云话题订阅}
- `/cloud_registered`来自fast_livo2，订阅话题为/cloud_registered。一般在导航系统中，我们认为odom系就是雷达开机位置这个不随机器人运动的坐标系。
- `/cloud_registered`中的“Registered” 在 LiDAR SLAM 中是通用术语，指 “经过坐标配准（Registration）+ 运动畸变校正” 的点云 —— 区别于原始 LiDAR 话题（如`/points_raw`）的 “未配准、带运动畸变” 点云，`/cloud_registered`的命名核心是突出其 “坐标已对齐到固定坐标系、无畸变” 的属性
- 内容：
    - header.frame_id: 在`fast-livo2`源代码中是“camera_init”，定义是第一帧雷达数据的imu的位姿，他是固定不变的，**所以在TF中他就是odom坐标系（mid360的imu和雷达发射中心外参差非常小，这里可以忽略）**
        - 至于为什么是`camera_init`这个名字：我们的算法是fastlivo2关闭img_en,这一部分是视觉惯性导航模式的残留，没有被改过来，算法里应该可以改回来。
    - 点云数据：经过紧耦合 LiDAR-IMU 里程计配准 + 运动畸变校正后的点云，具备两个关键特性：
        - 无运动畸变：雷达转一圈需要100ms，这100ms里如果机器人运动了，点云的坐标就会发生变化，而fastlio2会对点云进行运动畸变校正，所以点云的坐标是没有运动畸变的
        - 坐标系：计算点云的xyz是相对于camera_init的，也就是第一帧雷达数据的imu的位姿，也就是odom。**你也可以近似认为这个点云就是【当前帧】建图结果，其实建图也就是把每一帧的雷达点云找到一个他到另一个坐标系的绝对位置，最终的建图结果点云是这些单帧积累的结果，并还需要经过去重、下采样、全局优化，回环检测等步骤得到最终结果，发布到`/cloud_map`**
- 总结：
    - 用/cloud_registered做为配准源是非常正确的
    - 重定位节点中，所有要拿到点云的frame_id的地方，拿到的都是odom，实现的变换也就是map → odom