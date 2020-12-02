# 速腾聚创算法SDK感知模块使用手册

- 版本: 2.5.0

[TOC]

## 1. 感知模块简介

​		感知模块是速腾聚创算法SDK中的主要模块之一，负责提供感知能力，主要包括如下核心功能：

- 障碍物检测
- 障碍物分类
- 运动物体跟踪
- 可行驶区域（freespace）检测

## 2. 感知模块的工作模式介绍

### 2.1 纯点云工作模式

​		此时感知模块只需要输入点云数据（需要带时间戳信息，跟踪需要）即可，不过，需要注意，此时感知输出的是**相对速度**，障碍物检测信息是相对于**车体坐标系**的。

### 2.2 点云+全局位姿信息工作模式

​		此时感知模块需要输入点云数据（需要带时间戳信息，跟踪需要）和全局位姿信息（可以来源于定位模块或者惯性导航模块），此时感知输出的是**绝对速度**，障碍物检测信息是相对于**全局坐标系**的。

## 3. 感知算法相关用户配置文件说明

​		考虑到不同场景的需求不同，我们提供用户配置文件来提供灵活性。用户可以通过该配置文件来配置感知算法的一些参数，例如感知范围，是否关闭某些模块以节约算力等，该配置文件为置于`rs_sdk/config/system_config/perception_config/`目录下的`rs_perception.yaml`:

```yaml
common: # 感知模块通用调用方式参数，包括设置感知模块的工作模式、是否开启debug信息输出等
  perception_mode: 0 # 0: 只有感知模块自己工作, 1: 感知模块配合ego-motion估计模块 (通过定位模块或者实时Odometry估计模块)
  with_debug_info: false # 是否开启调试信息输出到控制台窗口，为调试测试准备，正常运行时建议关闭
  perception_frame_id: rs_perception # 设置感知信息的frame_id
algorithm: # 感知算法相关的参数
  ## 注意：
  ## 以下参数中的所有“range”相关的参数都是针对lidar坐标系设置的，不是针对车体坐标系或者全局坐标系。
  ##========================感知算法通用参数设置===========================
  common_config: #感知算法通用参数
    lidar_info: #雷达信息设置
      hori_resolution: 0.2 # 水平角分辨率，角度（°）
      vert_resolution: 0.5 # 竖直角分辨率，角度（°）
    enable_auto_align: true # 是否启用自动水平校准，当车辆运动过程中晃动比较严重时建议开启
    ignore_range: # 设置排除区域，主要目的是排除车辆自身造成的误检，一般设置为车体自身的范围
      xmin: -2.5 # (车辆后方距离雷达的距离)
      xmax: 2.5 # (车辆前方距离雷达的距离)
      ymin: -1.5 # (车辆右侧距离雷达的距离)
      ymax: 1.5 # (车辆左侧距离雷达的距离)
      zmin: -3 # (车辆下方距离雷达的距离)
      zmax: 3 # (车辆上方距离雷达的距离)
    enable_fusion_adjust: false # 当配合P3模式使用时开启，单个32线时关闭
    enable_height_adjust: false # 是否开启高度兼容调整(sdk version <=2.3 时开启)
    enable_class_enhance: false # 保留位
    enable_roi: false # 是否开启ROI，默认关闭
  ##========================深度学习模型1参数设置：为32线系列产品准备===========================
  deep_detection_config:
    detect_range: # 检测范围，对于xavier或者tx2，可以减少范围以节约计算力，检测范围面积和时间消耗几乎成正比
      xmin: -100 # 检测范围后方范围
      xmax: 100 # 检测范围前方范围
      ymin: -60 # 检测范围右侧范围
      ymax: 60 # 检测范围左侧范围
      zmin: -3 # 检测范围下方范围
      zmax: 5 # 检测范围上方范围
    unit_size: 0.2 # 0.1875 # 模型基本参数，与具体模型有关，需要配合模型文件调整，用户请勿自行调整！
    obstacle_confidence_thd: 0.5 # 模型基本参数，与具体模型有关，需要配合模型文件调整，用户请勿自行调整！
    object_confidence_thd: 0.1 # 模型基本参数，与具体模型有关，需要配合模型文件调整，用户请勿自行调整！
    regress_height_thd: 0.5 # 模型基本参数，与具体模型有关，需要配合模型文件调整，用户请勿自行调整！
    density_base: 1.0 # 模型基本参数，与具体模型有关，需要配合模型文件调整，用户请勿自行调整！
    enable_intensity_normalize: false # 模型基本参数，与具体模型有关，需要配合模型文件调整，用户请勿自行调整！
    enable_fp_16: false # 当配合xavier或者tx2时，请一定要开启，这个是使用fp_16来进行推断，可以充分挖掘支持fp_16数据类型的加速器性能
    max_workspace: 30 # 显存预分配指标，30意味着预分配2^30字节，越大则分配的显存越大，不过够用就好，不是越大越好，不建议用户自行调整
    min_pts_num: 3 # 最小点数，当检测的物体的点数少于3个，则被认为是噪点而滤除
    ground_height_thd: 1.0 # 地面检测相关阈值，表示高于地面一米的点不会参与地面检测
    ground_h_error_thd: 0.2 # 地面检测相关阈值，表示位于同一栅格中的点的高度差如果大于该值则不会参与地面检测
    overlap_thd: 0.5 # 模型基本参数，与具体模型有关，需要配合模型文件调整，用户请勿自行调整！ 
  ##========================深度学习模型2参数设置：为MEMS线系列产品准备===========================
  deep_detection_config2:
    rows: 64 # 模型基本参数，与具体模型有关，需要配合模型文件调整，用户请勿自行调整！
    cols: 512 # 模型基本参数，与具体模型有关，需要配合模型文件调整，用户请勿自行调整！
    range: 150 # 检测范围半径，当不需要感知如此大的范围时，可以减小以节约算力
    min_height: -5 # 检测范围下界：距离地面5米以下会被忽略
    max_height: 5 # 检测范围上界：距离地面5米以上会被忽略
    min_theta: -15 # 检测范围角度下界：纵向pitch角-15°以下会被忽略，右手系 
    max_theta: 5 # 检测范围角度上界：纵向pitch角5°以上会被忽略，右手系 
    min_phi: -60 # 检测范围角度右界：水平向yaw角-60°以外会被忽略，右手系
    max_phi: 60 # 检测范围角度左界：水平向yaw角60°以外会被忽略，右手系 
    min_pts_num: 3 # 最小点数，当检测的物体的点数少于3个，则被认为是噪点而滤除
    delta_x: 0.5 # 模型基本参数，与具体模型有关，需要配合模型文件调整，用户请勿自行调整！  
    delta_y: 0.5 # 模型基本参数，与具体模型有关，需要配合模型文件调整，用户请勿自行调整！ 
    use_intensity: false # 是否使用点云数据的intensity通道
    enable_intensity_normalize: false # 模型基本参数，与具体模型有关，需要配合模型文件调整，用户请勿自行调整！
    enable_fp_16: false  # 当配合xavier或者tx2时，请一定要开启，这个是使用fp_16来进行推断，可以充分挖掘支持fp_16数据类型的加速器性能
    max_workspace: # 显存预分配指标，30意味着预分配2^30字节，越大则分配的显存越大，不过够用就好，不是越大越好，不建议用户自行调整
    ground_height_thd: 1.0 # 地面检测相关阈值，表示高于地面一米的点不会参与地面检测
    ground_h_error_thd: 0.2 # 地面检测相关阈值，表示位于同一栅格中的点的高度差如果大于该值则不会参与地面检测
    ground_grid_size: 0.25 # 模型基本参数，与具体模型有关，需要配合模型文件调整，用户请勿自行调整！ 
  ##=========================补充检测模块设置==========================
  refine_detection_config: # 补充检测模块，用于对关键区域进行二次检测，以保证不会漏检，一般用于自身周边较小的区域内
    enable_detection_refine: false # 默认不开启
    refine_range: # 二次补检测范围，前后50米，左右10米，主要服务于当前车道附近
      xmin: -50
      xmax: 50
      ymin: -10
      ymax: 10
      zmin: -3
      zmax: 4
    enable_geo_filter: true # 二次补检几何尺寸滤波，滤除太高、太大、长宽比太悬殊的物体
    obj_limit:
      obj_max_height: 5.0 
      obj_size_height: 4.0
      obj_size_length: 3.0
      obj_size_width: 2.0
      obj_length_width_ratio: 5.0
      obj_max_area: 5.0
    enable_upper_detect: true # 二次补检内部算法参数，是否开启车辆上方检测
    min_pts_num: 3 # 最小点数，当检测的物体的点数少于3个，则被认为是噪点而滤除
    overlap_thd: 0.5 # 重叠阈值，大于0.5的两个物体会被和并
  ##========================跟踪参数设置===========================
  tracking_config: # 跟踪模块
    enable_tracking: true # 是否开启tracking，对于低速情况，如果不需要tracking，可以关闭节约算力
    enable_track_optimize: false # 是否开启利用tracking对检测进行优化，需要在能够获取全局速度的情况下使用，这就需要配合定位或者odometry模块才可以使用
    enable_adaptive_filter_gain: false # 暂不建议开启
    enable_check_filter_state: false # 暂不建议开启
    enable_weighted_direction_filter: false # !!! # 暂不建议开启
    enable_direction_cue_to_filter: false # !!! # 暂不建议开启
    enable_adaptive_measure_noise_estimate: false # !!! # 暂不建议开启
    enable_history_improve_filter: true # 是否开启使用历史序列优化当前跟踪
    enable_check_static_obj: true  # 是否开启静态物体判断，降低对静态物体闪烁的干扰
    predict_time: 0.5 # 跟踪预测时长，超过预测时长后仍然没有关联上新物体，则该跟踪序列丢失
    history_num: 10 # 历史序列长度设置
    eva_seq_size: 10 # 序列分析长度设置
    basic_velocity_noise: 0.4 # 最小速度分辨力设置
    match_distance_max: 3.0 # 最大关联距离设置，超过该距离的物体将丢失关联
  ##========================动静态检测参数设置===========================
  dynamic_detect_config: # 动静态检测模块（暂不建议开启）
    enable_dynamic_detection: false
    dynamic_detect_range: 80
    min_hori_angle: -180
    max_hori_angle: 180
    min_vert_angle: -25
    max_vert_angle: 10
  ##========================可通行区域检测参数设置===========================
  freespace_detect_config: # 可通行区域检测模块，极坐标系下进行
    enable_freespace_detection: true
    vertical_range: # 纵向检测范围设置，地面以下1米到地上4.5米范围
      min: -1
      max: 4.5
    min_hori_angle: -60 # 检测范围角度右界：水平向yaw角-60°以外会被忽略，右手系
    max_hori_angle: 60 # 检测范围角度左界：水平向yaw角60°以外会被忽略，右手系
    sector_num: 60 # 输出格式，检测范围会被分成60个扇区输出
    free_width_thre: 1.5 # 扇区最小可通行宽度设置，小于该宽度的扇区会被和并
  ##========================后处理参数设置===========================
  post_denoise_config: # 后处理模块，为了滤除由于灰尘、汽车尾气等造成的误检
    enable_post_denoise_filter: false # 默认不建议开启，暂不够成熟
    post_denoise_range:
      xmin: -10
      xmax: 5
      ymin: -5
      ymax: 5
      zmin: -3
      zmax: 5
    post_denoise_filter_thd: 0.5 # 滤除算法内部参数，越大则滤除的越多
```

## 4. 输出感知信息列表

对于每一帧数据，感知模块会输出一个感知物体列表，对于每个障碍物，将输出如下格式消息，**默认是相对全局坐标系**的，单位为公制单位（m（米）、s（秒）、m/s（米/秒）、m/s^2、rad（弧度）、rad/s）。关于SDK坐标系设置，请参考 [coordinates](./../coordinates.md)：

| 感知内容(数据格式)                     | 说明                                                         |
| -------------------------------------- | ------------------------------------------------------------ |
| **timestamp** (double)                 | 全局的时间戳信息；                                           |
| **device_code**                        | 当前物体检测结果来自的设备编号，为车路协同准备；             |
| **id** (int32)                         | 物体的检测ID，范围是0～2^31-1，只能在当前帧（一帧）中有效，如果为-1，说明物体还未被成功检测，检测结果不可信； |
| **anchor** (float, 3)                  | 物体的锚点（较稳定位置估计），由点云重心初始化；             |
| **geo_center** (float, 3)              | 物体包围盒的几何中心，会随着包围盒估计的变换而变化；         |
| **geo_size** (float, 3)                | 物体包围盒的尺寸，按长宽高顺序，长度>=宽度；                 |
| **geo_direction** (float, 3)           | 物体自身的朝向，单位向量，平息与物体包围盒的长边；           |
| **polygon** (float, 3)xN               | 物体最小包围凸多边形在水平面（X-Y平面）投影的投影顶点集合，顶点个数N与物体的具体形状有关，可能是4边形，6边形甚至是10边形，z值高度等于物体的最低高度; |
| **detect_confidence** (float)          | 物体检测的置信度，注意此处不包含分类，与分类置信度不同；     |
| **nearest_point** (float, 3)           | 从雷达中心看向物体，物体距离雷达的最近点坐标，取自**polygon**中的最近顶点； |
| **left_point** (float, 3)              | 从雷达中心看向物体，物体最左侧的顶点；                       |
| **right_point** (float, 3)             | 从雷达中心看向物体，物体最右侧的顶点；                       |
| **distance**(float)                    | 物体最近点到雷达中心的距离；                                 |
| **yaw** (float)                        | 物体锚点位置在雷达坐标系下的水平分布方向角；                 |
| **point_num** (int32)                  | 物体点云个数；                                               |
| **type** (int32)                       | 物体初始检测的类别，0表示未知，1表示行人，2表示骑行者，3表示小型车辆，4表示大型车辆； |
| **type_confidence** (float)            | 物体分类的置信度，0～1，越高越好；                           |
| **latent_types** (float)xN             | 物体所有潜在类别的置信度；                                   |
| **motion_state** (int32)               | 物体运动状态检测，0表示未知，1表示运动，2表示静止，3表示停止。停止和静止的区别在于停止是指有潜在运动能力，例如停止的车辆和行人； |
| **is_track_converged** (bool)          | 物体是否被跟踪成功，如果没有，那么说明跟踪信息是不可靠的；   |
| **tracker_id** (int32)                 | 物体的跟踪ID，范围是0~10000，如果更大，会循环重新从0开始，如果为-1，说明物体还未被成功跟踪； |
| **velocity** (float, 3)                | 物体速度;                                                    |
| **velocity_cov** (float, 9)            | 物体速度估计的协方差；                                       |
| **velocity_uncertainty** (float)       | 物体速度估计的不确定性，越小越好                             |
| **ave_velocity** (float, 9)            | 考虑物体若干历史帧（默认为10帧）的平均速度；                 |
| **acceleration** (float, 3)            | 物体全局坐标系下的加速度;                                    |
| **acceleration_cov** (float, 9)        | 物体加速度估计的协方差；                                     |
| **acceleration_uncertainty** (float)   | 物体加速度估计的不确定性，越小越好                           |
| **ave_acceleration** (float, 9)        | 考虑物体若干历史帧（默认为10帧）的平均加速度；               |
| **angle_velocity** (float)             | 物体的角速度;                                                |
| **angle_velocity_cov** (float)         | 物体角速度估计的协方差；                                     |
| **angle_velocity_uncertainty** (float) | 物体角速度估计的不确定性，越小越好                           |
| **ave_angle_velocity** (float)         | 考虑物体若干历史帧（默认为10帧）的平均角速度；               |
| **asso_quality** (float)               | 物体被关联到一个跟踪序列的关联质量（置信度），0～1，越高越好； |
| **tracker_quality** (float)            | 物体所在的跟踪序列的稳定度，根据该跟踪序列历史数据评估得到，0～1，越高越好； |
| **tracking_time** (double)             | 物体总的被跟踪时长，包括可见帧和不可见帧（被遮挡）；         |

- **注意：**

1. **如果未提供全局信息（通过定位算法或RTK），则只依赖感知模块获取的感知结果是基于激光雷达局部坐标系的**。这意味着障碍物的速度相对于激光雷达（例如，静态路边物体将具有速度），同理对于定位和其它感知信息也是一样的。请注意，对于跟踪，如果未提供全局信息，则无法获得相对于地面的全局速度。
2. 如果在没有定位的情况下得到绝对速度信息，需要配合**rs_odometry**模块来进行，该模块可以通过实时SLAM技术估计车体绝对姿态；
3. 如果用户希望通过感知信息来恢复障碍物的包围盒信息，可以通过**geo_center**、**geo_size**、**geo_direction**信息来恢复。首先根据长宽高来恢复box的尺寸，然后通过中心点来恢复位置，最后通过方向来恢复姿态（航向角）。而如果只是希望估计物体的位置，则建议使用**anchor**， 相比**geo_center**会更稳定一些；
4. **推荐用户使用polygon**，相对于立方体包围盒（主要适合车辆描述），多边形是一种更通用的障碍物描述方式。另外，基于点与数据的散乱稀疏性，准确估计立方体包围盒比较困难，所以包围盒一般会存在抖动问题，会影响后续算法可通行性评估，多边形不存在这种问题。

## 5. 输出信息格式

### 5.1 Robosense自定义格式

​		这种格式主要用于速腾算法SDK内部模块之间通信，如果用户希望在速腾SDK的基础上进行二次开发，那么可以使用这种格式传递信息。

### 5.2 Protobuf格式

​		此时，可以通过UDP/Socket通信模式发送和接收消息。

### 5.3 ROS格式

​		此时可以通过ros提供的rviz工具来调试查看接收到感知信息可视化的结果：订阅 ```/rs_obstacle_rviz``` topic。

- 不同颜色的包围盒意味着不同的分类类型，通过topic ```/rs_obstacle_rviz```下面的 ```/box```和```/cube```来描述，另外也可以使用多边形```/polygon```来表征物体：

| 包围盒颜色 | 说明                     |
| :----- | :------------------------- |
| 灰色 | 未知物体                 |
| 蓝色 | 卡车或公共汽车等大型车辆 |
| 红色 | 汽车或小型货车等小型车辆 |
| 青色 | 骑行者                   |
| 黄色 | 行人                     |

- 点云的不同颜色表示原始输入点云的不同部分，通过不同消息发布：

| 点云颜色           | ROS Topic 名称                       | 说明                     |
| :----------------- | :------------------------------------- | :------------------------- |
| 绿色             | /ground                              | 地面点                   |
| 奶白色或淡蓝色 | /original_cloud | 原始点云                 |
| 彩色             | /cluster                             | 原始检测出来的障碍物点云 |
| 和 /cluster 同色 | /trajectory                          | 跟踪历史轨迹             |
| 暗橙色    | /valid_space                     | SDK检测区域              |

- 不同颜色的文字表示不同的感知信息， 通过同一个消息的不同`namespace`发布： 

| 文本形式                  | ROS Topic 名称                   | 说明                                       |
| :-------------------------- | :--------------------------------: | :------------------------------------------: |
| "17.8 (5.0 2.0 1.6)"      | /percept_info_rviz/box_info | "距离 (长, 宽, 高)"         |
| "<1169> 20.8km/h >> 0.87" | /percept_info_rviz/track_info | "<跟踪ID> 速度 >> 关联置信度" |
| "car >> 0.79"             | /percept_info_rviz/label_info | "类别 >> 类别置信度"                  |
| 绿色箭头         | /percept_info_rviz/velocity_dir | 速度的向量化表示                           |

**注意**：

上述图示并没有将所有感知信息显示输出，显示代码以源码形式提供，详见`rs_sdk/src/perception_rviz.cpp`，您可以自定义要显示的感知信息。
