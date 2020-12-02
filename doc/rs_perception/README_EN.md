# Perception Module User Guide

- version: 2.5.0

[TOC]

## 1. Perception Module Introduction

​		Perception Module is the key function module of the whole SDK, the core functions involving：

- Obstacle Perception
- Obstacle Classification
- Dynamic Object Tracking 
- Freespace / Drivable Zone Detection

## 2. WorkMode

### 2.1 Pointcloud Only

​		By this way, only pointcloud data is needed as the input, be care, the timestamp is needed. At this situation, the perception output is based on **lidar local coordinate**, so the tracking information like speed is **relative quantity**.   		

### 2.2 Pointcloud and Global Pose

​		By this way, pointcloud with timestamp and global pose information estimated by localization module or odometry module are both needed. At this situation, the perception output is based on **global coordiante**, so the tracking information like speed is **absolute quantity**. 

## 3. Perception Related User Configuration

​		Considering the different application situations, we provide a configuration file that can be tuned by users according to their specific need. This file named with `rs_perception.yaml` and located in path`rs_sdk/config/system_config/perception_config/`: 

```yaml
common: # common args before invoke the perception algorithm core
  perception_mode: 0 # 0: perception with points only, 1: perception with points and pose (from localizaiton or odometry module)
  with_debug_info: false # enable debug info output or not
  perception_frame_id: rs_perception # setting perception info frame_id
algorithm: # true perception algorithm args
  ## robosense perception sdk user configure
  ## !!!Notes:
  ## all of the "range" parameters are set based on lidar coordinate (lidar itself is origin, right-hand, not vehicle, be carefull)
  ##========================common config===========================
  common_config:
    lidar_info:
      hori_resolution: 0.2
      vert_resolution: 0.5
    enable_auto_align: true
    ignore_range: # if object located within the 'ignore_ange', they will be moved out
      xmin: -2.5 # (background of the vehicle)
      xmax: 2.5 # (foreground of the vehicle)
      ymin: -1.5 # (right side of the vehicle)
      ymax: 1.5 # (left side of the vehicle)
      zmin: -3 # (lower side of the vehicle)
      zmax: 3 # (upper side of the vehicle)
    enable_fusion_adjust: false # true for P3
    enable_height_adjust: false # true for old model (ground is below zero, sdk version <=2.3)
    enable_class_enhance: false
    enable_roi: false
  ##========================deeplearning detection config 1===========================
  deep_detection_config: # for rotational lidar like rs-32
    detect_range: # main detection range, should set smaller if with low computing power such as nvidia edge devices like tx2 or xavier
      xmin: -100 #just like 'ignore_range' setting
      xmax: 100
      ymin: -60
      ymax: 60
      zmin: -3
      zmax: 5
    unit_size: 0.2 # 0.1875 # be care! this should not be changed if keep deeplearning model unchanged.
    obstacle_confidence_thd: 0.5
    object_confidence_thd: 0.1
    regress_height_thd: 0.5
    density_base: 1.0
    enable_intensity_normalize: false
    enable_fp_16: false # should be set true when use with nvidia edge devices like tx2 or xavier
    max_workspace: 30
    min_pts_num: 3
    ground_height_thd: 1.0
    ground_h_error_thd: 0.2
    overlap_thd: 0.5
  ##========================deeplearning detection config 2===========================
  deep_detection_config2: # for mems lidar like rs-m1
    rows: 64 # be care! this should not be changed if keep deeplearning model unchanged.
    cols: 512 # be care! this should not be changed if keep deeplearning model unchanged.
    range: 150 # main detection range, should set smaller if with low computing power such as nvidia edge devices like tx2 or xavier
    min_height: -5
    max_height: 5
    min_theta: -15
    max_theta: 5
    min_phi: -60
    max_phi: 60
    min_pts_num: 3
    delta_x: 0.5
    delta_y: 0.5
    use_intensity: false # should set true when intensity channel data is ok
    enable_intensity_normalize: false
    enable_fp_16: false  # should be set true when use with nvidia edge devices like tx2 or xavier
    max_workspace: 30
    ground_height_thd: 1.0
    ground_h_error_thd: 0.2
    ground_grid_size: 0.25
  ##========================second stage detection refine config===========================
  refine_detection_config: # complementary detection range, to make sure no objects are missed, only near field are recommended
    enable_detection_refine: false
    refine_range: 
      xmin: -50 #just like 'ignore_range' setting
      xmax: 50
      ymin: -10
      ymax: 10
      zmin: -3
      zmax: 4
    enable_geo_filter: true
    obj_limit:
      obj_max_height: 5.0
      obj_size_height: 4.0
      obj_size_length: 3.0
      obj_size_width: 2.0
      obj_length_width_ratio: 5.0
      obj_max_area: 5.0
    enable_upper_detect: true
    min_pts_num: 3
    overlap_thd: 0.5
  ##========================tracking config===========================
  tracking_config:
    enable_tracking: true
    enable_track_optimize: false # can be set true when global pose is obtained, in order to promote the box direciton
    enable_adaptive_filter_gain: false
    enable_check_filter_state: false
    enable_weighted_direction_filter: false # !!!
    enable_direction_cue_to_filter: false # !!! be care, should with global pose estimation or static
    enable_adaptive_measure_noise_estimate: false # !!!
    enable_history_improve_filter: true
    enable_check_static_obj: true
    predict_time: 0.5
    history_num: 10
    eva_seq_size: 10
    basic_velocity_noise: 0.4
    match_distance_max: 3.0
  ##========================dynamic detection===========================
  dynamic_detect_config: # dynamic detection, (not recommended to use now)
    enable_dynamic_detection: false
    dynamic_detect_range: 80
    min_hori_angle: -180
    max_hori_angle: 180
    min_vert_angle: -25
    max_vert_angle: 10
  ##========================freespace detection config===========================
  freespace_detect_config: # freespace detection
    enable_freespace_detection: true
    vertical_range:
      min: -1
      max: 4.5
    min_hori_angle: -60
    max_hori_angle: 60
    sector_num: 60
    free_width_thre: 1.5
  ##========================post denoising (dust, tail gas etc.) filter config===========================
  post_denoise_config: # in order to filter out miss-detections caused by dust or tail gas of vehicle (not recommended to use now)
    enable_post_denoise_filter: false
    post_denoise_range:
      xmin: -10
      xmax: 5
      ymin: -5
      ymax: 5
      zmin: -3
      zmax: 5
    post_denoise_filter_thd: 0.5
```

## 4. Output List

​		For every received lidar data frame, perception module will process and output a detailed detection results with obstacle list. All the information are based on global coordinate by default. Without specification, the measurements are all by metric unit: m, s, rad, m/s, m/s^2, rad/s.

| Perception Content (format)            | Specification                                                |
| -------------------------------------- | ------------------------------------------------------------ |
| **timestamp** (double)                 | global timestamp in seconds;                                 |
| **device_code**                        | device code used to distinguish which device the obstacle comes from, useful in vehicle-road cooperation situation; |
| **id** (int32)                         | obstacle detection ID, from 0 to 2^31-1, only valid for the current frame, i.e., will be reinit every frame, if is -1, means the obstacle is not detected successfully, so can not be trusted; |
| **anchor** (float, 3)                  | anchor point estimated from barycenter of obstacle pointcloud; |
| **geo_center** (float, 3)              | geometric center of bounding box of the obstacle estimated from pointcloud; |
| **geo_size** (float, 3)                | geometric size of bounding box of the obstacle estimated from pointcloud, length>=width; |
| **geo_direction** (float, 3)           | geometric direction of bounding box of the obstacle estimated from pointcloud, horizontal to the long edge of the box. |
| **polygon** (float, 3)xN               | the convex polygon of obstacle pointcloud, the corner number is up to the obstacle shape, may variant from 4 to even 10; |
| **detect_confidence** (float)          | the detection confidence of the obstacle, without classification; |
| **nearest_point** (float, 3)           | the nearest corner of polygon of the obstacle to the lidar center; |
| **left_point** (float, 3)              | the left-most corner of polygon of the obstacle when seeing from the lidar center; |
| **right_point** (float, 3)             | the right-most corner of polygon of the obstacle when seeing from the lidar center; |
| **distance**(float)                    | the distance from the nearest_point to the lidar center;     |
| **yaw** (float)                        | the yaw angle of the anchor point relative to lidar local coordiante; |
| **point_num** (int32)                  | the number of the obstacle pointcloud;                       |
| **type** (int32)                       | obstacle type optimized by tracking: 0 is unknow, 1 is pedestrian, 2 is bicycle, 3 is small vehicle and 4 is big vehicle; |
| **type_confidence** (float)            | the confidence for the classification from classification module, 0~1, the greater, the better; |
| **latent_types** (float)xN             | all the potential types confidence the obstacle may belong to; |
| **motion_state** (int32)               | obstacle dynamic state, 0 means unknow, 1 means moving, 2 means static, 3 means stoped. the different between 'static' and 'stoped' is that the 'stoped' means potential movable obstacle like car but currently keep static. |
| **is_track_converged** (bool)          | obstacle tracking state: tracked succeed or not, if not, the tracking information is not reliable; |
| **tracker_id** (int32)                 | obstacle tracking ID, from 0 to 10000, if get bigger, will reinit from 0, valid accross frame sequence all the time, if is -1, means the obstacle is not tracked successfully; |
| **velocity** (float, 3)                | obstacle velocity with unit m/s;                             |
| **velocity_cov** (float, 9)            | estimated covariance of the velocity;                        |
| **velocity_uncertainty** (float)       | uncertainty of the velocity estimated from historical velocities; |
| **ave_velocity** (float, 9)            | average velocity estimated from historical velocities by 10 frames by default; |
| **acceleration** (float, 3)            | obstacle acceleration with unit m^2/s;                       |
| **acceleration_cov** (float, 9)        | estimated covariance of the acceleration;                    |
| **acceleration_uncertainty** (float)   | uncertainty of the acceleration estimated from historical accelerations; |
| **ave_acceleration** (float, 9)        | average acceleration estimated from historical velocities by 10 frames by default; |
| **angle_velocity** (float)             | obstacle angular velocity with unit rad/s;                   |
| **angle_velocity_cov** (float)         | estimated covariance of the angular velocity;                |
| **angle_velocity_uncertainty** (float) | uncertainty of the angular velocity estimated from historical angular velocities; |
| **ave_angle_velocity** (float)         | average angular velocity estimated from historical angular velocities by 10 frames by default; |
| **asso_quality** (float)               | the confidence for the obstacle when associated to a tracker, 0~1, the greater, the better; |
| **tracker_quality** (float)            | the robustness for a tracker that the obstacle belong to, 0~1, the greater, the better; |
| **tracking_time** (double)             | the total time for the obstacle been tracked since been first detected, including visible or shadowed in seconds; |

**Note**:

1. **If no global information (via localization or RTK) is provided, the percepted results are correspongding to Lidar local coordinate**. It means that the velocity of obstacles are relative to lidar (for example, the static roadside object will have a speed) and so dose the location and other informations. Please be aware of a fact that, for tracking, if no global information is provided, there is no way to get a global velocity relative to the ground.
2. If user want get global velocity and have no map, please adding **rs_odometry** module which utilize SLAM technology to estimate ego-motion；
3. User can recover bounding-box of obstacle by assemble the **geo_center**, **geo_size** and **geo_direction** information. For example,  use **geo_size** to recover the box size, use **geo_center** to recover the box position, use **geo_direction** to recover the box posture.  If just want to get a stable position estimation, **anchor** are recommended. 
4. **polygon** **is highly recommended**. Compared with box (mainly for vehicle), polygon is a more common way to depict obstacles. What's more, considering the sparsity of point-cloud, it is hard to estimate the box accurately. This will result in the shaking of the box which is harmful to the following trafficability estimation. Polygon dose not suffer this kind of problem. 

## 5. Output format

### 5.1 Robosense Format

​		This message format can be translated between modules within the SDK. If users wants a second development based on robosen SDK, this format can be used.

### 5.2 Protobuf Format

​		By this way, perception messages can be broadcasted or received by UDP/Socket.

### 5.3 ROS Format

​		By this way, users can sue rviz tool provided by ROS to receive and debug the perception results by subscribing the topic:  `/rs_obstacle_rviz` : 

- Box with different color means different classification type, represented by several ROS namespaces ```/box``` and ```/cube``` or ```/polygon``` under topic ```/percept_info_rviz``` : 

| Box Color | Specification                        |
| :-------- | :----------------------------------- |
| gray      | unknow                               |
| blue      | big vehicle like truck or bus        |
| red       | small vehicle like car or small vans |
| cyan      | bicycle or motorcycle with rider     |
| yellow    | pedestrian                           |

- Different color of pointcloud means different part of original input pointcloud:

| Point-cloud Color  | ROS Topic Name  | Specification                     |
| :----------------- | :-------------- | :-------------------------------- |
| green              | /ground         | ground points                     |
| pale white or blue | /original_cloud | original point-cloud              |
| colored            | /cluster        | original detected obstacle points |
| same with /cluster | /trajectory     | historical position by tracking   |
| dark orange        | /valid_space    | SDK detected range                |

- Different color of text means different perception information:

| Text Type                 | ROS Topic Name                  | Specification                              |
| :------------------------ | :------------------------------ | :----------------------------------------- |
| "17.8 (5.0 2.0 1.6)"      | /percept_info_rviz/box_info     | "distance (length, width, height)"         |
| "<1169> 20.8km/h >> 0.87" | /percept_info_rviz/track_info   | "<track ID> velocity >> association_score" |
| "car >> 0.79"             | /percept_info_rviz/label_info   | "type >> type_confidence"                  |
| green arrow               | /percept_info_rviz/velocity_dir | vectorized velocity                        |

**Note**:

Not all the percepted information all displayed in rviz. We provide the source code of display, please refer the code in `rs_sdk/src/perception_rviz.cpp`. You can customize the display by yourself. 