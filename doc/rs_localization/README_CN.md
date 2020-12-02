# rs_localization使用说明


- [rs_localization使用说明](#rs_localization使用说明)
  - [1. 简介](#1-简介)
  - [2. 快速入门](#2-快速入门)
  - [3. 依赖](#3-依赖)
    - [3.1 软件依赖](#31-软件依赖)
    - [3.2 硬件依赖](#32-硬件依赖)
  - [4. 消息格式](#4-消息格式)
  - [5. 坐标系定义](#5-坐标系定义)
  - [6. API](#6-API)

## 1. 简介

**rs_localization** 是速腾聚创算法SDK(**rs_sdk**)中的核心模块。该模块基于高精度地图和多传感器融合技术,为自动驾驶、机器人导航等系统提供实时的定位信息。 

## 2. 快速入门 

如果您对**rs_sdk**和**rs_localization**还不熟悉，您可以参考**rs_sdk**工程目录中的文档```(doc/README_CN.md)```学习如何编译、配置并运行rs_sdk的定位功能。

## 3. 依赖
### 3.1 软件依赖 
**rs_localization**模块依赖于**rs_sdk**中其他基础模块, 包括:
- rs_sensor
- rs_common
- rs_preprocessing

使用前,请务必保证各模块下载、编译完毕。具体编译方法请参考**rs_sdk**工程目录中的```doc/README_CN.md```文件。

### 3.2 硬件依赖
**rs_localization**基于多传感器融合原理进行定位, 需提供以下传感器:

传感器   |          消息类型          
:------:|:-------------------------:
激光雷达   | robosense::common::LidarPointsMsg  
GPS/RTK | robosense::common::GnssMsg|  
惯性测量单元(IMU)|robosense::common::ImuMsg |
车速  | robosense::common::OdomMsg|

传感器驱动可使用**rs_sensor**中提供的驱动程序。具体使用方法参考**rs_sensor**中的文档。

## 4. 消息格式

**rs_sdk** 定义了算法所需的所有消息格式。其中rs_localization使用的包括：


- **robosense::common::VehicleStateMsg**
  
  用于描述定位结果（即车体状态估计), 包含位置，姿态，速度，角速度等状态估计信息以及时间戳，坐标系等必要信息。

- **robosense::common::LidarPointsMsg**
  
  用于传递激光雷达的测量信息，包含时间戳，坐标系，点云等。

- **robosense::common::GnssMsg**
  
  用于传递RTK/GPS的测量信息，包含时间戳、经纬度及其协方差等。如果使用RTK，还包含线速度，姿态以及各自的协方差

- **robosense::common::ImuMsg**
  
  用于传递惯性测量单元(IMU)的测量信息，包含时间戳、加速度、加速度协方差、角速度、角速度协方差等。

- **robosense::common::OdomMsg**

  用于传递车速信息，包含时间戳，车速， 车速协方差等。

以上消息都定义在**rs_common**模块的include/rs_common/msg/rs_msg文件夹里的各相应头文件中。

## 5. 坐标系定义

定位模块涉及两个坐标系, 分别是全局坐标系与局部坐标系:

* **全局坐标系**（也称地图坐标系，世界坐标系）采用ENU定义，原点以经纬度的形式保存，X轴指向东，Y轴指向北，Z轴垂直向上。这个样的坐标系定义使得一个点的位置能够用比较小的数字表示， 同时也能很容易地将其变换到WGS-84 或者ECEF坐标系中。

* 为了表示车辆在全局坐标系中的位姿，引入一个**局部坐标系**，固联在车体的运动中心上，X轴指向正前，Y轴指向左，Z垂直XY平面向上，从而构成一个右手系。车体的姿态由局部坐标系相对于全局坐标系的Roll-Pitch-Yaw表示（欧拉角的一种）， 位置由局部坐标系原点的坐标表示。车体运动中心目前定义为汽车后轴中心在地面的投影。
  

## 6. API
**rs_localization**模块提供了若干接口(Application Programming Interface, API)，方便用户将定位功能集成到自己的解决方案中。

**rs_sdk**工程中的```src/rs_sdk_demo_node.cpp```和```src/rs_sdk_demo.cpp```提供了一个完整的示例程序， 使用**rs_localization** 和其他模块的API构建一个带可视化功能的定位系统。

关于API的详细文档，请参考 [API.md](API.md)。
