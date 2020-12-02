# rs_sensor

简介
---
**rs_sensor**模块由sensormanger，除雷达外的其他传感器的驱动，ROS适配器，以及Protobuf适配器组成，核心为sensormanager。
**sensormanager**类用于控制所有传感器驱动（包括雷达），并获取所有传感器（包括雷达）的原始数据， 并将原始数据发送给预处理模块。 同时，sensormanager也可将获得的原始数据通过ROS或Protobuf发出。

### 名称空间(namespace)

**robosense::sensor**

### 构造函数
#### SensorManager()

> 默认构造函数


### 成员函数（接口）
#### ErrCode init(const YAML::Node &sensor_config, bool use_ros, bool use_proto,std::string config_path)
> 使用yaml参数对sensormanager进行初始化，实现对各个传感器的配置。  
>   
> **参数:**  
> ```sensor_config```: 读取定位配置文件得到的yaml节点, 可通过**rs_common**中的YamlParser类读取配置文件获取。
> ```use_ros```: 是否使用ros相关的功能，若为false，则所有ros相关的功能都无法使用。
> ```use_proto```: 是否使用proto相关的功能，若为false，则所有proto相关的功能都无法使用。
> ```base_config_path```: rs_sdk最外层的配置文件config.yaml的绝对路径 。　　
> 　　
>   
> **返回值** 　　
> 
> ```error_code```: error_code 为ErrCode_Success时，表示初始化成功。若初始化过程出现异常导致初始化失败，会返回异常码,同时用该异常码调用```exception_callback```。

#### ErrCode start()
> 用于启动senormanager模块。该函数返回后，sensormanager模块开始工作。  
>
>**返回值**  
>
> 返回ErrCodeSuccess,表示senormanager程序成功启动, 否则会返回一个异常码，并用其调用```exception_callback```。

#### ErrCode stop()
> 停止senormanager模块的运行。执行该函数将使senormanager模块中所有子线程都退出
> 
>**返回值**  
>
> 返回ErrCodeSuccess,表示senormanager程序模块停止, 否则会返回一个异常码，并用其调用```exception_callback```。


#### void regExceptionCallback(const std::function< void(const common::ErrCode &) > &callBack)
>用于在sensormanager中注册一个异常回调函数，当各个传感器运行中出现异常时，会将robosense::common::ErrCode类型的异常码给该函数进行处理。这是**rs_sdk**中处理异常的主要机制。 ErrCode定义在**rs_common**的```debug/error_code.h```文件中
>  
> **参数**:  
> ```exception_callback```: 异常回调函数对象，该函数参数为const ErrCode&, 返回值为void

#### void regRecvCallback(const std::function< void(const common::ImuMsg &) > &callBack) 
>用于在sensormanager中注册一个IMU回调函数，当sensormanager收到新的IMU消息时，调用回调函数。这是**rs_sdk**中消息传递的主要机制。
>  
> **参数**:  
> ```callBack```: IMU消息回调函数

#### void regRecvCallback(const std::function< void(const common::GnssMsg &) > &callBack) 
>用于在sensormanager中注册一个GNSS回调函数，当sensormanager收到新的GNSS消息时，调用回调函数。这是**rs_sdk**中消息传递的主要机制。
>  
> **参数**:  
> ```callBack```: GNSS消息回调函数

#### void regRecvCallback(const std::function< void(const common::OdomMsg &) > &callBack) 
>用于在sensormanager中注册一个Odom回调函数，当sensormanager收到新的Odom消息时，调用回调函数。这是**rs_sdk**中消息传递的主要机制。
>  
> **参数**:  
> ```callBack```: Odom消息回调函数

#### void regRecvCallback(const std::function< void(const common::LidarPointsMsg &) > &callBack) 
>用于在sensormanager中注册一个点云回调函数，当sensormanager收到新的点云消息时，调用回调函数。这是**rs_sdk**中消息传递的主要机制。
>  
> **参数**:  
> ```callBack```: 点云消息回调函数











