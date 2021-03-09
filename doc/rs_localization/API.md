# RSLocalization

简介
---
**RSLocalization**类用于实现定位功能。 用户可以使用它提供的接口完成配置、启动定位功能，查询定位状态、实时获取定位结果等操作。

### 名称空间(namespace)

robosense::localization

### 构造函数
#### RSLocalization()

> 默认构造函数, 实例化定位所需的所有成员变量。

#### RSLocalization(const std::string& name)
> 构造一个名为```name```的RSLocalization对象  
>  
> **参数:**  
> ```name```: 所构造的RSLocalization对象的名字，用于显示在定位输出的信息中

### 成员函数（接口）
#### void regExceptionCallback(const std::function<void(const ErrCode& exception)>& exception_callback)
>必须在调用构造函数后，紧接着调用该函数，用于在RSLocalization中注册一个异常回调函数，当定位配置、启动和运行过程中出现任何异常都会传递一个robosense::common::ErrCode类型的异常码给该函数进行处理。这是**rs_sdk**中处理异常的主要机制。 ErrCode定义在**rs_common**的```debug/error_code.h```文件中
>  
> **参数**:  
> ```exception_callback```: 异常回调函数对象，该函数参数为const ErrCode&, 返回值为void

#### ErrCode init(const std::string& base_config_path, const YAML::Node& params)
> 使用yaml参数对RSLocalization进行初始化，实现对定位功能的配置。  
>   
> **参数:**  
> ```base_config_path```: rs_sdk最外层的配置文件config.yaml的绝对路径 。　　
> 
> ```params```: 读取定位配置文件得到的yaml节点, 可通过**rs_common**中的YamlParser类读取配置文件获取。　　
>   
> **返回值** 　　
> 
> ```error_code```: error_code 为ErrCode_Success时，表示初始化成功。若初始化过程出现异常导致初始化失败，会返回异常码,同时用该异常码调用```exception_callback```。

#### ErrCode start()
> 用于启动定位模块。该函数返回后，定位模块开始工作。  
>
>**返回值**  
>
> 返回ErrCodeSuccess,表示定位程序成功启动, 否则会返回一个异常码，并用其调用```exception_callback```。

#### ErrCode stop()
> 停止定位模块的运行。执行该函数将使定位模块中所有子线程都退出
> 
>**返回值**  
>
> 返回ErrCodeSuccess,表示定位程序模块停止, 否则会返回一个异常码，并用其调用```exception_callback```。

#### ErrCode resetVehicleState(const VehicleStateMsg &state)
> 重置定位模块中的各个状态并用参数值更新状态估计（定位结果）。相当于给定位模块重新设置一个初始值定位值。可用于定位初始位置计算失效或者定位过程中定位出错后的人工恢复。
>   
> **参数:**  
> ```state```: 用于重置定位模块的初始定位值
> 
> **返回值** 　　
> 
> 返回ErrCodeSuccess,表示定位程序模块重置成功, 否则会返回一个异常码，并用其调用```exception_callback```。

#### ModuleStatus getModuleStatus()
> 获取当前定位模块的状态 
>
> 定位模块的状态共有12位，分别表示：

> **bit 1**:   定位模块是否初始化  
> **bit 2**:   定位模块密钥检验成功与否  
> **bit 3~5**: 计算初始定位过程中的状态  
> ----- 001: 为开始计算初始定位  
> ----- 010: 正在计算初始定位  
> ----- 011: 计算失败，正在重新计算  
> ----- 100: 计算失败且放弃重试  
> ----- 101: 计算成功
> **bit6~8**: 定位运行状态　　
> ----- 001: 空闲  
> ----- 010: 定位正常  
> ----- 011: 定位精度低 
> ----- 100: 定位丢失  
> **bit9**: imu消息队列溢出  
> **bit10**: gnss消息队列溢出  
> **bit11**: odom消息队列溢出  
> **bit12**: Lidar消息队列溢出  
>
> 在**rs_common**里的```LocalizationInterface.h```中定义了```ModuleStatus```可方便状态的获取和判断。  
> 
> 例如：  
> 　
>> using robosense::common::LocalizationInterface::ModuleStatus;  
>> ModuleStatus s = rslocalization.getModuleStatus();  
>> if(s.module_initialized==1)  
>> ...   
>> else if(s.module.localization_status == 2)   
>> ...
> 
> **返回值** 　　
> 
> 返回一个ModuleStatus类型，表示当前定位程序的实时状态。

#### ErrCode getVehicleState(VehicleStateMsg &state) 
> 获取时间距离当前最近的一个状态估计结果(定位结果)。　　
> 
> 注意，该结果一般不是当前时刻的定位结果，而是根据最近一次收到的传感器消息所计算得到的结果。
>   
> **参数:**  
> ```state```: 传入一个VehicleStateMsg引用, 存储获取的结果　
> 
> **返回值** 　　
> 
> 返回ErrCodeSuccess,表示定位程序模块重置成功, 否则会返回一个异常码，并用其调用```exception_callback```。

#### ErrCode getVehicleState(VehicleStateMsg &state, double t) 
> 获取时间距离当前最近的一个状态估计结果并插值计算到给定的时刻, 返回给定时刻的状态估计结果。  
> 比如，将当前的时刻t作为参数传入，可以获得当前时刻的定位结果。
>   
> **参数:**  
> ```state```: 传入一个VehicleStateMsg引用, 存储获取的结果  
> ```t```: 给定的时刻 (UNIX时间或POSIX时间)
> 
> **返回值** 　　
> 
> 返回ErrCodeSuccess,表示定位结果获取成功, 否则会返回一个异常码，并用其调用```exception_callback```。

#### ErrCode getMapOriginGlobal(Eigen::Vector3d &lat_long_alt)
> 获取全局坐标系的原点
>   
> **参数:**  
> ```lat_long_alt```: 存储获取的结果, 全局坐标系的原点以经纬度表示在WGS-84坐标系中  
> 
> **返回值** 　　
> 
> 返回ErrCodeSuccess,表示获取成功, 否则会返回一个异常码，并用其调用```exception_callback```。

#### toGlobalCoordinate(const Eigen::Vector3d& local, Eigen::Vector3d& global)
> 将定位结果的xyz值转换到WGS-84坐标系
>   
> **参数:**  
> ```local```: 定位结果的xyz值  
> ```global```: 转换到WGS-84坐标系下的xyz值
> 
> **返回值** 　　
> 
> 返回ErrCodeSuccess,表示转换成功, 否则会返回一个异常码，并用其调用```exception_callback```。

#### ErrCode getMap(GridMap& map)
> 获取定位的栅格地图, 一般用于定位结果可视化  
> 栅格地图的格式定义在**rs_common**的```include/msg/rs_msg/grid_map_msg.h```中
>   
> **参数:**  
> ```map```: 获取到的地图  
> 
> **返回值** 　　
> 
> 返回ErrCodeSuccess,表示获取成功, 否则会返回一个异常码，并用其调用```exception_callback```。

#### ErrCode imuCallback(const ImuMsg &msg)
> 用于获取IMU数据输入的回调函数。将该函数注册到**rs_preprocessing**或者**rs_sensor**中, 可使定位模块获得IMU传感器数据输入  
> 例如：　　
>
>> // 从**rs_preprocessing**获取imu数据  
>> 
>> preProcess_ptr_->regRecvCallback(std::bind(&localization::RSLocalization::imuCallback,localization_ptr_.get(), std::placeholders::_1));

#### ErrCode gnssCallback(const GnssMsg &msg)
> 用法与imuCallback相同，用于获取gnss传感器数据输入  


#### ErrCode odomCallback(const OdomMsg &msg)  
>用法与imuCallback相同，用于获取odom传感器数据输入

#### ErrCode lidarCallback(const LidarPointsMsg &msg)
>用法与imuCallback相同，用于获取Lidar传感器数据输入
