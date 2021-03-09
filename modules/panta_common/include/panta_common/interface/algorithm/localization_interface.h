/******************************************************************************
 * Copyright 2017 robosense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by robosense and might
 * only be used to access robosense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without robosense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL robosense BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#pragma once

#include <string>
#include <functional>
#include <panta_common/common.h>
#include <panta_common/msg/rs_msg/gnss_msg.h>
#include <panta_common/msg/rs_msg/imu_msg.h>
#include <panta_common/msg/rs_msg/lidar_points_msg.h>
#include <panta_common/msg/rs_msg/odom_msg.h>
#include <panta_common/msg/rs_msg/vehiclestate_msg.h>
#include <panta_common/msg/rs_msg/grid_map_msg.h>

namespace robosense
{
namespace common
{

class LocalizationInterface : virtual public CommonBase
{
public:
  union ModuleStatus {
    struct
    {
      unsigned int module_initialized : 1;
      unsigned int key_check : 1;           /*  */
      unsigned int localization_init : 3;   /* 000: , 001: Not Initial, 010: Initializing, 011: Retry, 100: Fail, 101:Success*/
      unsigned int localization_status : 3; /*000: , 001: Idle, 010: Normal, 011: Low Accuracy, 100: Lost*/
      unsigned int imu_overflow : 1;        //6
      unsigned int gnss_overflow : 1;       //7
      unsigned int odom_overflow : 1;       //8
      unsigned int lidar_overflow : 1;      //9
    };
    uint32_t status_int = 0;
  };

public:
  virtual ~LocalizationInterface() = default;
  virtual ErrCode init(const std::string &base_config_path, const YAML::Node &params) = 0;
  virtual ErrCode start(const int &init_method = 0,
                        const Eigen::Matrix<double, 6, 1> &pose_guess = Eigen::Matrix<double, 6, 1>::Zero()) = 0;
  virtual ErrCode stop() = 0;
  virtual ErrCode resetVehicleState(const VehicleStateMsg &state) = 0;
  virtual ModuleStatus getModuleStatus() = 0;
  virtual ErrCode getVehicleState(VehicleStateMsg &state) = 0;
  virtual ErrCode getVehicleState(VehicleStateMsg &state, double t) = 0;
  virtual ErrCode logVehicleState(VehicleStateMsg &state, std::string &path) = 0;
  virtual ErrCode getMapOriginGlobal(Eigen::Vector3d &lat_long_alt) = 0;
  virtual ErrCode toGlobalCoordinate(const Eigen::Vector3d &local, Eigen::Vector3d &global) = 0;
  virtual ErrCode toLocalCoordinate(const Eigen::Vector3d &global, Eigen::Vector3d &local)=0;
  virtual ErrCode getMap(GridMap &map) = 0;
  virtual ErrCode getMap(LidarPointsMsg &map) = 0;
  virtual double getProcessingTime() = 0;

  virtual void regExceptionCallback(const std::function<void(const ErrCode &exception)> &) = 0;
  virtual ErrCode imuCallback(const ImuMsg &msg) = 0;
  virtual ErrCode gnssCallback(const GnssMsg &msg) = 0;
  virtual ErrCode odomCallback(const OdomMsg &msg) = 0;
  virtual ErrCode lidarCallback(const LidarPointsMsg &msg) = 0;
};
} /* namespace common */
} /* namespace robosense */
