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
#ifndef RS_CONFIG_MANAGER_H_
#define RS_CONFIG_MANAGER_H_
#include <iostream>
#include <map>
#include <array>
#include <boost/filesystem.hpp>
#include <panta_common/common.h>
#include "panta_preprocessing/transform.h"
#include "panta_preprocessing/panta_time.h"
namespace robosense
{
namespace preprocessing
{

std::size_t static const LIDAR_NO_MAX = 6;

class SensorTF
{
public:
  SensorTF();
  std::string parent_frame_id;
  std::string frame_id;
  // std::string topic;
  bool is_use;
  int unit;
  robosense::preprocessing::Pose pose;
  SensorTF& operator=(const SensorTF& other);
};
std::ostream& operator<<(std::ostream& os, const SensorTF& sensor_tf);


struct SensorGroup
{
  std::string base_link_frame_id;
  SensorTF imu;
  SensorTF odom;
  SensorTF gnss;
  // 根据配置文件获得实际lidar个数
  std::vector<SensorTF, Eigen::aligned_allocator<SensorTF>> lidars;
};
std::ostream& operator<<(std::ostream& os, const SensorGroup& sensor_group);

struct SensorState
{
  TicToc timer;          // receive time(system time)
  double last_time = 0;  // stamp of last(sensor time)
  bool is_use = false;
  bool is_ok = false;
};

struct SensorStateGroup
{
  SensorState imu;
  SensorState odom;
  SensorState gnss;
  std::array<SensorState, LIDAR_NO_MAX> lidars;
};

enum SpeedUnit
{
  METER_PER_SECOND = 0,
  KILOMETER_PER_HOUR = 1,
  MILE_PER_HOUR = 2,
};

struct PreProcConfig
{
public:
  std::string version;
  std::string calib_file;
  YAML::Node calib_node;
  double timestamp_tlv = 0.5;      // second
  bool is_motion_correct = false;  // inner state
  bool use_3d_compensate = false;
  bool use_localization_result = false;
  int imu_correction=-1;

  // imu
  bool is_use_imu = false;
  float imu_timeout = 2.0;

  // odom
  bool is_use_odom = false;
  int odom_speed_unit = SpeedUnit::METER_PER_SECOND;
  float odom_timeout = 3.0;

  // gnss
  bool is_use_gnss = false;
  int gnss_speed_unit = SpeedUnit::KILOMETER_PER_HOUR;
  float gnss_timeout = 4.0;

  // lidar
  bool is_use_lidar = false;
  std::size_t lidar_cnt = 0;
  std::string fusion_frame_id = "/fusion";
  float lidar_timeout = 2.0;
};

std::ostream& operator<<(std::ostream& os, const PreProcConfig& _config);

/**
 *  解析配置文件的参数，并做文件检查
 * */
bool loadConfig(const std::string& _config_file, PreProcConfig& config);

bool loadSensorGroup(PreProcConfig& _config, SensorGroup& sensor_group);

bool loadConfig(const YAML::Node& _config_node, PreProcConfig& config);

bool loadSensor(const YAML::Node& node, SensorTF& sensor);

void printErrorCode(const robosense::common::ErrCode& code);

}  // namespace preprocessing
}  // namespace robosense
#endif /* ifndef CONFIG_MANAGER_H_ */
