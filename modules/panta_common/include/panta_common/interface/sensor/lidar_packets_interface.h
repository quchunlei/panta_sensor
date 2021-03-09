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

#include "panta_common/common.h"
#include "panta_common/msg/rs_msg/lidar_packet_msg.h"
#include "panta_common/msg/rs_msg/lidar_scan_msg.h"

namespace robosense
{
namespace common
{
/**
 * @brief Lidar packets message interface
 * @detail 1, Will be inheritted by Lidar packets ROS message adapter
 *         2, Will be inheritted by driver base which relate to Lidar packets messages(eg: LidarBase)
 */
class LidarPacketsInterface : virtual public CommonBase
{
public:
  LidarPacketsInterface() = default;
  virtual ~LidarPacketsInterface() = default;

  /**
  * @brief initialize function
  * @param config--yaml node
  */
  virtual ErrCode init(const YAML::Node &config) = 0;
  /**
  * @brief start function
  */
  virtual ErrCode start() = 0;
  /**
  * @brief stop function
  */
  virtual ErrCode stop() = 0;
  /**
  * @brief send function
  * @detail send lidar msop packets message through ROS
  * @param msg--the robosense LidarScanMsg message
  */
  virtual void send_msop(const LidarScanMsg &msg) {}
  /**
  * @brief send function
  * @detail send lidar difop packet message through ROS
  * @param msg--the robosense LidarPacketMsg message
  */
  virtual void send_difop(const LidarPacketMsg &msg) {}
  /**
  * @brief register receive call back function
  * @detail after registration, the Lidar module can pass lidar msop packet message to other module
  * @param callBack--the function pointer of the call back function
  */
  virtual void regRecvCallback(const std::function<void(const LidarScanMsg &)> callBack) = 0;
  /**
  * @brief register receive call back function
  * @detail after registration, the Lidar module can pass lidar difop packet message to other module
  * @param callBack--the function pointer of the call back function
  */
  virtual void regRecvCallback(const std::function<void(const LidarPacketMsg &)> callBack) = 0;
  /**
  * @brief register exception call back function
  * @detail after register the exception call back function, the lidar packets module can pass the error code to other module
  * @param callBack--the function pointer of the exception call back function
  */
  virtual void regExceptionCallback(const std::function<void(const ErrCode &)> excallBack) = 0;
  /**
  * @brief get the api of the module
  * @detail return the supported_api_
  */
  static uint16_t getApi() { return supported_api_; }

private:
  /**
  * @brief  
  * 0000 0000 0000 0001-only support Odom
  * 0000 0000 0000 0010-only support Gnss
  * 0000 0000 0000 0100-only support Imu
  * 0000 0000 0001 0000-only support Lidar packets
  * 0000 0000 0010 0000-only support Lidar points
  * 
  * @detail
  * e.g. If a module inheritted Gnss interface and Odom interface, which means it can support Gnss message and Odom message(Such like the GnssBase)
  *       Thus, it's supported_api_ should be set to 0000 0000 0000 0011 --support Odom & Gnss
  *       Through this varible(supported_api_), you can easily know what messages type the module supports 
  */
  static const uint16_t supported_api_ = 0x0010; // 0000 0000 0001 0000 (only support LiDAR packets)
};
} // namespace common
} // namespace robosense
