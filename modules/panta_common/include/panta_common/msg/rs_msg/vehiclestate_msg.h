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
#include <array>
namespace robosense
{
namespace common
{
/**
   * @brief Vehicle state for robosense SDK.
   * 
   * The VehicleState is used as the output from localization module
   * and input to perception module when absolute coordinate is required
   * 
   */
struct alignas(16) VehicleStateMsg
{
  double timestamp = 0.0;
  uint32_t seq = 0;
  std::string frame_id = "";
  std::string parent_frame_id = "";
  uint32_t status = 0;

  std::array<double, 3> origin{};          ///< Map origin in world fram(WGS84). Interms of [lat lon alt]
  std::array<double, 3> pos{};             ///< Position in map frame. Unit: meter, 0:x 1:y 2:z
  std::array<double, 9> pos_cov{};         ///<
  std::array<double, 3> orien{};           ///< Orientation in map frame. Unit: rad
  std::array<double, 9> orien_cov{};       ///<
  std::array<double, 3> angular_vel{};     ///< Angular velocity in map frame. Unit: rad/s
  std::array<double, 9> angular_vel_cov{}; ///<
  std::array<double, 3> linear_vel{};      ///< Linear velocity in map frame. Unit: m/s
  std::array<double, 9> linear_vel_cov{};  ///<
  std::array<double, 3> acc{};             ///< Linear acceleration in map frame. Unit: m/s^2
  std::array<double, 9> acc_cov{};         ///<

  std::array<double, 4> toQuaternion(const std::array<double, 3> &origin) const
  {
    std::array<double, 4> result{};
    double half_row = 0.5*origin[0];
    double half_pitch = 0.5*origin[1];
    double half_yaw = 0.5*origin[2];
    result[0] = sin(half_row) * cos(half_pitch) * cos(half_yaw) - cos(half_row) * sin(half_pitch) * sin(half_yaw); //x
    result[1] = cos(half_row) * sin(half_pitch) * cos(half_yaw) + sin(half_row) * cos(half_pitch) * sin(half_yaw); //y
    result[2] = cos(half_row) * cos(half_pitch) * sin(half_yaw) - sin(half_row) * sin(half_pitch) * cos(half_yaw); //z
    result[3] = cos(half_row) * cos(half_pitch) * cos(half_yaw) + sin(half_row) * sin(half_pitch) * sin(half_yaw); //w
    return std::move(result);
  }
  std::array<double, 3> toEuler(const std::array<double, 4> &origin) const
  {
    std::array<double, 3> result{};
    double x = origin[0];
    double y = origin[1];
    double z = origin[2];
    double w = origin[3];
    result[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)); //row
    result[1] = asin(2 * (w * y - z * x));                         // pitch
    result[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));  //yaw
    return std::move(result);
  }
};

} // namespace common
} // namespace robosense
