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
   * @brief Imu Message for robosense SDK.
   * @detail robosense ImuMsg is defined for passing Imu related information accross different modules
   *         If ROS is turned on , we provide translation functions between ROS message and robosense message
   *         If Proto is turned on , we provide translation functions between Protobuf message and robosense message
   * 
   */
struct alignas(16) ImuMsg
{
  double timestamp = 0.0;
  uint32_t seq = 0;
  std::string parent_frame_id = ""; ///< the frame id of its coordinate(***equal to frame_id in ROS***)
  std::string frame_id = "";        ///< the frame id of its self

  std::array<double, 3> orien{};           ///< Orientation, [row pitch yaw], unit: rad
  std::array<double, 9> orien_cov{};       ///< Orientation covariance, unit: rad^2
  std::array<double, 3> angular_vel{};     ///< Angular velocity, [x,y,z], unit: rad/s
  std::array<double, 9> angular_vel_cov{}; ///< Angular velocity covariance, unit: rad^2/s^2
  std::array<double, 9> angular_bias_cov{};
  std::array<double, 3> acc{};     ///< Acceleration, [x,y,z], unit: m/s^2
  std::array<double, 9> acc_cov{}; ///< Acceleration covariance
  std::array<double, 9> acc_bias_cov{};
};

} // namespace common
} // namespace robosense
