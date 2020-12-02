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
   * @brief Gnss Message for robosense SDK.
   * @detail robosense GnssMsg is defined for passing Gnss related information accross different modules
   *         If ROS is turned on , we provide translation functions between ROS message and robosense message
   *         If Proto is turned on , we provide translation functions between Protobuf message and robosense message
   * 
   */
struct alignas(16) GnssMsg
{
  double timestamp = 0.0;
  uint32_t seq = 0;
  std::string parent_frame_id = ""; ///< the frame id of its coordinate(***equal to frame_id in ROS***)
  std::string frame_id = "";        ///< the frame id of its self

  int status = 0;                ///< the Gnss status
  uint8_t type = 0;                       ///< GNSS Sensor type, 'R' : RTK  'G' : GPS
  uint8_t sat_cnt = 0;                    ///< Number of satellite connected. Should be at least 8 for good accuracy.
  std::array<double, 3> pos{};            ///< Interms of [lat lon alt]
  std::array<double, 9> pos_cov{};        ///< Position covariance, unit: meter^2
  std::array<double, 3> orien{};          ///< Orientation, [row pitch yaw], unit: rad
  std::array<double, 9> orien_cov{};      ///< Orientation covariance, unit: rad^2
  std::array<double, 3> linear_vel{};     ///< Linear velocity, [x,y,z], unit: m/s
  std::array<double, 9> linear_vel_cov{}; ///< Linear velocity covariance, unit: m^2/s^2
};

} // namespace common
} // namespace robosense
