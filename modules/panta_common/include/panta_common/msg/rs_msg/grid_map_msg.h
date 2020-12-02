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
 *****************************************************************************
 * @file vehiclestate_msg.h
 * @brief Header file which defines all msg types in RS Algorithms
 * @author YICHI ZHANG
 * @version 1.0.0
 * @date 2019-03-27
 * @copyright robosense
 *****************************************************************************/

#pragma once
#include <string>
#include <vector>
namespace robosense
{
namespace common
{
/**
   * @brief GridMap Message for robosense SDK.
   * 
   * A GridMap message stores either entire or partial map used in localization algorithm  
   * 
   */
struct alignas(16) GridMap
{
  double timestamp = 0.0;
  uint32_t seq = 0;
  std::string frame_id = "";
  std::string parent_frame_id = "";
  uint32_t width = 0;
  uint32_t height = 0;
  float resolution = 1.0;
  std::array<float,2> origin{{0.0, 0.0}};
  std::array<float,4> orientation{{0.0 ,0.0 ,0.0 ,1.0}};
  std::vector<float> grids;
};

} // namespace common
} // namespace robosense