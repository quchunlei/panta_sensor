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
#include "rs_lidar/lidar_base.h"

namespace robosense
{
namespace sensor
{
using namespace robosense::common;
ErrCode LidarBase::init(const YAML::Node &config)
{
  setName("LidarBase");
  ST_Param param;
  int resolution;
  int intensity;
  int echo_mode;
  YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
  yamlReadAbort<std::string>(driver_config, "frame_id", frame_id_);
  yamlRead<bool>(driver_config, "use_lidar_clock", use_lidar_clock_, false);
  yamlRead<uint32_t>(driver_config, "timeout", timeout_, 100);
  yamlReadAbort<std::string>(driver_config, "device_type", lidar_model_);
  yamlRead<int>(driver_config, "resolution_type", resolution, 0);
  yamlRead<int>(driver_config, "intensity_mode", intensity, 3);
  yamlRead<int>(driver_config, "echo_mode", echo_mode, 1);
  yamlRead<float>(driver_config, "min_distance", param.min_distance, 0.2);
  yamlRead<float>(driver_config, "max_distance", param.max_distance, 200);
  yamlRead<float>(driver_config, "start_angle", param.start_angle, 0);
  yamlRead<float>(driver_config, "end_angle", param.end_angle, 360);
  yamlRead<float>(driver_config, "cut_angle", param.cut_angle, 0);
  param.resolution = RS_RESOLUTION_TYPE(resolution);
  param.intensity = RS_INTENSITY_TYPE(intensity);
  param.echo = RS_ECHO_MODE(echo_mode);
  lidar_decoder_ptr_ = DecoderFactory<pcl::PointXYZI>::createDecoder(lidar_model_, param);
  lidar_input_ptr_ = std::make_shared<Input>();
  lidar_input_ptr_->init(driver_config);
  pointcloud_ptr_ = PointCloudPtr(new PointCloud);
  thread_flag_ = false;
  excb_ = NULL;
  self_state_ = IDLE;
  points_seq_ = 0;
  scan_seq_ = 0;
  setinitFlag(true);
  return ErrCode_Success;
}

ErrCode LidarBase::start()
{
  if (pointscb_.empty() && pkts_msop_cb_.empty() == 0 && pkts_difop_cb_.empty() == 0)
  {
    ERROR << "LidarBase: Please register at least one callback function first!" << REND;
    exit(-1);
  }
  if (thread_flag_ == false)
  {
    thread_flag_ = true;
    self_state_ = IDLE;
    const auto &func1 = [this] { stateMachine(); };
    lidar_thread_ptr_ = std::make_shared<std::thread>(func1);
#if (DEBUG_LEVEL > 1)
    INFO << "LidarBase START successfully!" << REND;
#endif
  }
  return ErrCode_Success;
}
ErrCode LidarBase::stop()
{
  if (thread_flag_ == true)
  {
    thread_flag_ = false;
    lidar_thread_ptr_->join();
  }
#if (DEBUG_LEVEL > 1)
  INFO << "LidarBase stop" << REND;
#endif
  return ErrCode_Success;
}

bool LidarBase::processMsopPackets(const common::LidarScanMsg &pkt_scan_msg)
{
  if (pkt_scan_msg.packets.size() == 1)
  {
    std::vector<pcl::PointXYZI> point_vec;
    int ret = lidar_decoder_ptr_->processMsopPkt(pkt_scan_msg.packets[0].packet.data(), point_vec);
    if (ret == E_DECODE_OK || ret == E_FRAME_SPLIT)
    {
      for (auto iter = point_vec.cbegin(); iter != point_vec.cend(); iter++)
      {
        pointcloud_ptr_->push_back(*iter);
      }
      if (ret == E_FRAME_SPLIT)
      {
        points_seq_++;
        if (lidar_model_ == "RS32" || lidar_model_ == "RSBP")
        {
          pointcloud_ptr_->height = 32;
        }
        else if (lidar_model_ == "RS16")
        {
          pointcloud_ptr_->height = 16;
        }
        else
        {
          pointcloud_ptr_->height = 1;
        }
        pointcloud_ptr_->width = pointcloud_ptr_->points.size() / pointcloud_ptr_->height;
        pointcloud_ptr_->is_dense = false;
        common::LidarPointsMsg msg(pointcloud_ptr_);
        preparePointsMsg(msg);
        if (use_lidar_clock_ == true)
        {
          ST_MsopPkt *mpkt_ptr = (ST_MsopPkt *)pkt_scan_msg.packets[0].packet.data();
          std::tm stm;
          memset(&stm, 0, sizeof(stm));
          stm.tm_year = mpkt_ptr->header.timestamp.year + 100;
          stm.tm_mon = mpkt_ptr->header.timestamp.month - 1;
          stm.tm_mday = mpkt_ptr->header.timestamp.day;
          stm.tm_hour = mpkt_ptr->header.timestamp.hour;
          stm.tm_min = mpkt_ptr->header.timestamp.minute;
          stm.tm_sec = mpkt_ptr->header.timestamp.second;
          msg.timestamp = std::mktime(&stm) + (double)RS_SWAP_SHORT(mpkt_ptr->header.timestamp.ms) / 1000.0 + (double)RS_SWAP_SHORT(mpkt_ptr->header.timestamp.us) / 1000000.0;
        }
        if ((lidar_model_ == "RS32" || lidar_model_ == "RSBP") && msg.cloudPtr->points.size() > 50000)
        {
          runCallBack(msg);
        }
        else if (lidar_model_ == "RS16" && msg.cloudPtr->points.size() > 25000)
        {
          runCallBack(msg);
        }

        else
        {
          WARNING << " The number of points is smaller than the threshold value! Abort this frame! " << REND;
        }
        pointcloud_ptr_.reset(new PointCloud);
        return true;
      }
    }
  }

  else
  {
    std::vector<std::vector<pcl::PointXYZI>> point_vvec;
    point_vvec.resize(pkt_scan_msg.packets.size());
#pragma omp parallel for
    for (uint32_t i = 0; i < pkt_scan_msg.packets.size(); i++)
    {
      std::vector<pcl::PointXYZI> point_vec;
      int ret = lidar_decoder_ptr_->processMsopPkt(pkt_scan_msg.packets[i].packet.data(), point_vec);
      if (ret == E_DECODE_OK || ret == E_FRAME_SPLIT)
      {
        point_vvec[i] = std::move(point_vec);
      }
    }
    for (auto iiter : point_vvec)
    {
      for (auto iter = iiter.cbegin(); iter != iiter.cend(); iter++)
      {
        pointcloud_ptr_->push_back(*iter);
      }
    }
    points_seq_++;
    if (lidar_model_ == "RS32")
    {
      pointcloud_ptr_->height = 32;
    }
    else if (lidar_model_ == "RS16")
    {
      pointcloud_ptr_->height = 16;
    }
    else
    {
      pointcloud_ptr_->height = 1;
    }
    pointcloud_ptr_->width = pointcloud_ptr_->points.size() / pointcloud_ptr_->height;
    common::LidarPointsMsg msg(pointcloud_ptr_);
    preparePointsMsg(msg);
    msg.timestamp = pkt_scan_msg.timestamp;
    runCallBack(msg);
    pointcloud_ptr_.reset(new PointCloud);
    return true;
  }
  return false;
}
void LidarBase::processDifopPackets(const common::LidarPacketMsg &pkt_msg)
{
  lidar_decoder_ptr_->processDifopPkt(pkt_msg.packet.data());
}
void LidarBase::stateMachine()
{
  InputState ret = INPUT_OK;
  common::LidarPacketMsg pkt_msg;
  common::LidarScanMsg scan_msg;
  common::LidarScanMsg tmp_scan;
  while (thread_flag_)
  {
    switch (self_state_)
    {
    case (IDLE):
    {
      self_state_ = GETPACKET;
      break;
    }
    case (GETPACKET):
    {
      ret = lidar_input_ptr_->getPacket(pkt_msg.packet.data(), timeout_);
      if (ret == INPUT_MSOP)
      {
        tmp_scan.packets.clear();
        preparePacketMsg(pkt_msg);
        scan_msg.packets.emplace_back(pkt_msg);
        tmp_scan.packets.emplace_back(pkt_msg);
        if (processMsopPackets(tmp_scan))
        {
          scan_seq_++;
          prepareLidarScanMsg(scan_msg);
          runCallBack(scan_msg);
          scan_msg.packets.clear();
        }
      }
      else if (ret == INPUT_DIFOP)
      {
        processDifopPackets(pkt_msg);
        preparePacketMsg(pkt_msg);
        runCallBack(pkt_msg);
      }
      else if (ret == INPUT_ERROR || ret == INPUT_EXIT || ret == INPUT_TIMEOUT)
      {
        reportError(ErrCode_LidarDriverInterrupt);
      }
      self_state_ = IDLE;
      break;
    }
    }
  }
}
void LidarBase::preparePacketMsg(common::LidarPacketMsg &msg)
{
  msg.timestamp = getTime();
  if (use_lidar_clock_ == true)
  {
    ST_MsopPkt *mpkt_ptr = (ST_MsopPkt *)msg.packet.data();
    std::tm stm;
    memset(&stm, 0, sizeof(stm));
    stm.tm_year = mpkt_ptr->header.timestamp.year + 100;
    stm.tm_mon = mpkt_ptr->header.timestamp.month - 1;
    stm.tm_mday = mpkt_ptr->header.timestamp.day;
    stm.tm_hour = mpkt_ptr->header.timestamp.hour;
    stm.tm_min = mpkt_ptr->header.timestamp.minute;
    stm.tm_sec = mpkt_ptr->header.timestamp.second;
    msg.timestamp = std::mktime(&stm) + (double)RS_SWAP_SHORT(mpkt_ptr->header.timestamp.ms) / 1000.0 + (double)RS_SWAP_SHORT(mpkt_ptr->header.timestamp.us) / 1000000.0;
  }
  msg.frame_id = frame_id_;
}
void LidarBase::prepareLidarScanMsg(common::LidarScanMsg &msg)
{
  msg.timestamp = getTime();
  if (use_lidar_clock_ == true)
  {
    ST_MsopPkt *mpkt_ptr = (ST_MsopPkt *)msg.packets[0].packet.data();
    std::tm stm;
    memset(&stm, 0, sizeof(stm));
    stm.tm_year = mpkt_ptr->header.timestamp.year + 100;
    stm.tm_mon = mpkt_ptr->header.timestamp.month - 1;
    stm.tm_mday = mpkt_ptr->header.timestamp.day;
    stm.tm_hour = mpkt_ptr->header.timestamp.hour;
    stm.tm_min = mpkt_ptr->header.timestamp.minute;
    stm.tm_sec = mpkt_ptr->header.timestamp.second;
    msg.timestamp = std::mktime(&stm) + (double)RS_SWAP_SHORT(mpkt_ptr->header.timestamp.ms) / 1000.0 + (double)RS_SWAP_SHORT(mpkt_ptr->header.timestamp.us) / 1000000.0;
  }
  msg.seq = scan_seq_;
  msg.parent_frame_id = frame_id_;
  msg.frame_id = frame_id_;
}
void LidarBase::preparePointsMsg(common::LidarPointsMsg &msg)
{
  msg.timestamp = getTime();
  msg.seq = points_seq_;
  msg.parent_frame_id = frame_id_;
  msg.frame_id = frame_id_;
  msg.height = msg.cloudPtr->height;
  msg.width = msg.cloudPtr->width;
  msg.is_dense = false;
  msg.is_transform = false;
  msg.lidar_model = lidar_model_;
  msg.points_type = "XYZI";
}

} // namespace sensor
} // namespace robosense