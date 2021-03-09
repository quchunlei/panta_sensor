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

#ifdef PROTO_FOUND
#include "panta_sensor/proto/lidar_points_proto_adapter.h"
#define RECEIVE_BUF_SIZE 3000000
#define MAX_RECEIVE_LENGTH 5200
#define SPLIT_SIZE 5000
namespace robosense
{
namespace sensor
{
using namespace robosense::common;
ErrCode LidarPointsProtoAdapter::init(const YAML::Node& config)
{
  setName("LidarPointsProtoAdapter");
  setinitFlag(true);
  thread_flag_ = false;
  send_thread_flag_ = false;
  int msg_source = 0;
  bool send_points_proto;
  std::string points_send_port;
  uint16_t points_recv_port;
  std::string points_send_ip;
  YAML::Node proto_config = yamlSubNodeAbort(config, "proto");
  yamlRead<int>(config, "msg_source", msg_source);
  yamlRead<bool>(config, "send_points_proto", send_points_proto, false);
  yamlReadAbort<std::string>(proto_config, "points_send_port", points_send_port);
  yamlReadAbort<std::string>(proto_config, "points_send_ip", points_send_ip);
  yamlReadAbort<uint16_t>(proto_config, "points_recv_port", points_recv_port);

  if (msg_source == 5)
  {
    if (initReceiver(points_recv_port) == -1)
    {
      ERROR << "LidarPointsProtoAdapter: Create UDP Receiver Socket Failed OR Bind Network failed!" << REND;
      exit(-1);
    }
  }
  if (send_points_proto)
  {
    if (initSender(points_send_port, points_send_ip) == -1)
    {
      ERROR << "LidarPointsProtoAdapter: Create UDP Sender Socket Failed ! " << REND;
      exit(-1);
    }
    else
    {
      send_thread_flag_ = true;
      send_thread_.reset(new std::thread(std::bind(&LidarPointsProtoAdapter::send_points, this)));
    }
  }
  return ErrCode_Success;
}

ErrCode LidarPointsProtoAdapter::start()
{
  if (points_cb_.empty())
  {
    ERROR << "LidarPointsProtoAdapter: Please register at least one callback function first!" << REND;
    exit(-1);
  }
  if (thread_flag_ == false)
  {
    thread_flag_ = true;
    self_state_ = IDLE;
    const auto& func1 = [this] { stateMachine(); };
    points_thread_.reset(new std::thread(func1));
#if (DEBUG_LEVEL > 1)
    INFO << "LidarPointsProtoAdapter Start !" << REND;
#endif
  }
  return ErrCode_Success;
}
ErrCode LidarPointsProtoAdapter::stop()
{
  if (thread_flag_ == true)
  {
    thread_flag_ = false;
    points_thread_->join();
  }
  if (send_thread_flag_ == true)
  {
    send_thread_flag_ = false;
    send_cv_.notify_all();
    send_thread_->join();
  }
#if (DEBUG_LEVEL > 1)
  INFO << "LidarPointsProtoAdapter stop" << REND;
#endif
  return common::ErrCode_Success;
}

void LidarPointsProtoAdapter::send(const LidarPointsMsg& msg)  // Will send NavSatStatus and Odometry
{
  proto_msg_ = toProtoMsg(msg);
  send_cv_.notify_all();
}

void LidarPointsProtoAdapter::send_points()
{
  std::unique_lock<std::mutex> lk(send_mutex_);
  std::mutex tmp_lk;
  while (send_thread_flag_)
  {
    send_cv_.wait(lk);
    tmp_lk.lock();
    void* buf = malloc(proto_msg_.ByteSize() + SPLIT_SIZE);
    proto_msg_.SerializeToArray(buf, proto_msg_.ByteSize());
    proto_MsgHeader tmp_header;
    int pkt_count = ceil(1.0 * proto_msg_.ByteSize() / SPLIT_SIZE);
    tmp_header.frmNumber = proto_msg_.seq();
    tmp_header.msgLen = SPLIT_SIZE;
    tmp_header.totalMsgCnt = pkt_count;
    tmp_header.totalMsgLen = proto_msg_.ByteSize();
    for (int i = 0; i < pkt_count; i++)
    {
      tmp_header.msgID = i;
      void* tmp_buf = malloc(SPLIT_SIZE);
      memcpy(tmp_buf, (uint8_t*)buf + i * SPLIT_SIZE, SPLIT_SIZE);
      if (sendProtoMsg(tmp_buf, tmp_header) == -1)
      {
        // ERROR << "Points: Send Message Data Failed, Error: " << errno << REND;
        reportError(ErrCode_LidarPointsProtoSendError);
      }
      free(tmp_buf);
      usleep(30);
    }
    free(buf);
    tmp_lk.unlock();
  }
}

void LidarPointsProtoAdapter::stateMachine()
{
  int old_frmNum = 0;
  int new_frmNum = 0;
  proto_MsgHeader old_header;
  proto_MsgHeader new_header;
  void* buff = malloc(RECEIVE_BUF_SIZE);
  while (thread_flag_)
  {
    switch (self_state_)
    {
      case IDLE: {
        self_state_ = RECEIVE;
        break;
      }
      case RECEIVE: {
        void* pMsgData = malloc(MAX_RECEIVE_LENGTH);
        old_header = new_header;
        int ret = receiveProtoMsg(pMsgData, MAX_RECEIVE_LENGTH, new_header);
        if (ret == -1)
        {
          reportError(ErrCode_LidarPointsProtoReceiveError);
          self_state_ = IDLE;
          break;
        }
        old_frmNum = old_header.frmNumber;
        new_frmNum = new_header.frmNumber;
        if (old_frmNum == new_frmNum)
        {
          memcpy((uint8_t*)buff + new_header.msgID * SPLIT_SIZE, pMsgData, SPLIT_SIZE);
          if (new_header.msgID == new_header.totalMsgCnt - 1)
          {
            self_state_ = SENDMSG;
          }
          else
          {
            self_state_ = IDLE;
          }
        }
        else
        {
          memcpy((uint8_t*)buff + new_header.msgID * SPLIT_SIZE, pMsgData, SPLIT_SIZE);
          self_state_ = IDLE;
        }
        free(pMsgData);
        break;
      }
      case SENDMSG: {
        Proto_msg::LidarPoints proto_msg;
        proto_msg.ParseFromArray(buff, new_header.totalMsgLen);
        localCallback(toRsMsg(proto_msg));
        self_state_ = IDLE;
        break;
      }
    }
  }
  free(buff);
}
}  // namespace sensor
}  // namespace robosense
#endif  // ROS_FOUND