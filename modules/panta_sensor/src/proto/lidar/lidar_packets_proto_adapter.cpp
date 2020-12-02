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
#include "panta_sensor/proto/lidar_packets_proto_adapter.h"
#define RECEIVE_BUF_SIZE 500000
#define MAX_RECEIVE_LENGTH 3200
#define SPLIT_SIZE 3000
namespace robosense
{
namespace sensor
{
using namespace robosense::common;
ErrCode LidarPacketsProtoAdapter::init(const YAML::Node& config)
{
  setName("LidarPacketsProtoAdapter");
  setinitFlag(true);
  thread_flag_ = false;
  send_thread_flag_ = false;
  bool send_packets_proto;
  int msg_source = 0;
  std::string packets_send_ip;
  std::string msop_send_port;
  std::string difop_send_port;
  uint16_t msop_recv_port;
  uint16_t difop_recv_port;
  YAML::Node proto_config = yamlSubNodeAbort(config, "proto");
  yamlRead<int>(config, "msg_source", msg_source);
  yamlRead<bool>(config, "send_packets_proto", send_packets_proto, false);
  yamlReadAbort<std::string>(proto_config, "packets_send_ip", packets_send_ip);
  yamlReadAbort<std::string>(proto_config, "msop_send_port", msop_send_port);
  yamlReadAbort<std::string>(proto_config, "difop_send_port", difop_send_port);
  yamlReadAbort<uint16_t>(proto_config, "msop_recv_port", msop_recv_port);
  yamlReadAbort<uint16_t>(proto_config, "difop_recv_port", difop_recv_port);
  msop_proto_ptr_.reset(new common::ProtoBase);
  difop_proto_ptr_.reset(new common::ProtoBase);
  if (msg_source == 4)
  {
    if ((msop_proto_ptr_->initReceiver(msop_recv_port) == -1) ||
        (difop_proto_ptr_->initReceiver(difop_recv_port) == -1))
    {
      ERROR << "LidarPacketsReceiver: Create UDP Receiver Socket Failed OR Bind Network failed!" << REND;
      exit(-1);
    }
  }
  if (send_packets_proto)
  {
    if ((msop_proto_ptr_->initSender(msop_send_port, packets_send_ip) == -1) ||
        (difop_proto_ptr_->initSender(difop_send_port, packets_send_ip) == -1))
    {
      ERROR << "LidarPacketsReceiver: Create UDP Sender Socket Failed ! " << REND;
      exit(-1);
    }
    else
    {
      send_thread_flag_ = true;
      msop_send_thread_.reset(new std::thread(std::bind(&LidarPacketsProtoAdapter::send_msop_packets, this)));
    }
  }
  return ErrCode_Success;
}

ErrCode LidarPacketsProtoAdapter::start()
{
  if ((difop_cb_.empty()) && (msop_cb_.empty()))
  {
    ERROR << "LidarPacketProtoAdapter: Please register at least one callback function first!" << REND;
    exit(-1);
  }
  if (thread_flag_ == false)
  {
    thread_flag_ = true;
    msop_state_ = IDLE;
    difop_state_ = IDLE;
    const auto& func1 = [this] { msopStateMachine(); };
    const auto& func2 = [this] { difopStateMachine(); };
    msop_recv_thread_.reset(new std::thread(func1));
    difop_recv_thread_.reset(new std::thread(func2));

#if (DEBUG_LEVEL > 1)
    INFO << "LidarPacketProtoAdapter Start !" << REND;
#endif
  }
  return ErrCode_Success;
}
ErrCode LidarPacketsProtoAdapter::stop()
{
  if (thread_flag_ == true)
  {
    thread_flag_ = false;
    msop_recv_thread_->join();
    difop_recv_thread_->join();
  }
  if (send_thread_flag_ == true)
  {
    send_thread_flag_ = false;
    msop_send_cv_.notify_all();
    msop_send_thread_->join();
  }
#if (DEBUG_LEVEL > 1)
  INFO << "LidarPacketProtoAdapter stop" << REND;
#endif
  return common::ErrCode_Success;
}

void LidarPacketsProtoAdapter::send_msop(const LidarScanMsg& msg)  // Will send NavSatStatus and Odometry
{
  proto_msop_msg_ = toProtoMsg(msg);
  msop_send_cv_.notify_all();
}

void LidarPacketsProtoAdapter::send_difop(const LidarPacketMsg& msg)  // Will send NavSatStatus and Odometry
{
  Proto_msg::LidarPacket proto_msg = toProtoMsg(msg);
  proto_MsgHeader proto_header;
  proto_header.msgLen = proto_msg.ByteSize();
  void* buff = malloc(proto_header.msgLen);
  proto_msg.SerializeToArray(buff, proto_header.msgLen);
  if (difop_proto_ptr_->sendProtoMsg(buff, proto_header) == -1)
  {
    // ERROR << "LidarPacketsProtoAdapter: Send Message Data Failed, Error: " << errno << REND;
    reportError(ErrCode_LidarPacketsProtoSendError);
  }
  free(buff);
}

void LidarPacketsProtoAdapter::send_msop_packets()
{
  std::unique_lock<std::mutex> lk(msop_send_mutex_);
  std::mutex tmp_lk;
  while (send_thread_flag_)
  {
    msop_send_cv_.wait(lk);
    tmp_lk.lock();
    void* buf = malloc(proto_msop_msg_.ByteSize() + SPLIT_SIZE);
    proto_msop_msg_.SerializeToArray(buf, proto_msop_msg_.ByteSize());
    proto_MsgHeader tmp_header;
    int pkt_count = ceil(1.0 * proto_msop_msg_.ByteSize() / SPLIT_SIZE);
    tmp_header.frmNumber = proto_msop_msg_.seq();
    tmp_header.msgLen = SPLIT_SIZE;
    tmp_header.totalMsgCnt = pkt_count;
    tmp_header.totalMsgLen = proto_msop_msg_.ByteSize();
    for (int i = 0; i < pkt_count; i++)
    {
      tmp_header.msgID = i;
      void* tmp_buf = malloc(SPLIT_SIZE);
      memcpy(tmp_buf, (uint8_t*)buf + i * SPLIT_SIZE, SPLIT_SIZE);
      if (msop_proto_ptr_->sendProtoMsg(tmp_buf, tmp_header) == -1)
      {
        //         //ERROR << "Points: Send Message Data Failed, Error: " << errno << REND;
        reportError(ErrCode_LidarPacketsProtoSendError);
      }
      free(tmp_buf);
      usleep(30);
    }
    free(buf);
    tmp_lk.unlock();
  }
}

void LidarPacketsProtoAdapter::msopStateMachine()
{
  int old_frmNum = 0;
  int new_frmNum = 0;
  proto_MsgHeader old_header;
  proto_MsgHeader new_header;
  void* buff = malloc(RECEIVE_BUF_SIZE);
  while (thread_flag_)
  {
    switch (msop_state_)
    {
      case IDLE: {
        msop_state_ = RECEIVE;
        break;
      }
      case RECEIVE: {
        void* pMsgData = malloc(MAX_RECEIVE_LENGTH);
        old_header = new_header;
        int ret = msop_proto_ptr_->receiveProtoMsg(pMsgData, MAX_RECEIVE_LENGTH, new_header);
        if (ret == -1)
        {
          reportError(ErrCode_LidarPacketsProtoReceiveError);
          msop_state_ = IDLE;
          break;
        }
        old_frmNum = old_header.frmNumber;
        new_frmNum = new_header.frmNumber;
        if (old_frmNum == new_frmNum)
        {
          memcpy((uint8_t*)buff + new_header.msgID * SPLIT_SIZE, pMsgData, SPLIT_SIZE);
          if (new_header.msgID == new_header.totalMsgCnt - 1)
          {
            msop_state_ = SENDMSG;
          }
          else
          {
            msop_state_ = IDLE;
          }
        }
        else
        {
          memcpy((uint8_t*)buff + new_header.msgID * SPLIT_SIZE, pMsgData, SPLIT_SIZE);
          msop_state_ = IDLE;
        }
        free(pMsgData);
        break;
      }
      case SENDMSG: {
        Proto_msg::LidarScan proto_msg;
        proto_msg.ParseFromArray(buff, new_header.totalMsgLen);
        localMsopCallback(toRsMsg(proto_msg));
        msop_state_ = IDLE;
        break;
      }
    }
  }
  free(buff);
}

void LidarPacketsProtoAdapter::difopStateMachine()
{
  LidarPacketMsg rs_msg;
  while (thread_flag_)
  {
    switch (difop_state_)
    {
      case IDLE: {
        difop_state_ = RECEIVE;
        break;
      }
      case RECEIVE: {
        void* pMsgData = malloc(MAX_RECEIVE_LENGTH);
        proto_MsgHeader header;
        int ret = difop_proto_ptr_->receiveProtoMsg(pMsgData, MAX_RECEIVE_LENGTH, header);
        if (ret == -1)
        {
          // ERROR << "ProtoBase: Receive Message Data Failed or not complete  Error: " << errno << " !" << REND;
          reportError(ErrCode_LidarPacketsProtoReceiveError);
          difop_state_ = IDLE;
          break;
        }
        Proto_msg::LidarPacket protomsg;
        protomsg.ParseFromArray(pMsgData, header.msgLen);
        rs_msg = toRsMsg(protomsg);
        free(pMsgData);
        difop_state_ = SENDMSG;
        break;
      }
      case SENDMSG: {
        localDifopCallback(rs_msg);
        difop_state_ = IDLE;
        break;
      }
    }
  }
}

}  // namespace sensor
}  // namespace robosense
#endif  // ROS_FOUND