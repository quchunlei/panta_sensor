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
#include "panta_sensor/proto/imu_proto_adapter.h"
#define MAX_REVEIVE_LEN 1024

namespace robosense
{
namespace sensor
{
using namespace robosense::common;
ErrCode ImuProtoAdapter::init(const YAML::Node& config)
{
  setName("ImuProtoAdapter");
  setinitFlag(true);
  thread_flag_ = false;
  int msg_source = 0;
  bool send_msg_proto;
  std::string proto_send_port;
  uint16_t proto_recv_port;
  std::string proto_send_ip;
  YAML::Node proto_config = yamlSubNodeAbort(config, "proto");
  YAML::Node common_config = yamlSubNodeAbort(config, "common");
  yamlRead<int>(common_config, "msg_source", msg_source);
  yamlRead<bool>(common_config, "send_msg_proto", send_msg_proto, false);
  yamlReadAbort<std::string>(proto_config, "proto_send_port", proto_send_port);
  yamlReadAbort<std::string>(proto_config, "proto_send_ip", proto_send_ip);
  yamlReadAbort<uint16_t>(proto_config, "proto_recv_port", proto_recv_port);
  if (msg_source == 3)
  {
    if (initReceiver(proto_recv_port) == -1)
    {
      ERROR << "ProtoBase: Create UDP Receiver Socket Failed OR Bind Network failed!  Abort!" << REND;
      exit(-1);
    }
  }

  if (send_msg_proto)
  {
    if (initSender(proto_send_port, proto_send_ip) == -1)
    {
      ERROR << "ImuProtoAdapter: Create UDP Sender Socket Failed ! Abort!" << REND;
      exit(-1);
    }
  }

  return ErrCode_Success;
}
ErrCode ImuProtoAdapter::start()
{
  if (imu_cb_.empty())
  {
    ERROR << "ImuProtoAdapter: Please register at least one callback function first!" << REND;
    exit(-1);
  }
  if (thread_flag_ == false)
  {
    thread_flag_ = true;
    self_state_ = IDLE;
    const auto& func1 = [this] { stateMachine(); };
    imu_thread_.reset(new std::thread(func1));
#if (DEBUG_LEVEL > 1)
    INFO << "ImuProtoAdapter Start !" << REND;
#endif
  }
  return ErrCode_Success;
}
ErrCode ImuProtoAdapter::stop()
{
  if (thread_flag_ == true)
  {
    thread_flag_ = false;
    imu_thread_->join();
  }
#if (DEBUG_LEVEL > 1)
  INFO << "ImuProtoAdapter stop" << REND;
#endif
  return common::ErrCode_Success;
}

void ImuProtoAdapter::send(const ImuMsg& msg)  // Will send NavSatStatus and Odometry
{
  Proto_msg::Imu proto_msg = toProtoMsg(msg);
  proto_MsgHeader proto_header;
  proto_header.msgLen = proto_msg.ByteSize();
  proto_header.frmNumber = msg.seq;
  void* buff = malloc(proto_header.msgLen);
  proto_msg.SerializeToArray(buff, proto_header.msgLen);
  if (sendProtoMsg(buff, proto_header) == -1)
  {
    // ERROR << "ImuProtoAdapter: Send Message Data Failed, Error: " << errno << REND;
    reportError(ErrCode_ImuProtoSendError);
  }
  free(buff);
}

void ImuProtoAdapter::stateMachine()
{
  ImuMsg rs_msg;
  while (thread_flag_)
  {
    switch (self_state_)
    {
      case IDLE: {
        self_state_ = RECEIVE;
        break;
      }
      case RECEIVE: {
        void* pMsgData = malloc(MAX_REVEIVE_LEN);
        proto_MsgHeader header;
        int ret = receiveProtoMsg(pMsgData, MAX_REVEIVE_LEN, header);
        if (ret == -1)
        {
          // ERROR << "ProtoBase: Receive Message Data Failed or not complete  Error: " << errno << " !" << REND;
          reportError(ErrCode_ImuProtoReceiveError);
          self_state_ = IDLE;
          break;
        }
        Proto_msg::Imu protomsg;
        protomsg.ParseFromArray(pMsgData, header.msgLen);
        rs_msg = toRsMsg(protomsg);
        free(pMsgData);
        self_state_ = SENDMSG;
        break;
      }
      case SENDMSG: {
        localCallback(rs_msg);
        self_state_ = IDLE;
        break;
      }
    }
  }
}
}  // namespace sensor
}  // namespace robosense
#endif  // ROS_FOUND