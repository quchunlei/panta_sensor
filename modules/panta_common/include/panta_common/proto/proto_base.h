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
#ifdef PROTO_FOUND
#include <panta_common/common.h>
#include <string>
#include <iostream>
#include <memory.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace robosense
{
namespace common
{
enum class RS_DATA_ENDIAN_TYPE
{
  RS_DATA_BIG_ENDIAN = 0,
  RS_DATA_LITTLE_ENDIAN = 1,
};
template <typename T>
class CRSEndian
{
public:
  typedef typename std::shared_ptr<CRSEndian> Ptr;
  typedef typename std::shared_ptr<const CRSEndian> ConstPtr;

public:
  CRSEndian()
  {
    unsigned char *pChar = (unsigned char *)(&magicData);
    if (*pChar == magicBigEndian[0] && (*(pChar + 1)) == magicBigEndian[1] && (*(pChar + 2)) == magicBigEndian[2] & (*(pChar + 3)) == magicBigEndian[3])
    {
      m_hostEndianType = RS_DATA_ENDIAN_TYPE::RS_DATA_BIG_ENDIAN;
    }
    else if (*pChar == magicLittleEndian[0] && (*(pChar + 1)) == magicLittleEndian[1] && (*(pChar + 2)) == magicLittleEndian[2] && (*(pChar + 3)) == magicLittleEndian[3])
    {
      m_hostEndianType = RS_DATA_ENDIAN_TYPE::RS_DATA_LITTLE_ENDIAN;
    }
  }

public:
  RS_DATA_ENDIAN_TYPE getHostEndian()
  {
    return m_hostEndianType;
  }

  int toTargetEndianArray(T t, void *pArray, unsigned int maxSize, RS_DATA_ENDIAN_TYPE dstEndianType)
  {
    unsigned int tSize = sizeof(T);
    assert(tSize <= maxSize);
    assert(pArray != nullptr);
    if (m_hostEndianType != dstEndianType)
    {
      char *pData = (char *)(&t);
      unsigned int tSize_h = tSize / 2;
      for (unsigned int idx = 0; idx < tSize_h; ++idx)
      {
        char tmp = pData[idx];
        unsigned int swapIdx = tSize - idx - 1;
        pData[idx] = pData[swapIdx];
        pData[swapIdx] = tmp;
      }
    }
    memcpy(pArray, &t, tSize);
    return 0;
  }

  int toHostEndianValue(T &t, const void *pArray, unsigned int maxSize, RS_DATA_ENDIAN_TYPE srcEndianType)
  {
    unsigned int tSize = sizeof(T);
    assert(pArray != nullptr);
    assert(tSize <= maxSize);
    if (srcEndianType != m_hostEndianType)
    {
      char *pData = (char *)(pArray);
      unsigned int tSize_h = tSize / 2;
      for (unsigned idx = 0; idx < tSize_h; ++idx)
      {
        char tmp = pData[idx];
        unsigned int swapIdx = tSize - idx - 1;
        pData[idx] = pData[swapIdx];
        pData[swapIdx] = tmp;
      }
    }
    memcpy(&t, pArray, tSize);
    return 0;
  }

private:
  RS_DATA_ENDIAN_TYPE m_hostEndianType;

private:
  const unsigned int magicData = 0xA1B2C3D4;
  const unsigned char magicLittleEndian[4] = {(unsigned char)0xD4, (unsigned char)0xC3, (unsigned char)0xB2, (unsigned char)0xA1};
  const unsigned char magicBigEndian[4] = {(unsigned char)0xA1, (unsigned char)0xB2, (unsigned char)0xC3, (unsigned char)0xD4};
};

struct alignas(16) proto_MsgHeader
{
  unsigned int msgType;          // 消息类型: 0: obstacle, 1: freespace, 99: keepalive
  unsigned int frmNumber;        // 消息帧编号
  unsigned int totalMsgCnt;      // 消息帧中的总消息个数(针对一帧数据量较大)
  unsigned int msgID;            // 消息序列中的编号
  unsigned int msgLen;           // 每个消息的长度
  unsigned int deviceNum;        // 设备编号
  std::time_t deviceTimeStampMs; // 时间戳(s): 如果设备包含GPS，则时间为GPS授时时间, 否则，取值为设备系统时间(仅仅用于参考)
  unsigned int flags;            // 最低位取值为1表示设备时间戳为GPS时间, 取值为0表示为设备系统时间
  unsigned int totalMsgLen;      // 消息帧中的总消息长度(针对一帧数据量较大)
  unsigned int res[2];           // 填充

  proto_MsgHeader()
  {
    msgType = 99;
    frmNumber = 0;
    totalMsgCnt = 0;
    msgID = 0;
    msgLen = 0;
    deviceNum = 0;
    deviceTimeStampMs = 0;
    flags = 0;
    totalMsgLen = 0;
    memset(res, 0, sizeof(unsigned int) * 2);
  }
  bool toTargetEndianArray(char *pArray, const unsigned int maxArraySize, const RS_DATA_ENDIAN_TYPE dstEndianType = RS_DATA_ENDIAN_TYPE::RS_DATA_BIG_ENDIAN) const
  {
    if (pArray == nullptr || maxArraySize < sizeof(proto_MsgHeader))
    {
      return false;
    }

    CRSEndian<unsigned int> uint32Endian;
    CRSEndian<std::time_t> uint64Endian;

    int offset = 0;
    uint32Endian.toTargetEndianArray(msgType, pArray + offset, maxArraySize - offset, dstEndianType);

    offset += sizeof(unsigned int);
    uint32Endian.toTargetEndianArray(frmNumber, pArray + offset, maxArraySize - offset, dstEndianType);

    offset += sizeof(unsigned int);
    uint32Endian.toTargetEndianArray(totalMsgCnt, pArray + offset, maxArraySize - offset, dstEndianType);

    offset += sizeof(unsigned int);
    uint32Endian.toTargetEndianArray(msgID, pArray + offset, maxArraySize - offset, dstEndianType);

    offset += sizeof(unsigned int);
    uint32Endian.toTargetEndianArray(msgLen, pArray + offset, maxArraySize - offset, dstEndianType);

    offset += sizeof(unsigned int);
    uint32Endian.toTargetEndianArray(deviceNum, pArray + offset, maxArraySize - offset, dstEndianType);

    offset += sizeof(unsigned int);
    uint64Endian.toTargetEndianArray(deviceTimeStampMs, pArray + offset, maxArraySize - offset, dstEndianType);

    offset += sizeof(std::time_t);
    uint32Endian.toTargetEndianArray(flags, pArray + offset, maxArraySize - offset, dstEndianType);

    offset += sizeof(unsigned int);
    uint32Endian.toTargetEndianArray(totalMsgLen, pArray + offset, maxArraySize - offset, dstEndianType);
    // res: don't care
    offset += sizeof(unsigned int);
    memcpy(pArray + offset, &res, sizeof(unsigned int) * 2);

    return true;
  }
  bool toHostEndianValue(const char *pArray, const unsigned int arraySize, const RS_DATA_ENDIAN_TYPE srcEndianType = RS_DATA_ENDIAN_TYPE::RS_DATA_BIG_ENDIAN)
  {
    if (pArray == nullptr || arraySize < sizeof(proto_MsgHeader))
    {
      return false;
    }
    CRSEndian<unsigned int> uint32Endian;
    CRSEndian<std::time_t> uint64Endian;

    int offset = 0;
    uint32Endian.toHostEndianValue(msgType, pArray + offset, arraySize - offset, srcEndianType);

    offset += sizeof(unsigned int);
    uint32Endian.toHostEndianValue(frmNumber, pArray + offset, arraySize - offset, srcEndianType);

    offset += sizeof(unsigned int);
    uint32Endian.toHostEndianValue(totalMsgCnt, pArray + offset, arraySize - offset, srcEndianType);

    offset += sizeof(unsigned int);
    uint32Endian.toHostEndianValue(msgID, pArray + offset, arraySize - offset, srcEndianType);

    offset += sizeof(unsigned int);
    uint32Endian.toHostEndianValue(msgLen, pArray + offset, arraySize - offset, srcEndianType);

    offset += sizeof(unsigned int);
    uint32Endian.toHostEndianValue(deviceNum, pArray + offset, arraySize - offset, srcEndianType);

    offset += sizeof(unsigned int);
    uint64Endian.toHostEndianValue(deviceTimeStampMs, pArray + offset, arraySize - offset, srcEndianType);

    offset += sizeof(std::time_t);
    uint32Endian.toHostEndianValue(flags, pArray + offset, arraySize - offset, srcEndianType);

    offset += sizeof(unsigned int);
    uint32Endian.toHostEndianValue(totalMsgLen, pArray + offset, arraySize - offset, srcEndianType);
    memset(&res, 0, sizeof(unsigned int) * 2);
    return true;
  }
};
using boost::asio::deadline_timer;
using boost::asio::ip::udp;
/**
 * @brief  Protobuf UDP transmission basic class
 * @note   used to initialize socket sender and receiver, and define send function and receive function
 */
class ProtoBase : virtual public common::CommonBase
{
public:
  ProtoBase() = default;
  ~ProtoBase() = default;

  /* Sender & Receiver initialize function */
  int initSender(std::string proto_send_port, std::string proto_send_ip);
  int initReceiver(uint16_t proto_recv_port);

  /* message send function & message receive function */
  int sendProtoMsg(const void *pMsgData, const proto_MsgHeader &header);
  int receiveProtoMsg(void *pMsgData, const int msgMaxLen, proto_MsgHeader &header);

private:
  inline void check_deadline()
  {
    if (deadline_->expires_at() <= deadline_timer::traits_type::now())
    {
      recv_sock_ptr_->cancel();
      deadline_->expires_at(boost::posix_time::pos_infin);
    }
    deadline_->async_wait(boost::bind(&ProtoBase::check_deadline, this));
  }
  static void handle_receive(
      const boost::system::error_code &ec, std::size_t length,
      boost::system::error_code *out_ec, std::size_t *out_length)
  {
    *out_ec = ec;
    *out_length = length;
  }
private:
  /* UDP socket related varibles */
  boost::asio::io_service io_service_;
  udp::resolver::iterator iterator_;
  std::unique_ptr<udp::socket> send_sock_ptr_;
  std::unique_ptr<udp::socket> recv_sock_ptr_;
  std::unique_ptr<deadline_timer> deadline_;
};

} // namespace common
} // namespace robosense
#endif // PROTO_FOUND