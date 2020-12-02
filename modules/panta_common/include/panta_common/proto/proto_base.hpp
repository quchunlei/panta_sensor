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
#include "proto_base.h"
namespace robosense
{
namespace common
{

/**
   * @brief  intilize the socket sender   
   * @param  proto_send_port: destination port
   * @param  proto_send_ip: destination IP address
   * @retval 0: success -1: failed
   */
inline int ProtoBase::initSender(std::string proto_send_port, std::string proto_send_ip)
{
    send_sock_ptr_.reset(new udp::socket(io_service_, udp::endpoint(udp::v4(), 0)));
    boost::asio::socket_base::broadcast option(true);
    send_sock_ptr_->set_option(option);
    udp::resolver resolver(io_service_);
    udp::resolver::query query(udp::v4(), proto_send_ip, proto_send_port);
    iterator_ = resolver.resolve(query);
    return 0;
}
/**
 * @brief  initialize the socket receiver
 * @param  proto_recv_port: the receiver port number
 * @retval 0:success -1:failed
 */
inline int ProtoBase::initReceiver(uint16_t proto_recv_port)
{
    recv_sock_ptr_.reset(new udp::socket(io_service_, udp::endpoint(udp::v4(), proto_recv_port)));
    deadline_.reset(new deadline_timer(io_service_));
    deadline_->expires_at(boost::posix_time::pos_infin);
    check_deadline();
    return 0;
}
/**
 * @brief  the message send function
 * @note   serialize the message to protobuf and send it 
 * @param  *pMsgData: data
 * @param  &header: header
 * @retval >=0 : success -1: failed
 */
inline int ProtoBase::sendProtoMsg(const void *pMsgData, const proto_MsgHeader &header)
{
    int sendLen = header.msgLen + sizeof(proto_MsgHeader);
    char *sendBuffer = (char *)malloc(sendLen);
    memset(sendBuffer, 0, sendLen);
    header.toTargetEndianArray(sendBuffer, sizeof(proto_MsgHeader), RS_DATA_ENDIAN_TYPE::RS_DATA_BIG_ENDIAN);
    if (header.msgLen > 0)
    {
        memcpy(sendBuffer + sizeof(proto_MsgHeader), pMsgData, header.msgLen);
    }
    int ret = send_sock_ptr_->send_to(boost::asio::buffer(sendBuffer, sendLen), *iterator_);
    free(sendBuffer);
    return ret;
}
/**
 * @brief  message receive function
 * @note   receive the message and deserilize it
 * @param  *pMsgData: the output message
 * @param  msgMaxLen: max receive message length
 * @param  &header: output header
 * @retval >=0: success -1: failed
 */
inline int ProtoBase::receiveProtoMsg(void *pMsgData, const int msgMaxLen, proto_MsgHeader &header)
{
    deadline_->expires_from_now(boost::posix_time::seconds(1));
    boost::system::error_code ec = boost::asio::error::would_block;
    std::size_t ret = 0;
    char *pRecvBuffer = (char *)malloc(msgMaxLen + sizeof(proto_MsgHeader));
    recv_sock_ptr_->async_receive(boost::asio::buffer(pRecvBuffer, msgMaxLen + sizeof(proto_MsgHeader)),
                                  boost::bind(&ProtoBase::handle_receive, _1, _2, &ec, &ret));
    do
    {
        io_service_.run_one();
    } while (ec == boost::asio::error::would_block);
    if (ec)
    {
        return -1;
    }
    header.toHostEndianValue(pRecvBuffer, sizeof(proto_MsgHeader), RS_DATA_ENDIAN_TYPE::RS_DATA_BIG_ENDIAN);
    if (ret < (std::size_t)(header.msgLen + sizeof(proto_MsgHeader)))
    {
        free(pRecvBuffer);
        return -1;
    }
    if (header.msgLen > 0)
    {
        memcpy(pMsgData, pRecvBuffer + sizeof(proto_MsgHeader), header.msgLen);
    }
    free(pRecvBuffer);
    return ret;
}

} // namespace common
} // namespace robosense

#endif