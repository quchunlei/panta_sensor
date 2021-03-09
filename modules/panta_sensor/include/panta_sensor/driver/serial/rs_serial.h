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
#include <panta_common/common.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <functional>
#include <memory>
namespace robosense
{
namespace sensor
{
class Serial
{
public:
  int openPort(const char *portname);
  void closePort();
  bool isOpen();
  int setPort(const int &nSpeed, const int &nBits, const char &nEvent, const int &nStop);
  int serialRead(std::vector<uint8_t> &buf, const size_t &nbytes, const float &timeout);
  bool connect(const std::string &portname, const int &nSpeed, const int &nBits, const char &nEvent, const int &nStop);
  bool autoConnect(const std::function<bool()> &f, const std::shared_ptr<Serial> &s, const int &baudrate);

private:
  enum State
  {
    IDLE = 0,
    READ,
    CHECK_RESULT,
  };

private:
  int fd_ = -1;
};
} // namespace serial
} // namespace robosense
