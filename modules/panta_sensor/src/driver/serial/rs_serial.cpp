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

#include "panta_sensor/driver/serial/rs_serial.h"

namespace robosense
{
namespace sensor
{
bool Serial::connect(const std::string& portname, const int& nSpeed, const int& nBits, const char& nEvent,
                     const int& nStop)
{
#if (DEBUG_LEVEL > 1)
  INFO << "[Start connect to port " << portname << "]" << REND;
#endif
  this->openPort((char*)portname.c_str());
  if (this->isOpen())
  {
    this->setPort(nSpeed, nBits, nEvent, nStop);
    return true;
  }
  return false;
}

bool Serial::autoConnect(const std::function<bool()>& f, const std::shared_ptr<Serial>& s, const int& baudrate)
{
#if (DEBUG_LEVEL > 1)
  INFO << "[Start auto connect]" << REND;
#endif
  std::vector<std::string> v_port;
  for (uint32_t i = 0; i < 10; i++)
  {
    v_port.emplace_back("/dev/ttyUSB" + std::to_string(i));
  }
  for (uint32_t i = 0; i < 10; i++)
  {
    v_port.emplace_back("/dev/ttyACM" + std::to_string(i));
  }
  for (uint32_t i = 0; i < v_port.size(); i++)
  {
    if (s->connect(v_port[i].c_str(), baudrate, 8, 'N', 1))  // future: use parameter to set the stop bit and check bit
    {
      if (f())
        return true;
    }
    if (s->isOpen())
      s->closePort();
  }
  return false;
}

int Serial::setPort(const int& nSpeed, const int& nBits, const char& nEvent, const int& nStop)
{
  struct termios rs_tio;
  bzero(&rs_tio, sizeof(rs_tio));
  rs_tio.c_cflag |= CLOCAL | CREAD;
  rs_tio.c_cflag &= ~CSIZE;
  switch (nBits)
  {
    case 7:
      rs_tio.c_cflag |= CS7;
      break;
    case 8:
      rs_tio.c_cflag |= CS8;
      break;
  }
  switch (nEvent)
  {
    case 'o':
    case 'O':
      rs_tio.c_cflag |= PARENB;
      rs_tio.c_cflag |= PARODD;
      rs_tio.c_iflag |= (INPCK | ISTRIP);
      break;
    case 'e':
    case 'E':
      rs_tio.c_iflag |= (INPCK | ISTRIP);
      rs_tio.c_cflag |= PARENB;
      rs_tio.c_cflag &= ~PARODD;
      break;
    case 'n':
    case 'N':
      rs_tio.c_cflag &= ~PARENB;
      break;
    default:
      break;
  }

  switch (nSpeed)
  {
    case 2400:
      cfsetispeed(&rs_tio, B2400);
      cfsetospeed(&rs_tio, B2400);
      break;
    case 4800:
      cfsetispeed(&rs_tio, B4800);
      cfsetospeed(&rs_tio, B4800);
      break;
    case 9600:
      cfsetispeed(&rs_tio, B9600);
      cfsetospeed(&rs_tio, B9600);
      break;
    case 38400:
      cfsetispeed(&rs_tio, B38400);
      cfsetospeed(&rs_tio, B38400);
      break;
    case 115200:
      cfsetispeed(&rs_tio, B115200);
      cfsetospeed(&rs_tio, B115200);
      break;
    case 230400:
      cfsetispeed(&rs_tio, B230400);
      cfsetospeed(&rs_tio, B230400);
      break;
    case 460800:
      cfsetispeed(&rs_tio, B460800);
      cfsetospeed(&rs_tio, B460800);
      break;
    default:
      cfsetispeed(&rs_tio, B9600);
      cfsetospeed(&rs_tio, B9600);
      break;
  }

  if (nStop == 1)
    rs_tio.c_cflag &= ~CSTOPB;
  else if (nStop == 2)
    rs_tio.c_cflag |= CSTOPB;

  rs_tio.c_cc[VTIME] = 1;
  rs_tio.c_cc[VMIN] = 0;

  tcflush(fd_, TCIFLUSH);

  if ((tcsetattr(fd_, TCSANOW, &rs_tio)) != 0)
  {
    return -1;
  }
  return 0;
}

int Serial::openPort(const char* portname)
{
  fd_ = open(portname, O_RDONLY | O_NOCTTY);

  return fd_;
}

bool Serial::isOpen()
{
  if (fd_ == -1)
    return 0;
  else
    return 1;
}

void Serial::closePort()
{
  close(fd_);
  fd_ = -1;
}

int Serial::serialRead(std::vector<uint8_t>& buf, const size_t& nbytes, const float& timeout)
{
  buf.reserve(nbytes);
  if (timeout == 0)
  {
    return read(fd_, buf.data(), nbytes);
  }

  std::vector<uint8_t> tmp(nbytes);
  auto return_time = std::chrono::system_clock::now();
  int count = 0;
  State self_state = READ;

  while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - return_time).count() <
         timeout * 1000)
  {
    switch (self_state)
    {
      case (READ): {
        int i = read(fd_, tmp.data(), nbytes - count);
        count = count + i;

        if (i != 0)
          buf.insert(buf.begin() + buf.size(), tmp.begin(), tmp.begin() + i);

        self_state = CHECK_RESULT;
        break;
      }

      case (CHECK_RESULT): {
        if (buf.size() >= nbytes)
        {
          return buf.size();
        }
        else
        {
          self_state = READ;
          break;
        }
      }
      default:
        break;
    }
  }

  return -1;
}
}  // namespace sensor
}  // namespace robosense