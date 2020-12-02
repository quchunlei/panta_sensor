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
#include "panta_sensor/driver/gnss/gnss_base.h"
namespace robosense
{
namespace sensor
{
using namespace robosense::common;
ErrCode GnssBase::init(const YAML::Node& config)
{
  setinitFlag(true);
  YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
  yamlRead<std::string>(driver_config, "device_type", parameter_.device_type, "");
  yamlRead<bool>(driver_config, "auto_scan_port", parameter_.auto_scan_port, true);
  yamlRead<std::string>(driver_config, "port_name", parameter_.port_name, "/dev/tyUSB0");
  yamlRead<int>(driver_config, "baudrate", parameter_.baudrate, 115200);
  yamlRead<std::string>(driver_config, "frame_id", parameter_.frame_id, "/gnss");

  yamlRead<bool>(driver_config, "do_reconnect", parameter_.do_reconnect, false);
  yamlRead<int>(driver_config, "reconnect_interval", parameter_.reconnect_interval, 3);
  yamlRead<int>(driver_config, "reconnect_attempts", parameter_.reconnect_attempts, 3);

  yamlRead<int>(driver_config, "min_satellite", parameter_.min_satellite, 3);
  yamlRead<double>(driver_config, "warning_gyro_z", parameter_.warning_gyro_z, 3.14);
  yamlRead<double>(driver_config, "timeout", parameter_.timeout, 0.1);
  yamlRead<bool>(driver_config, "use_gnss_clock", parameter_.use_gnss_clock, false);

  thread_flag_ = false;
  gnss_thread_ = std::make_shared<std::thread>();
  gnss_ser_ = std::make_shared<Serial>();
  gnss_seq_ = 0;
  odom_seq_ = 0;
  excb_ = NULL;
  gnsscb_.reserve(10);
  odomcb_.reserve(10);
  self_state_ = IDLE;
  return ErrCode_Success;
}

ErrCode GnssBase::start()
{
  if (thread_flag_ == false)
  {
    thread_flag_ = true;
    self_state_ = IDLE;
    const auto& func1 = [this] { stateMachine(); };
    gnss_thread_ = std::make_shared<std::thread>(func1);
#if (DEBUG_LEVEL > 1)
    INFO << "GnssBase START !" << REND;
#endif
  }
  return ErrCode_Success;
}

ErrCode GnssBase::stop()
{
  if (thread_flag_ == true)
  {
    thread_flag_ = false;
    gnss_thread_->join();
  }
#if (DEBUG_LEVEL > 1)
  INFO << "GnssBase Stop" << REND;
#endif
  return ErrCode_Success;
}

bool GnssBase::processData(std::vector<uint8_t>& data_buf, std::vector<std::string>& data)
{
  uint32_t i = 0;
  for (; i < data_buf.size(); i++)
  {
    if (data_buf[i] == '$')
    {
      break;
    }
  }
  uint32_t j = i;
  for (; j < data_buf.size(); j++)
  {
    if (data_buf[j] == '\n')
    {
      break;
    }
  }
  if (j == data_buf.size())
    return false;
  data_buf[j] = ',';
  for (uint32_t k = i; k <= j; ++k)
  {
    std::string tmp_str;
    while (data_buf[k] != ',')
    {
      tmp_str.push_back(data_buf[k]);
      ++k;
    }
    data.emplace_back(std::move(tmp_str));
  }
  if (j + 1 < data_buf.size())
  {
    data_buf.erase(data_buf.begin(), data_buf.begin() + j + 1);
  }
  else
  {
    data_buf.clear();
  }
  return true;
}

bool GnssBase::processGSOFData(std::vector<uint8_t>& data_buf, std::vector<std::string>& data)
{
  return true;
}
//   int tcStx;
//   int tcStat;
//   int tcType;
//   int tcLength;
//   int tcCsum;
//   int tcEtx;

//   unsigned char tcData[256];

//   int i;

//   while (1)

//   {
//     tcStx = gc();

//     if (tcStx == 0x02)

//     {
//       tcStat = gc();

//       tcType = gc();

//       tcLength = gc();

//       for (i = 0; i < tcLength; ++i)

//         tcData[i] = gc();

//       tcCsum = gc();

//       tcEtx = gc();

//       printf("STX:%02Xh Stat:%02Xh Type:%02Xh "

//              "Len:%d CS:%02Xh ETX:%02Xh\n",

//              tcStx,

//              tcStat,

//              tcType,

//              tcLength,

//              tcCsum,

//              tcEtx

//       );

//       if (tcType == 0x40)

//         postGsofData(tcData, tcLength);
//     }

//     else

//       printf("Skipping %02X\n", tcStx);
//   }
// }

// bool GnssBase::processGSOFData(std::vector<uint8_t>& data_buf, std::vector<std::string>& data)
// {
//   int i;
//   int gsofType;
//   int gsofLength;

//   unsigned char* pData;
//   printf("\nGSOF Records\n");

//   pData = gsofData;

//   while (pData < gsofData + gsofDataIndex)

//   {
//     gsofType = *pData++;

//     gsofLength = *pData++;

//     // If the type is one that we know about, then call the specific

//     // parser for that type.

//     if (gsofType == 1)

//     {
//       processPositionTime(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//         if (gsofType == 2)

//     {
//       processLatLonHeight(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//         if (gsofType == 3)

//     {
//       processECEF(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//         if (gsofType == 4)

//     {
//       processLocalDatum(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//         if (gsofType == 8)

//     {
//       processVelocityData(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//         if (gsofType == 9)

//     {
//       processPdopInfo(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//         if (gsofType == 13)

//     {
//       processBriefSVInfo(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//         if (gsofType == 16)

//     {
//       processUtcTime(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//         if (gsofType == 33)

//     {
//       processAllBriefSVInfo(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//         if (gsofType == 34)

//     {
//       processAllDetailedSVInfo(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//         if (gsofType == 14)

//     {
//       processSvDetailedInfo(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//         if (gsofType == 27)

//     {
//       processAttitudeInfo(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//         if (gsofType == 26)

//     {
//       processPositionTimeUtc(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//         if (gsofType == 6)

//     {
//       processEcefDelta(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//         if (gsofType == 7)

//     {
//       processTangentPlaneDelta(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//         if (gsofType == 40)

//     {
//       processLbandStatus(gsofLength, pData);

//       pData += gsofLength;
//     }

//     else

//     {
//       // Not a type we know about. Hex dump the bytes and move on.

//       printf(" GsofType:%d len:%d\n ",

//              gsofType,

//              gsofLength

//       );

//       for (i = 0; i < gsofLength; ++i)

//       {
//         printf("%02X%s",

//                *pData++,

//                i % 16 == 15 ? "\n " : " "

//         );
//       }

//       // Terminate the last line if needed.

//       if (gsofLength % 16 != 0)

//         printf("\n");
//     }

//     printf("\n");
//   }

//   printf("\n");
// }

void GnssBase::stateMachine()
{
  std::vector<uint8_t> data_buf;
  std::vector<std::string> data;
  int count = 0;
  while (thread_flag_)
  {
    switch (self_state_)
    {
      case IDLE: {
        self_state_ = CONNECT;
        break;
      }
      case CONNECT: {
        if (parameter_.auto_scan_port)
        {
          if (!autoConnect())
          {
#if (DEBUG_LEVEL > 0)
            WARNING << "GNSS AUTO_CONNECT failed" << REND;
#endif
            reportError(ErrCode_GnssDriverConnectfail);
            self_state_ = CHECK_CONNECTION;
          }
          else
          {
#if (DEBUG_LEVEL > 1)
            INFO << "GNSS Auto connect successfully!" << REND;
#endif
            count = 0;
            self_state_ = PREPARE_DATA;
          }
          break;
        }
        else
        {
          if (!gnss_ser_->connect(parameter_.port_name, parameter_.baudrate, 8, 'N', 1))
          {
            WARNING << "GNSS MANUAL_CONNECT failed" << REND;
            reportError(ErrCode_GnssDriverConnectfail);
            self_state_ = CHECK_CONNECTION;
          }
          else
          {
#if (DEBUG_LEVEL > 1)
            INFO << "GNSS Manually connect successfully!" << REND;
#endif
            count = 0;
            self_state_ = PREPARE_DATA;
          }
          break;
        }
      }
      case PREPARE_DATA: {
        std::vector<uint8_t> tmp_buf;

        if ((int)gnss_ser_->serialRead(tmp_buf, 50, parameter_.timeout) < 0)
        {
          reportError(ErrCode_GnssDriverInterrupt);
          self_state_ = CHECK_CONNECTION;
          break;
        }
        data_buf.insert(data_buf.begin() + data_buf.size(), tmp_buf.begin(), tmp_buf.begin() + tmp_buf.size());
        if (processData(data_buf, data))
        {
          self_state_ = READ_DATA;
        }
        else
        {
          self_state_ = PREPARE_DATA;
        }
        break;
      }
      case READ_DATA: {
        prepareMsg(data, parameter_.use_gnss_clock);
        data.clear();
        self_state_ = PREPARE_DATA;
        break;
      }
      case CHECK_CONNECTION: {
        if (parameter_.do_reconnect)
        {
          count++;
          if (count > parameter_.reconnect_attempts)
          {
            reportError(ErrCode_GnssDriverDisconnect);
            sleep(1);
            break;
          }
#if (DEBUG_LEVEL > 0)
          WARNING << "GNSS driver will start reconnect in " << parameter_.reconnect_interval << " seconds" << REND;
#endif
          sleep(parameter_.reconnect_interval);
          self_state_ = IDLE;
        }
        else
        {
          reportError(ErrCode_GnssDriverDisconnect);
          sleep(1);
          break;
        }
      }
    }
  }
}
}  // namespace sensor
}  // namespace robosense