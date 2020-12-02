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

#include <cstdint>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>

namespace robosense
{
namespace sensor
{
#define RS_POINTS_CHANNEL_PER_SECOND (20000)
#define RS_BLOCKS_CHANNEL_PER_PKT (12)

#define RS_CHANNELS_PER_BLOCK 32
#define RS_BLOCKS_PER_PKT 12

#define RS_TEMPERATURE_CNT 51

#define RS_LENGTH_PACKET 1248
#define RS_FIRST_BYTE_PKT 0x55

#define RS_SWAP_SHORT(x) ((((x)&0xFF) << 8) | (((x)&0xFF00) >> 8))
#define RS_SWAP_LONG(x) ((((x)&0xFF) << 24) | (((x)&0xFF00) << 8) | (((x)&0xFF0000) >> 8) | (((x)&0xFF000000) >> 24))
#define RS_TO_RADS(x) ((x) * (M_PI) / 180)

#define RS_MSOP_SYNC (0xA050A55A0A05AA55)
#define RS_BLOCK_ID (0xEEFF)
#define RS_DIFOP_SYNC (0x555511115A00FFA5)

#define RS_CHANNEL_TOFFSET (3)
#define RS_FIRING_TDURATION (50)
#define RS16_BLOCK_TDURATION_DUAL (50)
#define RS16_BLOCK_TDURATION_SINGLE (100)

#define RS_RESOLUTION_5mm_DISTANCE_COEF (0.005)
#define RS_RESOLUTION_10mm_DISTANCE_COEF (0.01)

#define RS_TEMPERATURE_MIN 31
#define RS_TEMPERATURE_MAX 81

typedef struct
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t ms;
    uint16_t us;
} __attribute__((packed)) ST_Timestamp;

typedef struct
{
    uint64_t sync;
    uint8_t reserved1[12];
    ST_Timestamp timestamp;
    uint8_t lidar_type;
    uint8_t reserved2[7];
    uint16_t temp_raw;
    uint8_t reserved3[2];
} __attribute__((packed)) ST_MsopHeader;

typedef struct
{
    uint16_t distance;
    uint8_t intensity;
} __attribute__((packed)) ST_Channel;

typedef struct
{
    uint16_t id;
    uint16_t azimuth;
    ST_Channel channels[RS_CHANNELS_PER_BLOCK];
} __attribute__((packed)) ST_MsopBlock;

typedef struct
{
    ST_MsopHeader header;
    ST_MsopBlock blocks[RS_BLOCKS_PER_PKT];
    uint32_t index;
    uint16_t tail;
} __attribute__((packed)) ST_MsopPkt;

typedef struct
{
    uint8_t lidar_ip[4];
    uint8_t host_ip[4];
    uint8_t mac_addr[6];
    uint16_t local_port;
    uint16_t dest_port;
    uint16_t port3;
    uint16_t port4;
} __attribute__((packed)) ST_EthNet;

typedef struct
{
    uint16_t start_angle;
    uint16_t end_angle;
} __attribute__((packed)) ST_FOV;

typedef struct
{
    uint8_t main_sn[5];
    uint8_t bottom_sn[5];
} __attribute__((packed)) ST_Version;

typedef struct
{
    uint8_t intensity_cali[240];
    uint8_t coef;
    uint8_t ver;
} __attribute__((packed)) ST_RS16_Intensity;

typedef struct
{
    uint8_t reserved[240];
    uint8_t coef;
    uint8_t ver;
} __attribute__((packed)) ST_RS32_Intensity;

typedef struct
{
    uint8_t num[6];
} __attribute__((packed)) ST_SN;

typedef struct
{
    uint8_t device_current[3];
    uint8_t main_current[3];
    uint16_t vol_12v;
    uint16_t vol_12vm;
    uint16_t vol_5v;
    uint16_t vol_3v3;
    uint16_t vol_2v5;
    uint16_t vol_1v2;
} __attribute__((packed)) ST_Status;

typedef struct
{
    uint8_t reserved1[10];
    uint8_t checksum;
    uint16_t manc_err1;
    uint16_t manc_err2;
    uint8_t gps_status;
    uint16_t temperature1;
    uint16_t temperature2;
    uint16_t temperature3;
    uint16_t temperature4;
    uint16_t temperature5;
    uint8_t reserved2[5];
    uint16_t cur_rpm;
    uint8_t reserved3[7];
} __attribute__((packed)) ST_Diagno;

typedef struct
{
    uint64_t sync;
    uint16_t rpm;
    ST_EthNet eth;
    ST_FOV fov;
    uint16_t static_base;
    uint16_t lock_phase_angle;
    ST_Version version;
    ST_RS16_Intensity intensity;
    ST_SN sn;
    uint16_t zero_cali;
    uint8_t return_mode;
    uint16_t sw_ver;
    ST_Timestamp timestamp;
    ST_Status status;
    uint8_t reserved1[11];
    ST_Diagno diagno;
    uint8_t gprmc[86];
    uint8_t static_cali[697];
    uint8_t pitch_cali[48];
    uint8_t reserved2[33];
    uint16_t tail;
} __attribute__((packed)) ST_RS16_DifopPkt;

typedef struct
{
    uint64_t sync;
    uint16_t rpm;
    ST_EthNet eth;
    ST_FOV fov;
    uint16_t reserved0;
    uint16_t lock_phase_angle;
    ST_Version version;
    ST_RS32_Intensity intensity;
    ST_SN sn;
    uint16_t zero_cali;
    uint8_t return_mode;
    uint16_t sw_ver;
    ST_Timestamp timestamp;
    ST_Status status;
    uint8_t reserved1[11];
    ST_Diagno diagno;
    uint8_t gprmc[86];
    uint8_t pitch_cali[96];
    uint8_t yaw_cali[96];
    uint8_t reserved2[586];
    uint16_t tail;
} __attribute__((packed)) ST_RS32_DifopPkt;
static const int RS_PACKET_LENGTH = 1248;

enum E_DECODER_RESULT
{
    E_DECODE_FAIL = -2,
    E_PARAM_INVALID = -1,
    E_DECODE_OK = 0,
    E_FRAME_SPLIT = 1
};

enum RS_RESOLUTION_TYPE
{
    RS_RESOLUTION_5mm = 0,
    RS_RESOLUTION_10mm
};

enum RS_ECHO_MODE
{
    RS_ECHO_DUAL = 0,
    RS_ECHO_MAX,
    RS_ECHO_LAST
};
enum RS_INTENSITY_TYPE
{
    RS_INTENSITY_EXTERN = 1,
    RS_INTENSITY_IN,
    RS_INTENSITY_AUTO
};

typedef struct eST_Param
{
    RS_RESOLUTION_TYPE resolution;
    RS_INTENSITY_TYPE intensity;
    RS_ECHO_MODE echo;
    float cut_angle;
    float max_distance;
    float min_distance;
    float start_angle;
    float end_angle;
} ST_Param;

//----------------- decoder ---------------------
template <typename vpoint>
class DecoderBase
{
public:
    DecoderBase() = default;
    ~DecoderBase();
    E_DECODER_RESULT processMsopPkt(const uint8_t *pkt, std::vector<vpoint> &pointcloud_vec);
    virtual int32_t processDifopPkt(const uint8_t *pkt) = 0;
    virtual int32_t loadCalibrationFile(std::string cali_path) = 0;

protected:
    int32_t rpm_;
    int32_t resolution_type_;
    int32_t intensity_mode_;
    int32_t echo_mode_;
    float Rx_;
    float Ry_;
    float Rz_;
    float max_distance_;
    float min_distance_;
    int start_angle_;
    int end_angle_;
    bool angle_flag_;
    //splite frame variable
    int32_t pkts_per_frame_;
    int32_t pkt_counter_;
    int32_t cut_angle_;
    int32_t last_azimuth_;
    //calibration data
    std::string cali_files_dir_;
    uint32_t cali_data_flag_;
    int32_t intensity_coef_;
    float vert_angle_list_[32];
    float hori_angle_list_[32];
    int32_t channel_cali_[32][51];
    float curve_rate_[32];
    float intensity_cali_[1600][32]; //compatable to old version
    std::vector<double> cos_lookup_table_;
    std::vector<double> sin_lookup_table_;

protected:
    float computeTemperatue(const uint16_t temp_raw);
    float distanceCalibration(int32_t distance, int32_t channel, float temp);
    int32_t azimuthCalibration(float azimuth, int32_t channel);
    int32_t ABPktCheck(int32_t distance);
    virtual int32_t decodeMsopPkt(const uint8_t *pkt, std::vector<vpoint> &vec)=0;
};

template <typename vpoint>
DecoderBase<vpoint>::~DecoderBase()
{
    this->cos_lookup_table_.clear();
    this->sin_lookup_table_.clear();
}

template <typename vpoint>
E_DECODER_RESULT DecoderBase<vpoint>::processMsopPkt(const uint8_t *pkt, std::vector<vpoint> &pointcloud_vec)
{
  if (pkt == NULL)
  {
    return E_PARAM_INVALID;
  }

  int azimuth = decodeMsopPkt(pkt, pointcloud_vec);
  if (azimuth < 0)
  {
    return E_DECODE_FAIL;
  }

  this->pkt_counter_++;
  if (this->cut_angle_ >= 0)
  {
    if (azimuth < this->last_azimuth_)
    {
      this->last_azimuth_ -= 36000;
    }
    if (this->last_azimuth_ != -36001 && this->last_azimuth_ < this->cut_angle_ && azimuth >= this->cut_angle_)
    {
      this->last_azimuth_ = azimuth;

      //          std::cout<<"[RS_decoder][msop][DEBUG] cut angle pkt num: "<<this->pkt_counter_<<", size: "<<pointcloud_vec.size()<<std::endl;

      this->pkt_counter_ = 0;
      return E_FRAME_SPLIT;
    }
    this->last_azimuth_ = azimuth;
  }
  else
  {
    if (this->pkt_counter_ >= this->pkts_per_frame_)
    {
      //          std::cout<<"[RS_decoder][msop][DEBUG] cut pkt num: "<<this->pkt_counter_<<", size: "<<pointcloud_vec.size()<<std::endl;

      this->pkt_counter_ = 0;
      return E_FRAME_SPLIT;
    }
  }

  return E_DECODE_OK;
}

template <typename vpoint>
float DecoderBase<vpoint>::computeTemperatue(const uint16_t temp_raw)
{
    uint8_t neg_flag = (temp_raw >> 8) & 0x80;
    float msb = (temp_raw >> 8) & 0x7F;
    float lsb = (temp_raw & 0xFF00) >> 3;
    float temp;
    if (neg_flag == 0x80)
    {
        temp = -1 * (msb * 32 + lsb) * 0.0625f;
    }
    else
    {
        temp = (msb * 32 + lsb) * 0.0625f;
    }

    return temp;
}

template <typename vpoint>
float DecoderBase<vpoint>::distanceCalibration(int distance, int channel, float temp)
{
    int temp_idx = (int)floor(temp + 0.5);
    if (temp_idx < RS_TEMPERATURE_MIN)
    {
        temp_idx = 0;
    }
    else if (temp_idx > RS_TEMPERATURE_MAX)
    {
        temp_idx = RS_TEMPERATURE_MAX - RS_TEMPERATURE_MIN;
    }
    else
    {
        temp_idx = temp_idx - RS_TEMPERATURE_MIN;
    }
    float dis_ret;
    if (distance <= this->channel_cali_[channel][temp_idx])
    {
        dis_ret = 0.0;
    }
    else
    {
        dis_ret = (float)(distance - this->channel_cali_[channel][temp_idx]);
    }

    return dis_ret;
}

template <typename vpoint>
int DecoderBase<vpoint>::azimuthCalibration(float azimuth, int channel)
{
    int azi_ret;

    if (azimuth > 0.0 && azimuth < 3000.0)
    {
        azimuth = azimuth + this->hori_angle_list_[channel] + 36000.0f;
    }
    else
    {
        azimuth = azimuth + this->hori_angle_list_[channel];
    }
    azi_ret = (int)azimuth;
    azi_ret %= 36000;

    return azi_ret;
}

template <typename vpoint>
int DecoderBase<vpoint>::ABPktCheck(int distance)
{
    int flag = 0;
    if ((distance & 32768) != 0)
    {
        flag = 1;
    }

    return flag;
}

} // namespace sensor
} // namespace robosense