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
#include "rs_lidar/decoder_base.hpp"
namespace robosense
{
namespace sensor
{
    
template <typename vpoint>
class Decoder16 : public DecoderBase<vpoint>
{
public:
    Decoder16(ST_Param &param);
    ~Decoder16();
    E_DECODER_RESULT processMsopPkt(const uint8_t *pkt, std::vector<vpoint> &pointcloud_vec);
    int32_t processDifopPkt(const uint8_t *pkt);
    int32_t loadCalibrationFile(std::string cali_path);

private:
    int32_t decodeMsopPkt(const uint8_t *pkt, std::vector<vpoint> &vec);
    float intensityCalibration(float intensity, int32_t channel, int32_t distance, float temp);
};

template <typename vpoint>
Decoder16<vpoint>::Decoder16(ST_Param &param)
{
    //param init
    this->rpm_ = 600;
    this->pkts_per_frame_ = 84;
    memset(this->channel_cali_, 0, sizeof(int) * 32 * 51);
    this->cut_angle_ = -1;
    this->cali_files_dir_ = ".";
    this->resolution_type_ = RS_RESOLUTION_10mm;
    this->intensity_mode_ = 1;
    this->echo_mode_ = RS_ECHO_MAX;

    this->Rx_ = 0.03825;
    this->Ry_ = -0.01088;
    this->Rz_ = 0;

    this->pkt_counter_ = 0;
    this->last_azimuth_ = -36001;
    this->cali_data_flag_ = 0x00;
    this->intensity_coef_ = 1;

    this->max_distance_ = 200.0f;
    this->min_distance_ = 0.2f;

    this->start_angle_ = 0.0f;
    this->end_angle_ = 360.0f;
    this->angle_flag_ = true;

    this->intensity_mode_ = param.intensity;
    this->resolution_type_ = param.resolution;
    if (param.cut_angle > 360.0f)
    {
        this->cut_angle_ = 0;
    }
    else
    {
        this->cut_angle_ = param.cut_angle * 100;
    }
    if (param.max_distance > 200.0f || param.max_distance < 0.2f)
    {
        this->max_distance_ = 200.0f;
    }
    else
    {
        this->max_distance_ = param.max_distance;
    }
    if (param.min_distance > 200.0f || param.min_distance > param.max_distance)
    {
        this->min_distance_ = 0.2f;
    }
    else
    {
        this->min_distance_ = param.min_distance;
    }
    if (param.start_angle > 360 || param.start_angle < 0 || param.end_angle > 360 || param.end_angle < 0)
    {
        param.start_angle = 0.0f;
        param.end_angle = 360.0f;
    }
    this->start_angle_ = (int)(param.start_angle * 100);
    this->end_angle_ = (int)(param.end_angle * 100);
    if (this->start_angle_ > this->end_angle_)
    {
        this->angle_flag_ = false;
    }
    this->echo_mode_ = param.echo;

    //packet rate
    int pkt_rate = ceil(RS_POINTS_CHANNEL_PER_SECOND / RS_BLOCKS_CHANNEL_PER_PKT);
    if (this->echo_mode_ == RS_ECHO_LAST || this->echo_mode_ == RS_ECHO_MAX)
    {
        pkt_rate = ceil(pkt_rate / 2);
    }
    this->pkts_per_frame_ = ceil(pkt_rate * 60 / this->rpm_);
    this->cos_lookup_table_.resize(36000);
    this->sin_lookup_table_.resize(36000);
    for (unsigned int i = 0; i < 36000; i++)
    {
        double rad = RS_TO_RADS(i / 100.0f);
        this->cos_lookup_table_[i] = std::cos(rad);
        this->sin_lookup_table_[i] = std::sin(rad);
    }
}

template <typename vpoint>
Decoder16<vpoint>::~Decoder16()
{
    this->cos_lookup_table_.clear();
    this->sin_lookup_table_.clear();
}

template <typename vpoint>
int Decoder16<vpoint>::decodeMsopPkt(const uint8_t *pkt, std::vector<vpoint> &vec)
{
    if (pkt == NULL)
    {
        return -1;
    }
    ST_MsopPkt *mpkt_ptr = (ST_MsopPkt *)pkt;
    if (mpkt_ptr->header.sync != RS_MSOP_SYNC)
    {
        return -2;
    }
    int first_azimuth;
    first_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
    float temperature = this->computeTemperatue(mpkt_ptr->header.temp_raw);
    for (int blk_idx = 0; blk_idx < RS_BLOCKS_PER_PKT; blk_idx++)
    {
        if (mpkt_ptr->blocks[blk_idx].id != RS_BLOCK_ID)
        {
            break;
        }
        int azimuth_blk = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
        int azi_prev;
        int azi_cur;

        if (blk_idx < (RS_BLOCKS_PER_PKT - 1)) // 12
        {
            azi_prev = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + 1].azimuth);
            azi_cur = azimuth_blk;
        }
        else
        {
            azi_prev = azimuth_blk;
            azi_cur = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - 1].azimuth);
        }
        float azimuth_diff = (float)((36000 + azi_prev - azi_cur) % 36000);
        float azimuth_channel;
        int ab_flag = 0;
        for (int channel_idx = 0; channel_idx < RS_CHANNELS_PER_BLOCK; channel_idx++)
        {
            int azimuth_final;

            if (this->echo_mode_ == RS_ECHO_DUAL)
            {
                azimuth_channel = azimuth_blk + azimuth_diff * RS_CHANNEL_TOFFSET * (channel_idx % 16) / RS16_BLOCK_TDURATION_DUAL;
            }
            else
            {
                azimuth_channel = azimuth_blk + azimuth_diff * (RS_FIRING_TDURATION * (channel_idx / 16) + RS_CHANNEL_TOFFSET * (channel_idx % 16)) / RS16_BLOCK_TDURATION_SINGLE;
            }
            azimuth_final = ((int)round(azimuth_channel)) % 36000;
            int idx_map = channel_idx;
            int distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[idx_map].distance);
            float intensity = mpkt_ptr->blocks[blk_idx].channels[idx_map].intensity;
            intensity = intensityCalibration(intensity, channel_idx, distance, temperature);
            float distance_cali = this->distanceCalibration(distance, channel_idx, temperature);
            if (this->resolution_type_ == RS_RESOLUTION_5mm)
            {
                distance_cali = distance_cali * RS_RESOLUTION_5mm_DISTANCE_COEF;
            }
            else
            {
                distance_cali = distance_cali * RS_RESOLUTION_10mm_DISTANCE_COEF;
            }
            //
            int angle_horiz_ori;
            int angle_horiz = (azimuth_final + 36000) % 36000;
            int angle_vert;
            angle_horiz_ori = angle_horiz;
            angle_vert = (((int)(this->vert_angle_list_[channel_idx % 16] * 100) % 36000) + 36000) % 36000;

            //store to pointcloud buffer
            vpoint point;
            if ((distance_cali <= this->max_distance_ && distance_cali >= this->min_distance_) && ((this->angle_flag_ && angle_horiz >= this->start_angle_ && angle_horiz <= this->end_angle_) || (!this->angle_flag_ && ((angle_horiz >= this->start_angle_ && angle_horiz <= 36000) || (angle_horiz >= 0 && angle_horiz <= this->end_angle_)))))
            {
                const double vert_cos_value = this->cos_lookup_table_[angle_vert];
                const double horiz_cos_value = this->cos_lookup_table_[angle_horiz];
                const double horiz_ori_cos_value = this->cos_lookup_table_[angle_horiz_ori];
                point.x = distance_cali * vert_cos_value * horiz_cos_value + this->Rx_ * horiz_ori_cos_value;

                const double horiz_sin_value = this->sin_lookup_table_[angle_horiz];
                const double horiz_ori_sin_value = this->sin_lookup_table_[angle_horiz_ori];
                point.y = -distance_cali * vert_cos_value * horiz_sin_value - this->Rx_ * horiz_ori_sin_value;

                const double vert_sin_value = this->sin_lookup_table_[angle_vert];
                point.z = distance_cali * vert_sin_value + this->Rz_;

                point.intensity = intensity;
                if (std::isnan(point.intensity))
                {
                    point.intensity = 0;
                }
            }
            else
            {
                point.x = NAN;
                point.y = NAN;
                point.z = NAN;
                point.intensity = NAN;
            }
            vec.push_back(point);
        }
    }
    return first_azimuth;
}

template <typename vpoint>
float Decoder16<vpoint>::intensityCalibration(float intensity, int channel, int distance, float temp)
{
    if (this->intensity_mode_ == 3)
    {
        return intensity;
    }
    else
    {
        float real_pwr = std::max((float)(intensity / (1 + (temp - RS_TEMPERATURE_MIN) / 24.0f)), 1.0f);
        if (this->intensity_mode_ == 1)
        {
            if ((int)real_pwr < 126)
            {
                real_pwr = real_pwr * 4.0f;
            }
            else if ((int)real_pwr >= 126 && (int)real_pwr < 226)
            {
                real_pwr = (real_pwr - 125.0f) * 16.0f + 500.0f;
            }
            else
            {
                real_pwr = (real_pwr - 225.0f) * 256.0f + 2100.0f;
            }
        }
        else if (this->intensity_mode_ == 2)
        {
            if ((int)real_pwr < 64)
            {
                real_pwr = real_pwr;
            }
            else if ((int)real_pwr >= 64 && (int)real_pwr < 176)
            {
                real_pwr = (real_pwr - 64.0f) * 4.0f + 64.0f;
            }
            else
            {
                real_pwr = (real_pwr - 176.0f) * 16.0f + 512.0f;
            }
        }

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

        int distance_cali = (distance > this->channel_cali_[channel][temp_idx]) ? distance : this->channel_cali_[channel][temp_idx];
        distance_cali = distance_cali - this->channel_cali_[channel][temp_idx];
        float distance_final = (float)distance_cali * RS_RESOLUTION_5mm_DISTANCE_COEF;
        if (this->resolution_type_ == RS_RESOLUTION_10mm)
        {
            distance_final = (float)distance_cali * RS_RESOLUTION_10mm_DISTANCE_COEF;
        }

        float ref_pwr_temp = 0.0f;
        int order = 3;
        float sect1 = 5.0f;
        float sect2 = 40.0f;

        if (this->intensity_mode_ == 1)
        {
            if (distance_final <= sect1)
            {
                ref_pwr_temp = this->intensity_cali_[0][channel] * exp(this->intensity_cali_[1][channel] -
                                                                       this->intensity_cali_[2][channel] * distance_final) +
                               this->intensity_cali_[3][channel];
            }
            else
            {
                for (int i = 0; i < order; i++)
                {
                    ref_pwr_temp += this->intensity_cali_[i + 4][channel] * (pow(distance_final, order - 1 - i));
                }
            }
        }
        else if (this->intensity_mode_ == 2)
        {
            if (distance_final <= sect1)
            {
                ref_pwr_temp = this->intensity_cali_[0][channel] * exp(this->intensity_cali_[1][channel] -
                                                                       this->intensity_cali_[2][channel] * distance_final) +
                               this->intensity_cali_[3][channel];
            }
            else if (distance_final > sect1 && distance_final <= sect2)
            {
                for (int i = 0; i < order; i++)
                {
                    ref_pwr_temp += this->intensity_cali_[i + 4][channel] * (pow(distance_final, order - 1 - i));
                }
            }
            else
            {
                float ref_pwr_t0 = 0.0f;
                float ref_pwr_t1 = 0.0f;
                for (int i = 0; i < order; i++)
                {
                    ref_pwr_t0 += this->intensity_cali_[i + 4][channel] * pow(40.0f, order - 1 - i);
                    ref_pwr_t1 += this->intensity_cali_[i + 4][channel] * pow(39.0f, order - 1 - i);
                }
                ref_pwr_temp = 0.3f * (ref_pwr_t0 - ref_pwr_t1) * distance_final + ref_pwr_t0;
            }
        }
        float ref_pwr = std::max(std::min(ref_pwr_temp, 500.0f), 4.0f);
        float intensity_f = (this->intensity_coef_ * ref_pwr) / real_pwr;
        intensity_f = (int)intensity_f > 255 ? 255.0f : intensity_f;
        return intensity_f;
    }
}
template <typename vpoint>
int Decoder16<vpoint>::processDifopPkt(const uint8_t *pkt)
{
    if (pkt == NULL)
    {
        return -1;
    }

    ST_RS16_DifopPkt *rs16_ptr = (ST_RS16_DifopPkt *)pkt;
    if (rs16_ptr->sync != RS_DIFOP_SYNC)
    {
        return -2;
    }

    ST_Version *p_ver = &(rs16_ptr->version);
    if ((p_ver->bottom_sn[0] == 0x08 && p_ver->bottom_sn[1] == 0x02 && p_ver->bottom_sn[2] >= 0x09) ||
        (p_ver->bottom_sn[0] == 0x08 && p_ver->bottom_sn[1] > 0x02))
    {
        if (rs16_ptr->return_mode == 0x01 || rs16_ptr->return_mode == 0x02)
        {
            this->echo_mode_ = rs16_ptr->return_mode;
        }
        else
        {
            this->echo_mode_ = 0;
        }
    }
    else
    {
        this->echo_mode_ = 1;
    }

    int pkt_rate = ceil(RS_POINTS_CHANNEL_PER_SECOND / RS_BLOCKS_CHANNEL_PER_PKT);
    if (this->echo_mode_ == RS_ECHO_LAST || this->echo_mode_ == RS_ECHO_MAX)
    {
        pkt_rate = ceil(pkt_rate / 2);
    }
    this->pkts_per_frame_ = ceil(pkt_rate * 60 / this->rpm_);

    if ((p_ver->main_sn[1] == 0x00 && p_ver->main_sn[2] == 0x00 && p_ver->main_sn[3] == 0x00) || (p_ver->main_sn[1] == 0xFF && p_ver->main_sn[2] == 0xFF && p_ver->main_sn[3] == 0xFF) || (p_ver->main_sn[1] == 0x55 && p_ver->main_sn[2] == 0xAA && p_ver->main_sn[3] == 0x5A) || (p_ver->main_sn[1] == 0xE9 && p_ver->main_sn[2] == 0x01 && p_ver->main_sn[3] == 0x00))
    {
        this->resolution_type_ = 1;
    }
    else
    {
        this->resolution_type_ = 0;
    }

    if (!(this->cali_data_flag_ & 0x1))
    {
        bool curve_flag = true;
        ST_RS16_Intensity *p_intensity = &(rs16_ptr->intensity);
        if ((p_intensity->intensity_cali[0] == 0x00 || p_intensity->intensity_cali[0] == 0xFF) && (p_intensity->intensity_cali[1] == 0x00 || p_intensity->intensity_cali[1] == 0xFF) && (p_intensity->intensity_cali[2] == 0x00 || p_intensity->intensity_cali[2] == 0xFF) && (p_intensity->intensity_cali[3] == 0x00 || p_intensity->intensity_cali[3] == 0xFF))
        {
            curve_flag = false;
        }

        if (curve_flag)
        {
            bool check_flag = true;
            uint8_t checksum;
            for (int k = 0; k < 16; k++)
            {
                checksum = p_intensity->intensity_cali[15 * k] ^ p_intensity->intensity_cali[15 * k + 1];
                for (int n = 1; n < 7; n++)
                {
                    checksum = checksum ^ (p_intensity->intensity_cali[k * 15 + n * 2]) ^ (p_intensity->intensity_cali[k * 15 + n * 2 + 1]);
                }
                if (checksum != p_intensity->intensity_cali[k * 15 + 14])
                {
                    check_flag = false;
                    break;
                }
            }

            if (check_flag)
            {
                uint16_t *inten_p;
                for (int i = 0; i < 16; i++)
                {
                    inten_p = (uint16_t *)(p_intensity->intensity_cali + i * 15);
                    for (int k = 0; k < 7; k++)
                    {
                        this->intensity_cali_[k][i] = RS_SWAP_SHORT(*(inten_p + k)) * 0.001;
                    }
                }

                this->cali_data_flag_ = this->cali_data_flag_ | 0x01;

                //std::cout << "[RS_decoder][difop][INFO] curves data is wrote in difop packet!" << std::endl;
            }
        }
        if (rs16_ptr->intensity.coef != 0x00 && rs16_ptr->intensity.coef != 0xFF)
        {
            this->intensity_coef_ = rs16_ptr->intensity.coef;
        }
    }
    {
        if (rs16_ptr->intensity.ver == 0x00 || rs16_ptr->intensity.ver == 0xFF || rs16_ptr->intensity.ver == 0xA1)
        {
            this->intensity_mode_ = 1;
        }
        else if (rs16_ptr->intensity.ver == 0xB1)
        {
            this->intensity_mode_ = 2;
        }
        else if (rs16_ptr->intensity.ver == 0xC1)
        {
            this->intensity_mode_ = 3;
        }
    }
    if (!(this->cali_data_flag_ & 0x2))
    {
        bool angle_flag = true;
        const uint8_t *p_pitch_cali;

        p_pitch_cali = rs16_ptr->pitch_cali;

        if ((p_pitch_cali[0] == 0x00 || p_pitch_cali[0] == 0xFF) && (p_pitch_cali[1] == 0x00 || p_pitch_cali[1] == 0xFF) && (p_pitch_cali[2] == 0x00 || p_pitch_cali[2] == 0xFF) && (p_pitch_cali[3] == 0x00 || p_pitch_cali[3] == 0xFF))
        {
            angle_flag = false;
        }

        if (angle_flag)
        {
            int lsb, mid, msb, neg = 1;

            for (int i = 0; i < 16; i++)
            {
                if (i < 8)
                {
                    neg = -1;
                }
                else
                {
                    neg = 1;
                }
                lsb = p_pitch_cali[i * 3];
                mid = p_pitch_cali[i * 3 + 1];
                msb = p_pitch_cali[i * 3 + 2];

                this->vert_angle_list_[i] = (lsb * 256 * 256 + mid * 256 + msb) * neg * 0.0001f; // / 180 * M_PI;
                this->hori_angle_list_[i] = 0;
            }

            this->cali_data_flag_ = this->cali_data_flag_ | 0x2;
        }
    }

    return 0;
}
template <typename vpoint>
int Decoder16<vpoint>::loadCalibrationFile(std::string cali_path)
{
    int row_index = 0;
    int laser_num = 32;
    std::string line_str;
    this->cali_files_dir_ = cali_path;
    std::string file_dir = this->cali_files_dir_ + "/curves.csv";
    std::ifstream fd(file_dir.c_str(), std::ios::in);

    if (!fd.is_open())
    {
        //  WARNING << "rs_lidar: file  " << file_dir << " open fail!" << REND;
    }
    else
    {
        fd.seekg(0, std::ios::end);
        int len = fd.tellg();
        int total = 7;
        if (len > 10000)
        {
            total = 1600;
        }

        fd.seekg(0);
        row_index = 0;

        laser_num = 16;

        while (getline(fd, line_str))
        {
            std::stringstream ss(line_str);
            std::string str;
            std::vector<std::string> vect_str;
            while (getline(ss, str, ','))
            {
                vect_str.push_back(str);
            }
            for (int i = 0; i < laser_num; i++)
            {
                this->intensity_cali_[row_index][i] = std::stof(vect_str[i]);
            }
            row_index++;
            if (row_index >= total)
            {
                break;
            }
        }

        fd.close();
    }

    //read angle.csf
    file_dir = this->cali_files_dir_ + "/angle.csv";
    fd.open(file_dir.c_str(), std::ios::in);
    if (!fd.is_open())
    {
        //  WARNING << "rs_lidar: file  " << file_dir << " open fail!" << REND;
    }
    else
    {
        row_index = 0;
        while (std::getline(fd, line_str))
        {
            std::stringstream ss(line_str);
            std::string str;
            std::vector<std::string> vect_str;
            while (std::getline(ss, str, ','))
            {
                vect_str.push_back(str);
            }
            this->vert_angle_list_[row_index] = std::stof(vect_str[0]);

            this->hori_angle_list_[row_index] = 0;

            row_index++;
            if (row_index >= laser_num)
            {
                break;
            }
        }
        fd.close();
    }

    //read channel.csf
    file_dir = this->cali_files_dir_ + "/ChannelNum.csv";
    fd.open(file_dir.c_str(), std::ios::in);
    if (!fd.is_open())
    {
        //  WARNING << "rs_lidar: file  " << file_dir << " open fail!" << REND;
    }
    else
    {
        row_index = 0;
        int temperature_range = 41;
        while (std::getline(fd, line_str))
        {
            std::stringstream ss(line_str);
            std::string str;
            std::vector<std::string> vect_str;
            while (std::getline(ss, str, ','))
            {
                vect_str.push_back(str);
            }
            for (int i = 0; i < temperature_range; i++)
            {
                this->channel_cali_[row_index][i] = std::stoi(vect_str[i]);
            }
            row_index++;
            if (row_index >= laser_num)
            {
                break;
            }
        }
        fd.close();
    }
    return 0;
}
} // namespace sensor
} // namespace robosense