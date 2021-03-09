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

#include <string>
#include <functional>
#include <panta_common/common.h>
#include <panta_common/msg/rs_msg/imu_msg.h>
#include <panta_common/msg/rs_msg/lidar_points_msg.h>
#include <panta_common/msg/rs_msg/odom_msg.h>
#include <panta_common/msg/rs_msg/vehiclestate_msg.h>

namespace robosense {
namespace common {

template <typename PointT>
class OdometryInterface : virtual public CommonBase {
public:
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    typedef std::shared_ptr<OdometryInterface<PointT>> Ptr;
    typedef std::shared_ptr<const OdometryInterface<PointT>> ConstPtr;

    OdometryInterface() = default;

    virtual ~OdometryInterface() = default;

    virtual ErrCode init(const YAML::Node &sdk_config) = 0;

    virtual ErrCode start() = 0;

    virtual ErrCode stop() = 0;

    virtual ErrCode getVehicleState(VehicleStateMsg &state) = 0;

//    virtual double getProcessingTime() = 0;

    virtual void regExceptionCallback(const std::function<void(const ErrCode &exception)> &) = 0;

    virtual ErrCode imuCallback(const ImuMsg &msg) = 0;

    virtual ErrCode odomCallback(const OdomMsg &msg) = 0;

    virtual ErrCode lidarCallback(const LidarPointsMsg &msg) = 0;
};
} /* namespace common */
} /* namespace robosense */
