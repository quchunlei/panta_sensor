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
#include <panta_common/msg/rs_msg/lidar_points_msg.h>
#include <panta_common/msg/rs_msg/vehiclestate_msg.h>
#include <panta_common/msg/rs_msg/freespace_msg.h>
#include <panta_common/msg/rs_msg/obstacle_msg.h>
// #include <pcl/io/io.h>

namespace robosense
{
namespace common
{

template <typename PointT>
class PerceptionInterface : virtual public CommonBase
{
public:
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    typedef std::shared_ptr<PerceptionInterface<PointT>> Ptr;
    typedef std::shared_ptr<const PerceptionInterface<PointT>> ConstPtr;

    PerceptionInterface() = default;

    virtual ~PerceptionInterface() = default;

    virtual ErrCode init(const YAML::Node &sdk_config, const std::array<double, 6> &base_pose, const std::string &config_path = "") = 0;
    virtual ErrCode start() = 0;
    virtual ErrCode stop() = 0;

    virtual void regObstacleCallback(const std::function<void(const ObstacleMsg::Ptr &)> &cb) = 0;
    virtual void regFreeSpaceCallback(const std::function<void(const FreeSpaceMsg::Ptr &)> &cb) = 0;
    virtual void regExceptionCallback(const std::function<void(const ErrCode &exception)> &callback) = 0;
    virtual double getProcessingTime() const = 0;
    virtual ErrCode lidarCallback(const LidarPointsMsg &msg) = 0;
    virtual ErrCode vehiclestateCallback(const VehicleStateMsg &msg) = 0;
};
} /* namespace common */
} /* namespace robosense */
