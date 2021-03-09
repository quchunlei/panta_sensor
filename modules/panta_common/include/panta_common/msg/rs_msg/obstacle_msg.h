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
#include <array>
#include <vector>
#include <algorithm>

namespace robosense {
namespace common {

struct alignas(16) Point3 {
    Point3() = default;

    Point3(float tx, float ty, float tz) {
        x = tx;
        y = ty;
        z = tz;
    }

    float norm() const {
        return sqrtf(x*x+y*y+z*z);
    }

    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
};

enum class ObjectType {
    UNKNOW = 0,
    PED = 1,
    BIC = 2,
    CAR = 3,
    TRUCK_BUS = 4,
    ULTRA_VEHICLE = 5,
    MAX_OBJ_TYPE_NUM = 6,
};

enum class MotionType
{
    UNKNOW  = 0,
    MOVING  = 1,
    STATIC  = 2,
    STOPED  = 3,
    MAX_MOTION_STATE_NUM = 4,
};


/**
   * @brief basic struct container for percepted obstacles by robosense SDK.
   * The ObstacleMsg is used as the output from perception module 
   */
struct alignas(16) Obstacle {
    //header
    double timestamp = 0.0;
    uint32_t device_code = 0; //source device code where the underlaying data come from.

    int32_t id = -1;          // obstacle detection ID, only valid for one frame, i.e., will be reinit every frame.

    //---------------------object level info-----------------------
    Point3 anchor;                   //stable anchor point, as lidar points varies dramatically, usually use bary center as init.
    Point3 geo_center;               //geometry center for bounding box.
    Point3 geo_size;
    Point3 geo_direction = Point3(1, 0, 0); //direction for the obstacle, will be refined by tracking, as the direction should be coincidence with velocity direction.

    std::vector<Point3> polygon;    // corner points of the convex hull of the obstacle.
    float detect_confidence = 0.f; //just for obstacle detection, without classification.

    Point3 nearest_point; //nearest corner point of object to lidar.
    Point3 left_point;    //clock-wise the leftmost point of the bostacle.
    Point3 right_point;   //clock-wise the rightmost point of the bostacle.

    float distance = 0.f; //distance between nearest point of obstacle and the lidar.
    float yaw = 0.f;      //yaw angle of the obstacle location relative to the lidar coordinate.
    int32_t point_num; //points num of obstacle pointcloud.

    //---------------------classification info-------------------
    ObjectType type = ObjectType::UNKNOW;
    float type_confidence = 0.f;
    std::vector<float> latent_types;

    //---------------------motion state prediction---------------------
    MotionType motion_state = MotionType::UNKNOW;

    //---------------------tracking info-------------------------
    bool is_track_converged = false;
    int32_t tracker_id = -1;

    Point3 velocity;
    std::array<float, 9> velocity_cov{};
    float velocity_uncertainty = 0.f;
    Point3 ave_velocity;

    Point3 acceleration;
    std::array<float, 9> acceleration_cov{};
    float acceleration_uncertainty = 0.f;
    Point3 ave_acceleration;

    float angle_velocity = 0.f;
    float angle_velocity_cov = 0.f;
    float angle_velocity_uncertainty = 0.f;
    float ave_angle_velocity = 0.f;

    float asso_quality = 0.f;      //the assotiation quality for the current obstacle when linked into a exist tracker.
    float tracker_quality = 0.f; //the estimate quality for the tracker where the obstacle in.
    double tracking_time = 0.f;


    typedef std::shared_ptr<Obstacle> Ptr;
    typedef std::shared_ptr<const Obstacle> ConstPtr;
};

struct alignas(16) ObstacleMsg {
    double timestamp = 0.0;
    uint32_t seq = 0;
    std::string frame_id = "";
    std::string parent_frame_id = "";

    std::vector<Obstacle::Ptr> obstacles;

    typedef std::shared_ptr<ObstacleMsg> Ptr;
    typedef std::shared_ptr<const ObstacleMsg> ConstPtr;
};

} // namespace common
} // namespace robosense
