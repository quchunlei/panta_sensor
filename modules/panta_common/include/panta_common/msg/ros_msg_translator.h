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
#ifdef ROS_FOUND
#include <ros/duration.h>
#include <ros/rate.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include "panta_common/msg/rs_msg/imu_msg.h"
#include "panta_common/msg/rs_msg/gnss_msg.h"
#include "panta_common/msg/rs_msg/odom_msg.h"
#include "panta_common/msg/rs_msg/lidar_points_msg.h"
#include "panta_common/msg/rs_msg/lidar_packet_msg.h"
#include "panta_common/msg/rs_msg/lidar_scan_msg.h"
#include "panta_common/msg/rs_msg/vehiclestate_msg.h"
#include "panta_common/msg/rs_msg/obstacle_msg.h"
#include "panta_common/msg/rs_msg/freespace_msg.h"
#include "panta_common/msg/ros_msg/freespace_set_ros_msg.h"
#include "panta_common/msg/ros_msg/freespace_ros_msg.h"
#include "panta_common/msg/ros_msg/obstacle_set_ros_msg.h"
#include "panta_common/msg/ros_msg/obstacle_ros_msg.h"
#include "panta_common/msg/ros_msg/lidar_scan_ros.h"
#include "panta_common/msg/rs_msg/grid_map_msg.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

namespace robosense
{
namespace common
{

/************************************************************************/
/**Translation functions between robosense message and ROS message**/
/************************************************************************/

inline FreeSpaceMsg::Ptr toRsMsg(const rs_perception::freespace_set_ros_msg &ros_msg)
{

    FreeSpaceMsg::Ptr rs_msg(new FreeSpaceMsg);

    rs_msg->timestamp = ros_msg.header.stamp.toSec();
    rs_msg->frame_id = ros_msg.header.frame_id;
    rs_msg->seq = ros_msg.header.seq;

    rs_msg->freespaces.resize(ros_msg.freespaces.size());
    for (size_t i = 0; i < ros_msg.freespaces.size(); ++i)
    {
        FreeSpace::Ptr &fs = rs_msg->freespaces[i];
        const rs_perception::freespace_ros_msg &r_fs = ros_msg.freespaces[i];
        fs->timestamp = r_fs.timestamp;
        fs->device_code = r_fs.device_code;

        fs->distance = r_fs.distance;
        fs->yaw_angle = r_fs.yaw_angle;
        fs->free_prob = r_fs.free_prob;
    }
    return rs_msg;
}

inline rs_perception::freespace_set_ros_msg toRosMsg(const FreeSpaceMsg::Ptr &rs_msg)
{

    rs_perception::freespace_set_ros_msg ros_msg;

    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg->timestamp);
    ros_msg.header.frame_id = rs_msg->frame_id;
    ros_msg.header.seq = rs_msg->seq;

    ros_msg.freespaces.resize(rs_msg->freespaces.size());
    for (size_t i = 0; i < rs_msg->freespaces.size(); ++i)
    {
        rs_perception::freespace_ros_msg &r_fs = ros_msg.freespaces[i];
        const FreeSpace::Ptr &fs = rs_msg->freespaces[i];

        r_fs.timestamp = fs->timestamp;
        r_fs.device_code = fs->device_code;

        r_fs.distance = fs->distance;
        r_fs.yaw_angle = fs->yaw_angle;
        r_fs.free_prob = fs->free_prob;
    }

    return ros_msg;
}

inline rs_perception::obstacle_set_ros_msg toRosMsg(const ObstacleMsg::Ptr &rs_msg)
{

    rs_perception::obstacle_set_ros_msg ros_msg;

    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg->timestamp);
    ros_msg.header.frame_id = rs_msg->frame_id;
    ros_msg.header.seq = rs_msg->seq;

    ros_msg.obstcles.resize(rs_msg->obstacles.size());
    for (size_t i = 0; i < rs_msg->obstacles.size(); ++i)
    {
        const Obstacle::Ptr &obs = rs_msg->obstacles[i];
        rs_perception::obstacle_ros_msg &r_obs = ros_msg.obstcles[i];

        r_obs.timestamp = obs->timestamp;
        r_obs.device_code = obs->device_code;
        r_obs.id = obs->id;

        r_obs.anchor.x = obs->anchor.x;
        r_obs.anchor.y = obs->anchor.y;
        r_obs.anchor.z = obs->anchor.z;

        r_obs.geo_center.x = obs->geo_center.x;
        r_obs.geo_center.y = obs->geo_center.y;
        r_obs.geo_center.z = obs->geo_center.z;

        r_obs.geo_size.x = obs->geo_size.x;
        r_obs.geo_size.y = obs->geo_size.y;
        r_obs.geo_size.z = obs->geo_size.z;

        r_obs.geo_direction.x = obs->geo_direction.x;
        r_obs.geo_direction.y = obs->geo_direction.y;
        r_obs.geo_direction.z = obs->geo_direction.z;

        r_obs.polygon.resize(obs->polygon.size());
        for (size_t k = 0; k < obs->polygon.size(); ++k)
        {
            r_obs.polygon[k].x = obs->polygon[k].x;
            r_obs.polygon[k].y = obs->polygon[k].y;
            r_obs.polygon[k].z = obs->polygon[k].z;
        }

        r_obs.detect_confidence = obs->detect_confidence;

        r_obs.nearest_point.x = obs->nearest_point.x;
        r_obs.nearest_point.y = obs->nearest_point.y;
        r_obs.nearest_point.z = obs->nearest_point.z;

        r_obs.left_point.x = obs->left_point.x;
        r_obs.left_point.y = obs->left_point.y;
        r_obs.left_point.z = obs->left_point.z;

        r_obs.right_point.x = obs->right_point.x;
        r_obs.right_point.y = obs->right_point.y;
        r_obs.right_point.z = obs->right_point.z;

        r_obs.distance = obs->distance;
        r_obs.yaw = obs->yaw;
        r_obs.point_num = obs->point_num;

        r_obs.type = static_cast<int32_t>(obs->type);
        r_obs.type_confidence = obs->type_confidence;
        r_obs.latent_types.resize(obs->latent_types.size());
        for (size_t k = 0; k < obs->latent_types.size(); ++k)
        {
            r_obs.latent_types[k] = obs->latent_types[k];
        }
        r_obs.motion_state = static_cast<int32_t>(obs->motion_state);

        r_obs.is_track_converged = obs->is_track_converged;
        r_obs.tracker_id = obs->tracker_id;

        r_obs.velocity.x = obs->velocity.x;
        r_obs.velocity.y = obs->velocity.y;
        r_obs.velocity.z = obs->velocity.z;

        r_obs.velocity_cov.resize(obs->velocity_cov.size());
        for (size_t k = 0; k < obs->velocity_cov.size(); ++k)
        {
            r_obs.velocity_cov[k] = obs->velocity_cov[k];
        }

        r_obs.velocity_uncertainty = obs->velocity_uncertainty;

        r_obs.ave_velocity.x = obs->ave_velocity.x;
        r_obs.ave_velocity.y = obs->ave_velocity.y;
        r_obs.ave_velocity.z = obs->ave_velocity.z;

        r_obs.acceleration.x = obs->acceleration.x;
        r_obs.acceleration.y = obs->acceleration.y;
        r_obs.acceleration.z = obs->acceleration.z;

        r_obs.acceleration_cov.resize(obs->acceleration_cov.size());
        for (size_t k = 0; k < obs->acceleration_cov.size(); ++k)
        {
            r_obs.acceleration_cov[k] = obs->acceleration_cov[k];
        }

        r_obs.acceleration_uncertainty = obs->acceleration_uncertainty;

        r_obs.ave_acceleration.x = obs->ave_acceleration.x;
        r_obs.ave_acceleration.y = obs->ave_acceleration.y;
        r_obs.ave_acceleration.z = obs->ave_acceleration.z;

        r_obs.angle_velocity = obs->angle_velocity;
        r_obs.angle_velocity_cov = obs->angle_velocity_cov;
        r_obs.angle_velocity_uncertainty = obs->angle_velocity_uncertainty;
        r_obs.ave_angle_velocity = obs->ave_angle_velocity;

        r_obs.asso_quality = obs->asso_quality;
        r_obs.tracker_quality = obs->tracker_quality;
        r_obs.tracking_time = obs->tracking_time;
    }

    return ros_msg;
}

inline ObstacleMsg::Ptr toRsMsg(const rs_perception::obstacle_set_ros_msg &ros_msg)
{
    ObstacleMsg::Ptr rs_msg(new ObstacleMsg);

    rs_msg->timestamp = ros_msg.header.stamp.toSec();
    rs_msg->frame_id = ros_msg.header.frame_id;
    rs_msg->seq = ros_msg.header.seq;

    rs_msg->obstacles.resize(ros_msg.obstcles.size());
    for (size_t i = 0; i < ros_msg.obstcles.size(); ++i)
    {
        Obstacle::Ptr &obs = rs_msg->obstacles[i];
        const rs_perception::obstacle_ros_msg &r_obs = ros_msg.obstcles[i];

        obs->timestamp = r_obs.timestamp;
        obs->device_code = r_obs.device_code;

        obs->id = r_obs.id;

        obs->anchor.x = r_obs.anchor.x;
        obs->anchor.y = r_obs.anchor.y;
        obs->anchor.z = r_obs.anchor.z;

        obs->geo_center.x = r_obs.geo_center.x;
        obs->geo_center.y = r_obs.geo_center.y;
        obs->geo_center.z = r_obs.geo_center.z;

        obs->geo_size.x = r_obs.geo_size.x;
        obs->geo_size.y = r_obs.geo_size.y;
        obs->geo_size.z = r_obs.geo_size.z;

        obs->geo_direction.x = r_obs.geo_direction.x;
        obs->geo_direction.y = r_obs.geo_direction.y;
        obs->geo_direction.z = r_obs.geo_direction.z;

        obs->polygon.resize(r_obs.polygon.size());
        for (size_t k = 0; k < r_obs.polygon.size(); ++k)
        {
            obs->polygon[k].x = r_obs.polygon[k].x;
            obs->polygon[k].y = r_obs.polygon[k].y;
            obs->polygon[k].z = r_obs.polygon[k].z;
        }

        obs->detect_confidence = r_obs.detect_confidence;

        obs->nearest_point.x = r_obs.nearest_point.x;
        obs->nearest_point.y = r_obs.nearest_point.y;
        obs->nearest_point.z = r_obs.nearest_point.z;

        obs->left_point.x = r_obs.left_point.x;
        obs->left_point.y = r_obs.left_point.y;
        obs->left_point.z = r_obs.left_point.z;

        obs->right_point.x = r_obs.right_point.x;
        obs->right_point.y = r_obs.right_point.y;
        obs->right_point.z = r_obs.right_point.z;

        obs->distance = r_obs.distance;
        obs->yaw = r_obs.yaw;
        obs->point_num = r_obs.point_num;

        obs->type = static_cast<ObjectType>(r_obs.type);
        obs->type_confidence = r_obs.type_confidence;
        obs->latent_types.resize(r_obs.latent_types.size());
        for (size_t k = 0; k < r_obs.latent_types.size(); ++k)
        {
            obs->latent_types[k] = r_obs.latent_types[k];
        }
        obs->motion_state = static_cast<MotionType>(r_obs.motion_state);

        obs->is_track_converged = r_obs.is_track_converged;
        obs->tracker_id = r_obs.tracker_id;

        obs->velocity.x = r_obs.velocity.x;
        obs->velocity.y = r_obs.velocity.y;
        obs->velocity.z = r_obs.velocity.z;

        for (size_t k = 0; k < r_obs.velocity_cov.size() && k < 9; ++k)
        {
            obs->velocity_cov[k] = r_obs.velocity_cov[k];
        }

        obs->velocity_uncertainty = r_obs.velocity_uncertainty;

        obs->ave_velocity.x = r_obs.ave_velocity.x;
        obs->ave_velocity.y = r_obs.ave_velocity.y;
        obs->ave_velocity.z = r_obs.ave_velocity.z;

        obs->acceleration.x = r_obs.acceleration.x;
        obs->acceleration.y = r_obs.acceleration.y;
        obs->acceleration.z = r_obs.acceleration.z;

        for (size_t k = 0; k < r_obs.acceleration_cov.size() && k < 9; ++k)
        {
            obs->acceleration_cov[k] = r_obs.acceleration_cov[k];
        }

        obs->acceleration_uncertainty = r_obs.acceleration_uncertainty;

        obs->ave_acceleration.x = r_obs.ave_acceleration.x;
        obs->ave_acceleration.y = r_obs.ave_acceleration.y;
        obs->ave_acceleration.z = r_obs.ave_acceleration.z;

        obs->angle_velocity = r_obs.angle_velocity;
        obs->angle_velocity_cov = r_obs.angle_velocity_cov;
        obs->angle_velocity_uncertainty = r_obs.angle_velocity_uncertainty;
        obs->ave_angle_velocity = r_obs.ave_angle_velocity;

        obs->asso_quality = r_obs.asso_quality;
        obs->tracker_quality = r_obs.tracker_quality;
        obs->tracking_time = r_obs.tracking_time;
    }

    return rs_msg;
}

inline ImuMsg toRsMsg(const sensor_msgs::Imu &ros_msg)
{
    ImuMsg rs_msg;
    geometry_msgs::Quaternion orientation = ros_msg.orientation;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    rs_msg.orien[0] = roll;
    rs_msg.orien[1] = pitch;
    rs_msg.orien[2] = yaw;

    rs_msg.acc[0] = ros_msg.linear_acceleration.x;
    rs_msg.acc[1] = ros_msg.linear_acceleration.y;
    rs_msg.acc[2] = ros_msg.linear_acceleration.z;

    rs_msg.angular_vel[0] = ros_msg.angular_velocity.x;
    rs_msg.angular_vel[1] = ros_msg.angular_velocity.y;
    rs_msg.angular_vel[2] = ros_msg.angular_velocity.z;

    rs_msg.timestamp = ros_msg.header.stamp.toSec();
    rs_msg.parent_frame_id = ros_msg.header.frame_id;
    rs_msg.frame_id = "/imu";
    rs_msg.seq = ros_msg.header.seq;
    return rs_msg;
}

inline sensor_msgs::Imu toRosMsg(const ImuMsg &rs_msg)
{
    sensor_msgs::Imu ros_msg;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(rs_msg.orien[0], rs_msg.orien[1], rs_msg.orien[2]);

    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
    ros_msg.header.frame_id = rs_msg.parent_frame_id;
    ros_msg.header.seq = rs_msg.seq;
    ros_msg.angular_velocity.x = (double)(rs_msg.angular_vel[0]);
    ros_msg.angular_velocity.y = (double)(rs_msg.angular_vel[1]);
    ros_msg.angular_velocity.z = (double)(rs_msg.angular_vel[2]);
    ros_msg.linear_acceleration.x = (double)((rs_msg.acc[0]));
    ros_msg.linear_acceleration.y = (double)((rs_msg.acc[1]));
    ros_msg.linear_acceleration.z = (double)((rs_msg.acc[2]));
    ros_msg.orientation = odom_quat;
    return ros_msg;
}
inline GnssMsg toRsMsg(const sensor_msgs::NavSatFix &ros_msg)
{
    GnssMsg rs_msg;
    rs_msg.seq = ros_msg.header.seq;
    rs_msg.timestamp = ros_msg.header.stamp.toSec();
    rs_msg.parent_frame_id = ros_msg.header.frame_id;
    rs_msg.frame_id = "/gnss";
    rs_msg.status = ros_msg.status.status;
    rs_msg.pos[0] = ros_msg.latitude;
    rs_msg.pos[1] = ros_msg.longitude;
    rs_msg.pos[2] = ros_msg.altitude;
    return rs_msg;
}
inline sensor_msgs::NavSatFix toRosMsg(const GnssMsg &rs_msg)
{
    sensor_msgs::NavSatFix ros_msg;
    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
    ros_msg.header.frame_id = rs_msg.parent_frame_id;
    ros_msg.header.seq = rs_msg.seq;
    ros_msg.status.status = rs_msg.status;
    ros_msg.latitude = rs_msg.pos[0];
    ros_msg.longitude = rs_msg.pos[1];
    ros_msg.altitude = rs_msg.pos[2];
    return ros_msg;
}
inline nav_msgs::Odometry toRosMsgOdom(const GnssMsg &rs_msg)
{
    nav_msgs::Odometry ros_msg;
    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
    ros_msg.header.frame_id = rs_msg.parent_frame_id;
    ros_msg.header.seq = rs_msg.seq;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(rs_msg.orien[0], rs_msg.orien[1], rs_msg.orien[2]);
    ros_msg.pose.pose.orientation = odom_quat;
    ros_msg.twist.twist.linear.x = rs_msg.linear_vel[0];
    ros_msg.twist.twist.linear.y = rs_msg.linear_vel[1];
    ros_msg.twist.twist.linear.z = rs_msg.linear_vel[2];
    return ros_msg;
}
inline OdomMsg toRsMsg(const nav_msgs::Odometry &ros_msg)
{
    OdomMsg rs_msg;
    rs_msg.seq = ros_msg.header.seq;
    rs_msg.timestamp = ros_msg.header.stamp.toSec();
    rs_msg.parent_frame_id = ros_msg.header.frame_id;
    rs_msg.frame_id = "/odom";
    rs_msg.linear_vel[0] = ros_msg.twist.twist.linear.x;
    rs_msg.linear_vel[1] = ros_msg.twist.twist.linear.y;
    rs_msg.linear_vel[2] = ros_msg.twist.twist.linear.z;
    return rs_msg;
}
inline nav_msgs::Odometry toRosMsg(const OdomMsg &rs_msg)
{
    nav_msgs::Odometry ros_msg;
    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
    ros_msg.header.frame_id = rs_msg.parent_frame_id;
    ros_msg.header.seq = rs_msg.seq;
    ros_msg.twist.twist.linear.x = rs_msg.linear_vel[0];
    ros_msg.twist.twist.linear.y = rs_msg.linear_vel[1];
    ros_msg.twist.twist.linear.z = rs_msg.linear_vel[2];
    return ros_msg;
}

inline LidarPointsMsg toRsMsg(const sensor_msgs::PointCloud2 &ros_msg, std::string frame_id = "/rslidar_points")
{
    LidarPointsMsg rs_msg;
    PointCloud *ptr_tmp = new PointCloud();
    pcl::fromROSMsg(ros_msg, *ptr_tmp);
    rs_msg.seq = ros_msg.header.seq;
    rs_msg.timestamp = ros_msg.header.stamp.toSec();
    rs_msg.parent_frame_id = ros_msg.header.frame_id;
    rs_msg.frame_id = frame_id;
    rs_msg.cloudPtr.reset(ptr_tmp);
    return rs_msg;
}
inline sensor_msgs::PointCloud2 toRosMsg(const LidarPointsMsg &rs_msg)
{
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(*rs_msg.cloudPtr, ros_msg);
    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
    ros_msg.header.frame_id = rs_msg.parent_frame_id;
    ros_msg.header.seq = rs_msg.seq;
    return ros_msg;
}
inline LidarPacketMsg toRsMsg(const rslidar_msgs::rslidarPacket &ros_msg)
{
    LidarPacketMsg rs_msg;
    rs_msg.timestamp = ros_msg.stamp.toSec();
    for (size_t i = 0; i < 1248; i++)
    {
        rs_msg.packet[i] = std::move(ros_msg.data[i]);
    }
    return rs_msg;
}
inline rslidar_msgs::rslidarPacket toRosMsg(const LidarPacketMsg &rs_msg)
{
    rslidar_msgs::rslidarPacket ros_msg;
    ros_msg.stamp = ros_msg.stamp.fromSec(rs_msg.timestamp);

    for (size_t i = 0; i < 1248; i++)
    {
        ros_msg.data[i] = std::move(rs_msg.packet[i]);
    }
    return ros_msg;
}
inline LidarScanMsg toRsMsg(const rslidar_msgs::rslidarScan &ros_msg)
{
    LidarScanMsg rs_msg;
    rs_msg.seq = ros_msg.header.seq;
    rs_msg.timestamp = ros_msg.header.stamp.toSec();
    rs_msg.parent_frame_id = ros_msg.header.frame_id;
    rs_msg.frame_id = "/lidar_scan";

    for (uint32_t i = 0; i < ros_msg.packets.size(); i++)
    {
        LidarPacketMsg tmp = toRsMsg(ros_msg.packets[i]);
        rs_msg.packets.emplace_back(tmp);
    }
    return rs_msg;
}
inline rslidar_msgs::rslidarScan toRosMsg(const LidarScanMsg &rs_msg)
{
    rslidar_msgs::rslidarScan ros_msg;
    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
    ros_msg.header.frame_id = rs_msg.parent_frame_id;
    ros_msg.header.seq = rs_msg.seq;
    for (uint32_t i = 0; i < rs_msg.packets.size(); i++)
    {
        rslidar_msgs::rslidarPacket tmp = toRosMsg(rs_msg.packets[i]);
        ros_msg.packets.emplace_back(tmp);
    }
    return ros_msg;
}

inline nav_msgs::Odometry toRosMsg(const VehicleStateMsg &rs_msg)
{
    nav_msgs::Odometry ros_msg;
    ros_msg.header.stamp.fromSec(rs_msg.timestamp);
    ros_msg.header.frame_id = rs_msg.parent_frame_id;
    ros_msg.header.seq = rs_msg.seq;
    ros_msg.child_frame_id = rs_msg.frame_id;
    ros_msg.pose.pose.position.x = rs_msg.pos[0];
    ros_msg.pose.pose.position.y = rs_msg.pos[1];
    ros_msg.pose.pose.position.z = rs_msg.pos[2];
    std::array<double, 4> quaternion = rs_msg.toQuaternion(rs_msg.orien);
    ros_msg.pose.pose.orientation.x = quaternion[0];
    ros_msg.pose.pose.orientation.y = quaternion[1];
    ros_msg.pose.pose.orientation.z = quaternion[2];
    ros_msg.pose.pose.orientation.w = quaternion[3];
    ros_msg.pose.covariance[0] = rs_msg.pos_cov[0];
    ros_msg.pose.covariance[1] = rs_msg.pos_cov[1];
    ros_msg.pose.covariance[2] = rs_msg.pos_cov[2];
    ros_msg.pose.covariance[6] = rs_msg.pos_cov[3];
    ros_msg.pose.covariance[7] = rs_msg.pos_cov[4];
    ros_msg.pose.covariance[8] = rs_msg.pos_cov[5];
    ros_msg.pose.covariance[12] = rs_msg.pos_cov[6];
    ros_msg.pose.covariance[13] = rs_msg.pos_cov[7];
    ros_msg.pose.covariance[14] = rs_msg.pos_cov[8];
    ros_msg.pose.covariance[21] = rs_msg.orien_cov[0];
    ros_msg.pose.covariance[22] = rs_msg.orien_cov[1];
    ros_msg.pose.covariance[23] = rs_msg.orien_cov[2];
    ros_msg.pose.covariance[27] = rs_msg.orien_cov[3];
    ros_msg.pose.covariance[28] = rs_msg.orien_cov[4];
    ros_msg.pose.covariance[29] = rs_msg.orien_cov[5];
    ros_msg.pose.covariance[33] = rs_msg.orien_cov[6];
    ros_msg.pose.covariance[34] = rs_msg.orien_cov[7];
    ros_msg.pose.covariance[35] = rs_msg.orien_cov[8];
    ros_msg.twist.covariance[0] = rs_msg.linear_vel_cov[0];
    ros_msg.twist.covariance[1] = rs_msg.linear_vel_cov[1];
    ros_msg.twist.covariance[2] = rs_msg.linear_vel_cov[2];
    ros_msg.twist.covariance[6] = rs_msg.linear_vel_cov[3];
    ros_msg.twist.covariance[7] = rs_msg.linear_vel_cov[4];
    ros_msg.twist.covariance[8] = rs_msg.linear_vel_cov[5];
    ros_msg.twist.covariance[12] = rs_msg.linear_vel_cov[6];
    ros_msg.twist.covariance[13] = rs_msg.linear_vel_cov[7];
    ros_msg.twist.covariance[14] = rs_msg.linear_vel_cov[8];
    ros_msg.twist.covariance[21] = rs_msg.acc_cov[0];
    ros_msg.twist.covariance[22] = rs_msg.acc_cov[1];
    ros_msg.twist.covariance[23] = rs_msg.acc_cov[2];
    ros_msg.twist.covariance[27] = rs_msg.acc_cov[3];
    ros_msg.twist.covariance[28] = rs_msg.acc_cov[4];
    ros_msg.twist.covariance[29] = rs_msg.acc_cov[5];
    ros_msg.twist.covariance[33] = rs_msg.acc_cov[6];
    ros_msg.twist.covariance[34] = rs_msg.acc_cov[7];
    ros_msg.twist.covariance[35] = rs_msg.acc_cov[8];
    ros_msg.twist.twist.linear.x = rs_msg.linear_vel[0];
    ros_msg.twist.twist.linear.y = rs_msg.linear_vel[1];
    ros_msg.twist.twist.linear.z = rs_msg.linear_vel[2];
    ros_msg.twist.twist.angular.x = rs_msg.angular_vel[0];
    ros_msg.twist.twist.angular.y = rs_msg.angular_vel[1];
    ros_msg.twist.twist.angular.z = rs_msg.angular_vel[2];

    return ros_msg;
}

inline geometry_msgs::TransformStamped toRosTf(const VehicleStateMsg &rs_msg)
{
    geometry_msgs::TransformStamped tf;
    tf.header.stamp.fromSec(rs_msg.timestamp);
    tf.header.frame_id = rs_msg.parent_frame_id;
    tf.header.seq = rs_msg.seq;
    tf.child_frame_id = rs_msg.frame_id;
    tf.transform.translation.x = rs_msg.pos[0];
    tf.transform.translation.y = rs_msg.pos[1];
    tf.transform.translation.z = rs_msg.pos[2];

    std::array<double, 4> quaternion = rs_msg.toQuaternion(rs_msg.orien);
    tf.transform.rotation.x = quaternion[0];
    tf.transform.rotation.y = quaternion[1];
    tf.transform.rotation.z = quaternion[2];
    tf.transform.rotation.w = quaternion[3];

    return tf;
}

inline void toRosMsg(const VehicleStateMsg &rs_msg, nav_msgs::Path &path)
{
    geometry_msgs::PoseStamped posestamp;
    posestamp.header.stamp.fromSec(rs_msg.timestamp);
    posestamp.header.seq = rs_msg.seq;
    posestamp.header.frame_id = rs_msg.parent_frame_id;
    posestamp.pose.position.x = rs_msg.pos[0];
    posestamp.pose.position.y = rs_msg.pos[1];
    posestamp.pose.position.z = rs_msg.pos[2];
    std::array<double, 4> quaternion = rs_msg.toQuaternion(rs_msg.orien);
    posestamp.pose.orientation.x = quaternion[0];
    posestamp.pose.orientation.y = quaternion[1];
    posestamp.pose.orientation.z = quaternion[2];
    posestamp.pose.orientation.w = quaternion[3];
    path.header.stamp.fromSec(rs_msg.timestamp);
    path.header.frame_id = rs_msg.parent_frame_id;
    path.header.seq = rs_msg.seq;
    path.poses.push_back(posestamp);
    if (path.poses.size() > 3000)
    {
        path.poses.erase(path.poses.begin(),
                         path.poses.begin() + (path.poses.size() - 3000));
    }
}

inline void toRosMsg(const GridMap &map, nav_msgs::OccupancyGrid &occu_grid)
{
    occu_grid.header.frame_id = map.frame_id;
    occu_grid.info.resolution = map.resolution;
    occu_grid.info.width = map.width;
    occu_grid.info.height = map.height;
    occu_grid.info.origin.position.x = map.origin.at(0);
    occu_grid.info.origin.position.y = map.origin.at(1);
    occu_grid.info.origin.orientation.x = map.orientation.at(0);
    occu_grid.info.origin.orientation.y = map.orientation.at(1);
    occu_grid.info.origin.orientation.z = map.orientation.at(2);
    occu_grid.info.origin.orientation.w = map.orientation.at(3);

    for (auto &grid : map.grids)
        occu_grid.data.emplace_back(grid);
}

} // namespace common

} // namespace robosense
#endif // ROS_FOUND
