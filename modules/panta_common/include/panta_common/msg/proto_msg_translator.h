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
#include <panta_common/msg/rs_msg/imu_msg.h>
#include <panta_common/msg/rs_msg/gnss_msg.h>
#include <panta_common/msg/rs_msg/odom_msg.h>
#include <panta_common/msg/rs_msg/lidar_points_msg.h>
#include <panta_common/msg/rs_msg/lidar_packet_msg.h>
#include <panta_common/msg/rs_msg/lidar_scan_msg.h>
#include <panta_common/msg/rs_msg/vehiclestate_msg.h>
#include <panta_common/msg/rs_msg/obstacle_msg.h>
#include <panta_common/msg/rs_msg/freespace_msg.h>
#include <panta_common/msg/rs_msg/grid_map_msg.h>
#include <panta_common/msg/proto_msg/Proto_msg.Imu.pb.h>
#include <panta_common/msg/proto_msg/Proto_msg.Gnss.pb.h>
#include <panta_common/msg/proto_msg/Proto_msg.Odom.pb.h>
#include <panta_common/msg/proto_msg/Proto_msg.LidarPoints.pb.h>
#include <panta_common/msg/proto_msg/Proto_msg.VehicleState.pb.h>
#include <panta_common/msg/proto_msg/Proto_msg.Percept.pb.h>
#include <panta_common/msg/proto_msg/Proto_msg.GridMap.pb.h>
#include <panta_common/msg/proto_msg/Proto_msg.LidarPacket.pb.h>
#include <panta_common/msg/proto_msg/Proto_msg.LidarScan.pb.h>
#include <pcl/io/io.h>
namespace robosense
{
namespace common
{

/************************************************************************/
/**Translation functions between robosense message and protobuf message**/
/************************************************************************/

inline Proto_msg::FreeSpaces toProtoMsg(const FreeSpaceMsg::Ptr &rs_msg)
{

    Proto_msg::FreeSpaces proto_msg;
    proto_msg.set_timestamp(rs_msg->timestamp);
    proto_msg.set_seq(rs_msg->seq);
    proto_msg.set_frame_id(rs_msg->frame_id);
    proto_msg.set_parent_frame_id(rs_msg->parent_frame_id);

    for (size_t k = 0; k < rs_msg->freespaces.size(); ++k)
    {
        Proto_msg::FreeSpace *pb_fs = proto_msg.add_freespaces();
        const FreeSpace::Ptr &fs = rs_msg->freespaces[k];

        pb_fs->set_timestamp(fs->timestamp);
        pb_fs->set_device_code(fs->device_code);

        pb_fs->set_distance(fs->distance);
        pb_fs->set_yaw_angle(fs->yaw_angle);
        pb_fs->set_free_prob(fs->free_prob);
    }

    return std::move(proto_msg);
}

inline FreeSpaceMsg::Ptr toRsMsg(const Proto_msg::FreeSpaces &proto_msg)
{
    FreeSpaceMsg::Ptr rs_msg(new FreeSpaceMsg);
    rs_msg->timestamp = proto_msg.timestamp();
    rs_msg->seq = proto_msg.seq();
    rs_msg->frame_id = proto_msg.frame_id();
    rs_msg->parent_frame_id = proto_msg.parent_frame_id();

    rs_msg->freespaces.resize(proto_msg.freespaces_size());
    for (int k = 0; k < proto_msg.freespaces_size(); ++k)
    {
        FreeSpace::Ptr &fs = rs_msg->freespaces[k];
        const Proto_msg::FreeSpace &pb_fs = proto_msg.freespaces(k);

        fs->timestamp = pb_fs.timestamp();
        fs->device_code = pb_fs.device_code();

        fs->distance = pb_fs.distance();
        fs->yaw_angle = pb_fs.yaw_angle();
        fs->free_prob = pb_fs.free_prob();
    }

    return std::move(rs_msg);
}

inline Obstacle::Ptr toRsMsg(const Proto_msg::Obstacle &pb_obj)
{
    Obstacle::Ptr obj(new Obstacle);
    obj->timestamp = pb_obj.timestamp();
    obj->device_code = pb_obj.device_code();

    obj->id = pb_obj.id();

    obj->anchor.x = pb_obj.anchor().x();
    obj->anchor.y = pb_obj.anchor().y();
    obj->anchor.z = pb_obj.anchor().z();

    obj->geo_center.x = pb_obj.geo_center().x();
    obj->geo_center.y = pb_obj.geo_center().y();
    obj->geo_center.z = pb_obj.geo_center().z();

    obj->geo_size.x = pb_obj.geo_size().x();
    obj->geo_size.y = pb_obj.geo_size().y();
    obj->geo_size.z = pb_obj.geo_size().z();

    obj->geo_direction.x = pb_obj.geo_direction().x();
    obj->geo_direction.y = pb_obj.geo_direction().y();
    obj->geo_direction.z = pb_obj.geo_direction().z();

    obj->polygon.clear();
    for (int i = 0; i < pb_obj.polygon_size(); ++i)
    {
        const auto &p = pb_obj.polygon(i);
        robosense::common::Point3 point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
        obj->polygon.emplace_back(point);
    }

    obj->detect_confidence = pb_obj.detect_confidence();

    obj->nearest_point.x = pb_obj.nearest_point().x();
    obj->nearest_point.y = pb_obj.nearest_point().y();
    obj->nearest_point.z = pb_obj.nearest_point().z();

    obj->left_point.x = pb_obj.left_point().x();
    obj->left_point.y = pb_obj.left_point().y();
    obj->left_point.z = pb_obj.left_point().z();

    obj->right_point.x = pb_obj.right_point().x();
    obj->right_point.y = pb_obj.right_point().y();
    obj->right_point.z = pb_obj.right_point().z();

    obj->distance = pb_obj.distance();
    obj->yaw = pb_obj.yaw();
    obj->point_num = pb_obj.point_num();

    obj->type = static_cast<ObjectType>(pb_obj.type());
    obj->type_confidence = pb_obj.type_confidence();
    obj->latent_types.reserve(pb_obj.latent_types_size());
    for (int i = 0; i < pb_obj.latent_types_size(); ++i)
    {
        const auto &t = pb_obj.latent_types(i);
        obj->latent_types.emplace_back(t);
    }
    obj->motion_state = static_cast<MotionType>(pb_obj.motion_state());

    obj->is_track_converged = pb_obj.is_track_converged();
    obj->tracker_id = pb_obj.tracker_id();

    obj->velocity.x = pb_obj.velocity().x();
    obj->velocity.y = pb_obj.velocity().y();
    obj->velocity.z = pb_obj.velocity().z();

    for (int i = 0; i < pb_obj.velocity_cov_size() && i < 9; ++i)
    {
        const auto &p = pb_obj.velocity_cov(i);
        obj->velocity_cov[i] = p;
    }

    obj->velocity_uncertainty = pb_obj.velocity_uncertainty();

    obj->ave_velocity.x = pb_obj.ave_velocity().x();
    obj->ave_velocity.y = pb_obj.ave_velocity().y();
    obj->ave_velocity.z = pb_obj.ave_velocity().z();

    obj->acceleration.x = pb_obj.acceleration().x();
    obj->acceleration.y = pb_obj.acceleration().y();
    obj->acceleration.z = pb_obj.acceleration().z();

    for (int i = 0; i < pb_obj.acceleration_cov_size() && i < 9; ++i)
    {
        const auto &p = pb_obj.acceleration_cov(i);
        obj->acceleration_cov[i] = p;
    }

    obj->acceleration_uncertainty = pb_obj.acceleration_uncertainty();

    obj->ave_acceleration.x = pb_obj.ave_acceleration().x();
    obj->ave_acceleration.y = pb_obj.ave_acceleration().y();
    obj->ave_acceleration.z = pb_obj.ave_acceleration().z();

    obj->angle_velocity = pb_obj.angle_velocity();
    obj->angle_velocity_cov = pb_obj.angle_velocity_cov();
    obj->angle_velocity_uncertainty = pb_obj.angle_velocity_uncertainty();
    obj->ave_angle_velocity = pb_obj.ave_angle_velocity();

    obj->asso_quality = pb_obj.asso_quality();
    obj->tracker_quality = pb_obj.tracker_quality();
    obj->tracking_time = pb_obj.tracking_time();

    return std::move(obj);
}

inline ObstacleMsg::Ptr toRsMsg(const Proto_msg::Obstacles &proto_msg)
{
    ObstacleMsg::Ptr rs_msg(new ObstacleMsg);
    rs_msg->timestamp = proto_msg.timestamp();
    rs_msg->seq = proto_msg.seq();
    rs_msg->frame_id = proto_msg.frame_id();
    rs_msg->parent_frame_id = proto_msg.parent_frame_id();

    rs_msg->obstacles.resize(proto_msg.obstacles_size());
    for (int k = 0; k < proto_msg.obstacles_size(); ++k)
    {
        Obstacle::Ptr &obj = rs_msg->obstacles[k];
        const Proto_msg::Obstacle &pb_obj = proto_msg.obstacles(k);

        obj = toRsMsg(pb_obj);
    }

    return std::move(rs_msg);
}

inline Proto_msg::Obstacle toProtoMsg(const Obstacle::Ptr &obj)
{
    Proto_msg::Obstacle pb_obj;

    pb_obj.set_timestamp(obj->timestamp);
    pb_obj.set_device_code(obj->device_code);

    pb_obj.set_id(obj->id);

    Proto_msg::Point3 *obj_anchor = pb_obj.mutable_anchor();
    obj_anchor->set_x(obj->anchor.x);
    obj_anchor->set_y(obj->anchor.y);
    obj_anchor->set_z(obj->anchor.z);

    Proto_msg::Point3 *obj_center = pb_obj.mutable_geo_center();
    obj_center->set_x(obj->geo_center.x);
    obj_center->set_y(obj->geo_center.y);
    obj_center->set_z(obj->geo_center.z);

    Proto_msg::Point3 *obj_size = pb_obj.mutable_geo_size();
    obj_size->set_x(obj->geo_size.x);
    obj_size->set_y(obj->geo_size.y);
    obj_size->set_z(obj->geo_size.z);

    Proto_msg::Point3 *obj_dir = pb_obj.mutable_geo_direction();
    obj_dir->set_x(obj->geo_direction.x);
    obj_dir->set_y(obj->geo_direction.y);
    obj_dir->set_z(obj->geo_direction.z);

    for (size_t i = 0; i < obj->polygon.size(); ++i)
    {
        Proto_msg::Point3 *p = pb_obj.add_polygon();
        p->set_x(obj->polygon[i].x);
        p->set_y(obj->polygon[i].y);
        p->set_z(obj->polygon[i].z);
    }

    pb_obj.set_detect_confidence(obj->detect_confidence);

    Proto_msg::Point3 *obj_nearest = pb_obj.mutable_nearest_point();
    obj_nearest->set_x(obj->nearest_point.x);
    obj_nearest->set_y(obj->nearest_point.y);
    obj_nearest->set_z(obj->nearest_point.z);

    Proto_msg::Point3 *obj_left = pb_obj.mutable_left_point();
    obj_left->set_x(obj->left_point.x);
    obj_left->set_y(obj->left_point.y);
    obj_left->set_z(obj->left_point.z);

    Proto_msg::Point3 *obj_right = pb_obj.mutable_right_point();
    obj_right->set_x(obj->right_point.x);
    obj_right->set_y(obj->right_point.y);
    obj_right->set_z(obj->right_point.z);

    pb_obj.set_distance(obj->distance);
    pb_obj.set_yaw(obj->yaw);
    pb_obj.set_point_num(obj->point_num);

    pb_obj.set_type(static_cast<Proto_msg::Obstacle::Type>(obj->type));
    pb_obj.set_type_confidence(obj->type_confidence);
    for (size_t i = 0; i < obj->latent_types.size(); ++i)
    {
        pb_obj.add_latent_types(obj->latent_types[i]);
    }
    pb_obj.set_motion_state(static_cast<Proto_msg::Obstacle::MotionType>(obj->motion_state));

    pb_obj.set_is_track_converged(obj->is_track_converged);
    pb_obj.set_tracker_id(obj->tracker_id);

    Proto_msg::Point3 *obj_vel = pb_obj.mutable_velocity();
    obj_vel->set_x(obj->velocity.x);
    obj_vel->set_y(obj->velocity.y);
    obj_vel->set_z(obj->velocity.z);

    for (size_t i = 0; i < obj->velocity_cov.size(); ++i)
    {
        pb_obj.add_velocity_cov(obj->velocity_cov[i]);
    }

    pb_obj.set_velocity_uncertainty(obj->velocity_uncertainty);

    Proto_msg::Point3 *obj_ave_vel = pb_obj.mutable_ave_velocity();
    obj_ave_vel->set_x(obj->ave_velocity.x);
    obj_ave_vel->set_y(obj->ave_velocity.y);
    obj_ave_vel->set_z(obj->ave_velocity.z);

    Proto_msg::Point3 *obj_acc = pb_obj.mutable_acceleration();
    obj_acc->set_x(obj->acceleration.x);
    obj_acc->set_y(obj->acceleration.y);
    obj_acc->set_z(obj->acceleration.z);

    for (size_t i = 0; i < obj->acceleration_cov.size(); ++i)
    {
        pb_obj.add_acceleration_cov(obj->acceleration_cov[i]);
    }

    pb_obj.set_acceleration_uncertainty(obj->acceleration_uncertainty);

    Proto_msg::Point3 *obj_ave_acc = pb_obj.mutable_ave_acceleration();
    obj_ave_acc->set_x(obj->ave_acceleration.x);
    obj_ave_acc->set_y(obj->ave_acceleration.y);
    obj_ave_acc->set_z(obj->ave_acceleration.z);

    pb_obj.set_angle_velocity(obj->angle_velocity);
    pb_obj.set_angle_velocity_cov(obj->angle_velocity_cov);
    pb_obj.set_angle_velocity_uncertainty(obj->angle_velocity_uncertainty);
    pb_obj.set_ave_angle_velocity(obj->ave_angle_velocity);

    pb_obj.set_asso_quality(obj->asso_quality);
    pb_obj.set_tracker_quality(obj->tracker_quality);
    pb_obj.set_tracking_time(obj->tracking_time);

    return std::move(pb_obj);
}

inline Proto_msg::Obstacles toProtoMsg(const ObstacleMsg::Ptr &rs_msg)
{
    Proto_msg::Obstacles proto_msg;
    proto_msg.set_timestamp(rs_msg->timestamp);
    proto_msg.set_seq(rs_msg->seq);
    proto_msg.set_frame_id(rs_msg->frame_id);
    proto_msg.set_parent_frame_id(rs_msg->parent_frame_id);

    for (size_t k = 0; k < rs_msg->obstacles.size(); ++k)
    {
        Proto_msg::Obstacle *pb_obj = proto_msg.add_obstacles();
        const Obstacle::Ptr &obj = rs_msg->obstacles[k];

        *pb_obj = toProtoMsg(obj);
    }

    return std::move(proto_msg);
}

inline Proto_msg::Imu toProtoMsg(const ImuMsg &rs_msg)
{
    Proto_msg::Imu proto_msg;
    proto_msg.set_timestamp(rs_msg.timestamp);
    proto_msg.set_seq(rs_msg.seq);
    proto_msg.set_parent_frame_id(rs_msg.parent_frame_id);
    proto_msg.set_frame_id(rs_msg.frame_id);
    proto_msg.add_orien(rs_msg.orien[0]);
    proto_msg.add_orien(rs_msg.orien[1]);
    proto_msg.add_orien(rs_msg.orien[2]);
    proto_msg.add_angular_vel(rs_msg.angular_vel[0]);
    proto_msg.add_angular_vel(rs_msg.angular_vel[1]);
    proto_msg.add_angular_vel(rs_msg.angular_vel[2]);
    proto_msg.add_acc(rs_msg.acc[0]);
    proto_msg.add_acc(rs_msg.acc[1]);
    proto_msg.add_acc(rs_msg.acc[2]);
    return std::move(proto_msg);
}
inline ImuMsg toRsMsg(const Proto_msg::Imu &proto_msg)
{
    ImuMsg rs_msg;
    rs_msg.timestamp = proto_msg.timestamp();
    rs_msg.seq = proto_msg.seq();
    rs_msg.parent_frame_id = proto_msg.parent_frame_id();
    rs_msg.frame_id = proto_msg.frame_id();
    rs_msg.orien[0] = proto_msg.orien(0);
    rs_msg.orien[1] = proto_msg.orien(1);
    rs_msg.orien[2] = proto_msg.orien(2);
    rs_msg.angular_vel[0] = proto_msg.angular_vel(0);
    rs_msg.angular_vel[1] = proto_msg.angular_vel(1);
    rs_msg.angular_vel[2] = proto_msg.angular_vel(2);
    rs_msg.acc[0] = proto_msg.acc(0);
    rs_msg.acc[1] = proto_msg.acc(1);
    rs_msg.acc[2] = proto_msg.acc(2);
    return std::move(rs_msg);
}

inline Proto_msg::Gnss toProtoMsg(const GnssMsg &rs_msg)
{
    Proto_msg::Gnss proto_msg;
    proto_msg.set_timestamp(rs_msg.timestamp);
    proto_msg.set_seq(rs_msg.seq);
    proto_msg.set_parent_frame_id(rs_msg.parent_frame_id);
    proto_msg.set_frame_id(rs_msg.frame_id);
    proto_msg.set_status(rs_msg.status);
    proto_msg.add_pos(rs_msg.pos[0]);
    proto_msg.add_pos(rs_msg.pos[1]);
    proto_msg.add_pos(rs_msg.pos[2]);
    proto_msg.add_orien(rs_msg.orien[0]);
    proto_msg.add_orien(rs_msg.orien[1]);
    proto_msg.add_orien(rs_msg.orien[2]);
    proto_msg.add_linear_vel(rs_msg.linear_vel[0]);
    proto_msg.add_linear_vel(rs_msg.linear_vel[1]);
    proto_msg.add_linear_vel(rs_msg.linear_vel[2]);
    return std::move(proto_msg);
}
inline GnssMsg toRsMsg(const Proto_msg::Gnss &proto_msg)
{
    GnssMsg rs_msg;
    rs_msg.timestamp = proto_msg.timestamp();
    rs_msg.seq = proto_msg.seq();
    rs_msg.parent_frame_id = proto_msg.parent_frame_id();
    rs_msg.frame_id = proto_msg.frame_id();
    rs_msg.status = proto_msg.status();
    rs_msg.pos[0] = proto_msg.pos(0);
    rs_msg.pos[1] = proto_msg.pos(1);
    rs_msg.pos[2] = proto_msg.pos(2);
    rs_msg.orien[0] = proto_msg.orien(0);
    rs_msg.orien[1] = proto_msg.orien(1);
    rs_msg.orien[2] = proto_msg.orien(2);
    rs_msg.linear_vel[0] = proto_msg.linear_vel(0);
    rs_msg.linear_vel[1] = proto_msg.linear_vel(1);
    rs_msg.linear_vel[2] = proto_msg.linear_vel(2);
    return std::move(rs_msg);
}

inline Proto_msg::Odom toProtoMsg(const OdomMsg &rs_msg)
{
    Proto_msg::Odom proto_msg;
    proto_msg.set_timestamp(rs_msg.timestamp);
    proto_msg.set_seq(rs_msg.seq);
    proto_msg.set_parent_frame_id(rs_msg.parent_frame_id);
    proto_msg.set_frame_id(rs_msg.frame_id);
    proto_msg.add_linear_vel(rs_msg.linear_vel[0]);
    proto_msg.add_linear_vel(rs_msg.linear_vel[1]);
    proto_msg.add_linear_vel(rs_msg.linear_vel[2]);
    return std::move(proto_msg);
}
inline OdomMsg toRsMsg(const Proto_msg::Odom &proto_msg)
{
    OdomMsg rs_msg;
    rs_msg.timestamp = proto_msg.timestamp();
    rs_msg.seq = proto_msg.seq();
    rs_msg.parent_frame_id = proto_msg.parent_frame_id();
    rs_msg.frame_id = proto_msg.frame_id();
    rs_msg.linear_vel[0] = proto_msg.linear_vel(0);
    rs_msg.linear_vel[1] = proto_msg.linear_vel(1);
    rs_msg.linear_vel[2] = proto_msg.linear_vel(2);
    return std::move(rs_msg);
}

inline Proto_msg::LidarPoints toProtoMsg(const LidarPointsMsg &rs_msg)
{
    Proto_msg::LidarPoints proto_msg;
    proto_msg.set_timestamp(rs_msg.timestamp);
    proto_msg.set_seq(rs_msg.seq);
    proto_msg.set_parent_frame_id(rs_msg.parent_frame_id);
    proto_msg.set_frame_id(rs_msg.frame_id);
    proto_msg.set_motion_correct(rs_msg.motion_correct);
    proto_msg.set_height(rs_msg.height);
    proto_msg.set_width(rs_msg.width);
    proto_msg.set_is_dense(rs_msg.is_dense);
    proto_msg.set_is_transform(rs_msg.is_transform);
    proto_msg.set_lidar_model(rs_msg.lidar_model);
    proto_msg.set_points_type(rs_msg.points_type);

    for (size_t i = 0; i < rs_msg.cloudPtr->size(); i++)
    {
        proto_msg.add_data(rs_msg.cloudPtr->points[i].x);
        proto_msg.add_data(rs_msg.cloudPtr->points[i].y);
        proto_msg.add_data(rs_msg.cloudPtr->points[i].z);
        proto_msg.add_data(rs_msg.cloudPtr->points[i].intensity);
    }

    return std::move(proto_msg);
}

inline LidarPointsMsg toRsMsg(const Proto_msg::LidarPoints &proto_msg)
{
    LidarPointsMsg rs_msg;
    rs_msg.timestamp = proto_msg.timestamp();
    rs_msg.seq = proto_msg.seq();
    rs_msg.parent_frame_id = proto_msg.parent_frame_id();
    rs_msg.frame_id = proto_msg.frame_id();
    rs_msg.motion_correct = proto_msg.motion_correct();
    rs_msg.height = proto_msg.height();
    rs_msg.width = proto_msg.width();
    rs_msg.is_dense = proto_msg.is_dense();
    rs_msg.is_transform = proto_msg.is_transform();
    rs_msg.lidar_model = proto_msg.lidar_model();
    rs_msg.points_type = proto_msg.points_type();
    PointCloud *ptr_tmp = new PointCloud();
    for (int i = 0; i < proto_msg.data_size(); i += 4)
    {
        pcl::PointXYZI point;
        point.x = proto_msg.data(i);
        point.y = proto_msg.data(i + 1);
        point.z = proto_msg.data(i + 2);
        point.intensity = proto_msg.data(i + 3);
        ptr_tmp->push_back(point);
    }
    ptr_tmp->height = rs_msg.height;
    ptr_tmp->width = rs_msg.width;
    ptr_tmp->is_dense = rs_msg.is_dense;
    rs_msg.cloudPtr.reset(ptr_tmp);
    return rs_msg;
}

inline Proto_msg::LidarScan toProtoMsg(const LidarScanMsg &rs_msg)
{
    Proto_msg::LidarScan proto_msg;
    proto_msg.set_timestamp(rs_msg.timestamp);
    proto_msg.set_seq(rs_msg.seq);
    for (auto iter : rs_msg.packets)
    {
        void *data_ptr = malloc(1248);
        memcpy(data_ptr, iter.packet.data(), 1248);
        proto_msg.add_data(data_ptr, 1248);
        free(data_ptr);
    }
    return proto_msg;
}

inline LidarScanMsg toRsMsg(const Proto_msg::LidarScan &proto_msg)
{
    LidarScanMsg rs_msg;
    rs_msg.timestamp = proto_msg.timestamp();
    rs_msg.seq = proto_msg.seq();
    for (int i = 0; i < proto_msg.data_size(); i++)
    {
        std::string data_str = proto_msg.data(i);
        LidarPacketMsg tmp_pkt;
        memcpy(tmp_pkt.packet.data(), data_str.data(), data_str.size());
        rs_msg.packets.emplace_back(tmp_pkt);
    }
    return rs_msg;
}

inline Proto_msg::LidarPacket toProtoMsg(const LidarPacketMsg &rs_msg)
{
    Proto_msg::LidarPacket proto_msg;
    proto_msg.set_timestamp(rs_msg.timestamp);
    void *data_ptr = malloc(1248);
    memcpy(data_ptr, rs_msg.packet.data(), 1248);
    proto_msg.set_data(data_ptr, 1248);
    free(data_ptr);
    return proto_msg;
}

inline LidarPacketMsg toRsMsg(const Proto_msg::LidarPacket &proto_msg)
{
    LidarPacketMsg rs_msg;
    rs_msg.timestamp = proto_msg.timestamp();
    std::string data_str = proto_msg.data();
    memcpy(rs_msg.packet.data(), data_str.data(), data_str.size());
    return rs_msg;
}
inline VehicleStateMsg toRsMsg(const Proto_msg::VehicleState &proto_msg)
{
    VehicleStateMsg rs_msg;
    rs_msg.timestamp = proto_msg.timestamp();
    rs_msg.seq = proto_msg.seq();
    rs_msg.parent_frame_id = proto_msg.parent_frame_id();
    rs_msg.frame_id = proto_msg.frame_id();
    rs_msg.status = proto_msg.status();
    rs_msg.pos[0] = proto_msg.pos(0);
    rs_msg.pos[1] = proto_msg.pos(1);
    rs_msg.pos[2] = proto_msg.pos(2);
    rs_msg.pos_cov[0] = proto_msg.pos_cov(0);
    rs_msg.pos_cov[1] = proto_msg.pos_cov(1);
    rs_msg.pos_cov[2] = proto_msg.pos_cov(2);
    rs_msg.pos_cov[3] = proto_msg.pos_cov(3);
    rs_msg.pos_cov[4] = proto_msg.pos_cov(4);
    rs_msg.pos_cov[5] = proto_msg.pos_cov(5);
    rs_msg.pos_cov[6] = proto_msg.pos_cov(6);
    rs_msg.pos_cov[7] = proto_msg.pos_cov(7);
    rs_msg.pos_cov[8] = proto_msg.pos_cov(8);
    rs_msg.orien[0] = proto_msg.orien(0);
    rs_msg.orien[1] = proto_msg.orien(1);
    rs_msg.orien[2] = proto_msg.orien(2);
    rs_msg.orien_cov[0] = proto_msg.orien_cov(0);
    rs_msg.orien_cov[1] = proto_msg.orien_cov(1);
    rs_msg.orien_cov[2] = proto_msg.orien_cov(2);
    rs_msg.orien_cov[3] = proto_msg.orien_cov(3);
    rs_msg.orien_cov[4] = proto_msg.orien_cov(4);
    rs_msg.orien_cov[5] = proto_msg.orien_cov(5);
    rs_msg.orien_cov[6] = proto_msg.orien_cov(6);
    rs_msg.orien_cov[7] = proto_msg.orien_cov(7);
    rs_msg.orien_cov[8] = proto_msg.orien_cov(8);
    rs_msg.angular_vel[0] = proto_msg.angular_vel(0);
    rs_msg.angular_vel[1] = proto_msg.angular_vel(1);
    rs_msg.angular_vel[2] = proto_msg.angular_vel(2);
    rs_msg.angular_vel_cov[0] = proto_msg.angular_vel_cov(0);
    rs_msg.angular_vel_cov[1] = proto_msg.angular_vel_cov(1);
    rs_msg.angular_vel_cov[2] = proto_msg.angular_vel_cov(2);
    rs_msg.angular_vel_cov[3] = proto_msg.angular_vel_cov(3);
    rs_msg.angular_vel_cov[4] = proto_msg.angular_vel_cov(4);
    rs_msg.angular_vel_cov[5] = proto_msg.angular_vel_cov(5);
    rs_msg.angular_vel_cov[6] = proto_msg.angular_vel_cov(6);
    rs_msg.angular_vel_cov[7] = proto_msg.angular_vel_cov(7);
    rs_msg.angular_vel_cov[8] = proto_msg.angular_vel_cov(8);

    rs_msg.linear_vel[0] = proto_msg.linear_vel(0);
    rs_msg.linear_vel[1] = proto_msg.linear_vel(1);
    rs_msg.linear_vel[2] = proto_msg.linear_vel(2);
    rs_msg.linear_vel_cov[0] = proto_msg.linear_vel_cov(0);
    rs_msg.linear_vel_cov[1] = proto_msg.linear_vel_cov(1);
    rs_msg.linear_vel_cov[2] = proto_msg.linear_vel_cov(2);
    rs_msg.linear_vel_cov[3] = proto_msg.linear_vel_cov(3);
    rs_msg.linear_vel_cov[4] = proto_msg.linear_vel_cov(4);
    rs_msg.linear_vel_cov[5] = proto_msg.linear_vel_cov(5);
    rs_msg.linear_vel_cov[6] = proto_msg.linear_vel_cov(6);
    rs_msg.linear_vel_cov[7] = proto_msg.linear_vel_cov(7);
    rs_msg.linear_vel_cov[8] = proto_msg.linear_vel_cov(8);

    rs_msg.acc[0] = proto_msg.acc(0);
    rs_msg.acc[1] = proto_msg.acc(1);
    rs_msg.acc[2] = proto_msg.acc(2);
    rs_msg.acc_cov[0] = proto_msg.acc_cov(0);
    rs_msg.acc_cov[1] = proto_msg.acc_cov(1);
    rs_msg.acc_cov[2] = proto_msg.acc_cov(2);
    rs_msg.acc_cov[3] = proto_msg.acc_cov(3);
    rs_msg.acc_cov[4] = proto_msg.acc_cov(4);
    rs_msg.acc_cov[5] = proto_msg.acc_cov(5);
    rs_msg.acc_cov[6] = proto_msg.acc_cov(6);
    rs_msg.acc_cov[7] = proto_msg.acc_cov(7);
    rs_msg.acc_cov[8] = proto_msg.acc_cov(8);

    return std::move(rs_msg);
}

inline Proto_msg::VehicleState toProtoMsg(const VehicleStateMsg &rs_msg)
{
    Proto_msg::VehicleState proto_msg;
    proto_msg.set_timestamp(rs_msg.timestamp);
    proto_msg.set_seq(rs_msg.seq);
    proto_msg.set_parent_frame_id(rs_msg.parent_frame_id);
    proto_msg.set_frame_id(rs_msg.frame_id);
    proto_msg.set_status(rs_msg.status);

    proto_msg.add_pos(rs_msg.pos[0]);
    proto_msg.add_pos(rs_msg.pos[1]);
    proto_msg.add_pos(rs_msg.pos[2]);
    proto_msg.add_pos_cov(rs_msg.pos_cov[0]);
    proto_msg.add_pos_cov(rs_msg.pos_cov[1]);
    proto_msg.add_pos_cov(rs_msg.pos_cov[2]);
    proto_msg.add_pos_cov(rs_msg.pos_cov[3]);
    proto_msg.add_pos_cov(rs_msg.pos_cov[4]);
    proto_msg.add_pos_cov(rs_msg.pos_cov[5]);
    proto_msg.add_pos_cov(rs_msg.pos_cov[6]);
    proto_msg.add_pos_cov(rs_msg.pos_cov[7]);
    proto_msg.add_pos_cov(rs_msg.pos_cov[8]);

    proto_msg.add_orien(rs_msg.orien[0]);
    proto_msg.add_orien(rs_msg.orien[1]);
    proto_msg.add_orien(rs_msg.orien[2]);
    proto_msg.add_orien_cov(rs_msg.orien_cov[0]);
    proto_msg.add_orien_cov(rs_msg.orien_cov[1]);
    proto_msg.add_orien_cov(rs_msg.orien_cov[2]);
    proto_msg.add_orien_cov(rs_msg.orien_cov[3]);
    proto_msg.add_orien_cov(rs_msg.orien_cov[4]);
    proto_msg.add_orien_cov(rs_msg.orien_cov[5]);
    proto_msg.add_orien_cov(rs_msg.orien_cov[6]);
    proto_msg.add_orien_cov(rs_msg.orien_cov[7]);
    proto_msg.add_orien_cov(rs_msg.orien_cov[8]);

    proto_msg.add_angular_vel(rs_msg.angular_vel[0]);
    proto_msg.add_angular_vel(rs_msg.angular_vel[1]);
    proto_msg.add_angular_vel(rs_msg.angular_vel[2]);
    proto_msg.add_angular_vel_cov(rs_msg.angular_vel_cov[0]);
    proto_msg.add_angular_vel_cov(rs_msg.angular_vel_cov[1]);
    proto_msg.add_angular_vel_cov(rs_msg.angular_vel_cov[2]);
    proto_msg.add_angular_vel_cov(rs_msg.angular_vel_cov[3]);
    proto_msg.add_angular_vel_cov(rs_msg.angular_vel_cov[4]);
    proto_msg.add_angular_vel_cov(rs_msg.angular_vel_cov[5]);
    proto_msg.add_angular_vel_cov(rs_msg.angular_vel_cov[6]);
    proto_msg.add_angular_vel_cov(rs_msg.angular_vel_cov[7]);
    proto_msg.add_angular_vel_cov(rs_msg.angular_vel_cov[8]);

    proto_msg.add_linear_vel(rs_msg.linear_vel[0]);
    proto_msg.add_linear_vel(rs_msg.linear_vel[1]);
    proto_msg.add_linear_vel(rs_msg.linear_vel[2]);
    proto_msg.add_linear_vel_cov(rs_msg.linear_vel_cov[0]);
    proto_msg.add_linear_vel_cov(rs_msg.linear_vel_cov[1]);
    proto_msg.add_linear_vel_cov(rs_msg.linear_vel_cov[2]);
    proto_msg.add_linear_vel_cov(rs_msg.linear_vel_cov[3]);
    proto_msg.add_linear_vel_cov(rs_msg.linear_vel_cov[4]);
    proto_msg.add_linear_vel_cov(rs_msg.linear_vel_cov[5]);
    proto_msg.add_linear_vel_cov(rs_msg.linear_vel_cov[6]);
    proto_msg.add_linear_vel_cov(rs_msg.linear_vel_cov[7]);
    proto_msg.add_linear_vel_cov(rs_msg.linear_vel_cov[8]);

    proto_msg.add_acc(rs_msg.acc[0]);
    proto_msg.add_acc(rs_msg.acc[1]);
    proto_msg.add_acc(rs_msg.acc[2]);
    proto_msg.add_acc_cov(rs_msg.acc_cov[0]);
    proto_msg.add_acc_cov(rs_msg.acc_cov[1]);
    proto_msg.add_acc_cov(rs_msg.acc_cov[2]);
    proto_msg.add_acc_cov(rs_msg.acc_cov[3]);
    proto_msg.add_acc_cov(rs_msg.acc_cov[4]);
    proto_msg.add_acc_cov(rs_msg.acc_cov[5]);
    proto_msg.add_acc_cov(rs_msg.acc_cov[6]);
    proto_msg.add_acc_cov(rs_msg.acc_cov[7]);
    proto_msg.add_acc_cov(rs_msg.acc_cov[8]);

    return std::move(proto_msg);
}

inline std::vector<Proto_msg::GridMap> toVecProto(const Proto_msg::GridMap &proto_msg, int N)
{
    std::vector<Proto_msg::GridMap> proto_vec;
    int grids_per_pkg = ceil((double)proto_msg.grids_size() / N);
    for (int i = 0; i < N; i++)
    {
        Proto_msg::GridMap tmp_msg;
        tmp_msg.set_resolution(proto_msg.resolution());
        tmp_msg.set_frame_id(proto_msg.frame_id());
        tmp_msg.set_width(proto_msg.width());
        tmp_msg.set_height(proto_msg.height());
        tmp_msg.add_origin(proto_msg.origin(0));
        tmp_msg.add_origin(proto_msg.origin(1));
        tmp_msg.add_orientation(proto_msg.orientation(0));
        tmp_msg.add_orientation(proto_msg.orientation(1));
        tmp_msg.add_orientation(proto_msg.orientation(2));
        tmp_msg.add_orientation(proto_msg.orientation(3));

        if ((i * grids_per_pkg) > proto_msg.grids_size())
        {
            continue;
        }

        for (int j = i * grids_per_pkg; j < (i + 1) * grids_per_pkg; j++)
        {
            if (j < proto_msg.grids_size())
            {
                tmp_msg.add_grids(proto_msg.grids(j));
            }
        }
        proto_vec.emplace_back(tmp_msg);
    }
    return std::move(proto_vec);
}

inline Proto_msg::GridMap toProtoMsg(const std::vector<Proto_msg::GridMap> &proto_vec)
{
    Proto_msg::GridMap proto_msg;
    proto_msg.set_resolution(proto_vec[0].resolution());
    proto_msg.set_frame_id(proto_vec[0].frame_id());
    proto_msg.set_width(proto_vec[0].width());
    proto_msg.set_height(proto_vec[0].height());
    proto_msg.add_origin(proto_vec[0].origin(0));
    proto_msg.add_origin(proto_vec[0].origin(1));
    proto_msg.add_orientation(proto_vec[0].orientation(0));
    proto_msg.add_orientation(proto_vec[0].orientation(1));
    proto_msg.add_orientation(proto_vec[0].orientation(2));
    proto_msg.add_orientation(proto_vec[0].orientation(3));

    for (size_t i = 0; i < proto_vec.size(); i++)
    {
        for (int j = 0; j < proto_vec[i].grids_size(); j++)
        {
            proto_msg.add_grids(proto_vec[i].grids(j));
        }
    }

    return std::move(proto_msg);
}

inline void toProtoMsg(const GridMap &map, Proto_msg::GridMap &occu_grid)
{
    occu_grid.set_frame_id(map.frame_id);
    occu_grid.set_resolution(map.resolution);
    occu_grid.set_width(map.width);
    occu_grid.set_height(map.height);
    occu_grid.add_origin(map.origin.at(0));
    occu_grid.add_origin(map.origin.at(1));
    occu_grid.add_orientation(map.orientation.at(0));
    occu_grid.add_orientation(map.orientation.at(1));
    occu_grid.add_orientation(map.orientation.at(2));
    occu_grid.add_orientation(map.orientation.at(3));

    for (auto &grid : map.grids)
        occu_grid.add_grids(grid);
}

inline void toRsMsg(GridMap &map, const Proto_msg::GridMap &occu_grid)
{
    map.frame_id = occu_grid.frame_id();
    map.resolution = occu_grid.resolution();
    map.width = occu_grid.width();
    map.height = occu_grid.height();
    map.origin.at(0) = occu_grid.origin(0);
    map.origin.at(1) = occu_grid.origin(1);
    map.orientation.at(0) = occu_grid.orientation(0);
    map.orientation.at(1) = occu_grid.orientation(1);
    map.orientation.at(2) = occu_grid.orientation(2);
    map.orientation.at(3) = occu_grid.orientation(3);

    for (int i = 0; i < occu_grid.grids_size(); i++)
    {
        map.grids.emplace_back(occu_grid.grids(i));
    }
}

} // namespace common

} // namespace robosense
#endif // PROTO_FOUND
