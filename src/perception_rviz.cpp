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
#include <cv_bridge/cv_bridge.h>
#include "panta_sdk_demo/perception_rviz.h"

#define USE_ClUSER_COLOR 0
#define TEXT_SCALE (0.8)

namespace robosense
{
namespace perception
{

DrawRviz::DrawRviz()
{

    marker_per_num_ = 0;
    generateColors(colors_, 100);
}

pcl::PCLHeader DrawRviz::convertROSHeader2PCLHeader(const std_msgs::Header &_header)
{
    pcl::PCLHeader header;
    header.seq = _header.seq;
    header.stamp = pcl_conversions::toPCL(_header.stamp);
    header.frame_id = _header.frame_id;

    return header;
}

std_msgs::Header DrawRviz::convertPCLHeader2ROSHeader(const pcl::PCLHeader &_header)
{
    std_msgs::Header header;
    header.seq = _header.seq;
    header.stamp = pcl_conversions::fromPCL(_header.stamp);
    header.frame_id = _header.frame_id;

    return header;
}

void DrawRviz::generateColors(std::vector<common::Point3> &colors, int num)
{
    colors.clear();
    colors.resize(num);
    for (int i = 0; i < num; ++i)
    {
        common::Point3 color;
        color.x = rand() % 255;
        color.y = rand() % 255;
        color.z = rand() % 255;

        color.x += 60;
        color.y += 60;
        color.z += 60;

        color.x = color.x < 255 ? color.x : 255;
        color.y = color.y < 255 ? color.y : 255;
        color.z = color.z < 255 ? color.z : 255;

        colors[i] = color;
    }
}

void DrawRviz::show_freespace(const ros::Publisher &pub_cloud, const std_msgs::Header &_header,
                              const std::vector<common::FreeSpace::Ptr> &freespaces, const float &range)
{   
    if(freespaces.empty())
    {
        return;
    }
    if (range <= 0)
    {
        std::cerr << "Draw Rviz Error: Illegal range set!!" << std::endl;
        throw;
    }
    float grid_size = 0.2;
    int height = (range * 2) / grid_size + 1;
    int width = (range * 2) / grid_size + 1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr free_sapce_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    free_sapce_cloud_ptr->resize(height * width);
    float hori_res = (freespaces.back()->yaw_angle - freespaces.front()->yaw_angle) / static_cast<float>(freespaces.size());
    int valid_cnt = 0;
    for (int row = 0; row < height; ++row)
    {
        for (int col = 0; col < width; ++col)
        {
            float center_x = static_cast<float>(col) * grid_size + grid_size / 2. - range;
            float center_y = static_cast<float>(row) * grid_size + grid_size / 2. - range;

            float tmp_range = sqrtf(center_x * center_x + center_y * center_y);
            float tmp_hori_angle = std::atan2(center_y, center_x) / M_PI * 180.;
            if (tmp_hori_angle < freespaces.front()->yaw_angle || tmp_hori_angle > freespaces.back()->yaw_angle)
            {
                continue;
            }
            int hori_index = (tmp_hori_angle - freespaces.front()->yaw_angle) / hori_res;
            if (tmp_range > freespaces[hori_index]->distance)
            {
                continue;
            }

            pcl::PointXYZ pt;
            pt.x = center_x;
            pt.y = center_y;
            pt.z = 0;

            free_sapce_cloud_ptr->points[valid_cnt++] = pt;
        }
    }
    free_sapce_cloud_ptr->resize(valid_cnt);
    free_sapce_cloud_ptr->header.frame_id = _header.frame_id;
    pub_cloud.publish(free_sapce_cloud_ptr);
}

void DrawRviz::show_percept(const ros::Publisher &pub_percept_info, const std_msgs::Header &_header,
                            const std::vector<common::Obstacle::Ptr> &percept_list)
{   
    if(percept_list.empty())
    {
        return;
    }
    visualization_msgs::MarkerArrayPtr marker_array(new visualization_msgs::MarkerArray);
    visualization_msgs::Marker marker_box;
    visualization_msgs::Marker marker_polygon;
    visualization_msgs::Marker marker_cube;
    visualization_msgs::Marker marker_text_box;
    visualization_msgs::Marker marker_text_label;
    visualization_msgs::Marker marker_text_track;
    visualization_msgs::Marker marker_velocity_arrow;
    visualization_msgs::Marker marker_acc_arrow;

    common::Point3 colors[5];
    colors[0] = common::Point3(0.6, 0.5, 0.6); //淡紫色 background
    colors[1] = common::Point3(1, 1, 0);       //黄色 pedestrian
    colors[2] = common::Point3(0, 1, 1);       //青色 bike
    colors[3] = common::Point3(0.7, 0, 0);     //红色 car
    colors[4] = common::Point3(0, 0, 1);       //蓝色 truck

    std::string info[5] = {"unknow", "ped", "bike", "car", "truck"};

    marker_text_box.header = _header;
    marker_text_box.ns = "box_info";
    marker_text_box.scale.x = TEXT_SCALE;
    marker_text_box.scale.y = TEXT_SCALE;
    marker_text_box.scale.z = TEXT_SCALE;
    marker_text_box.color.r = 0.7f;
    marker_text_box.color.g = 0.5f;
    marker_text_box.color.b = 0.0f;
    marker_text_box.color.a = 0.8f;

    marker_text_track.header = _header;
    marker_text_track.ns = "track_info";
    marker_text_track.scale.x = TEXT_SCALE;
    marker_text_track.scale.y = TEXT_SCALE;
    marker_text_track.scale.z = TEXT_SCALE;
    marker_text_track.color.r = 0.3f;
    marker_text_track.color.g = 0.9f;
    marker_text_track.color.b = 0.0f;
    marker_text_track.color.a = 0.8f;

    marker_text_label.header = _header;
    marker_text_label.ns = "label_info";
    marker_text_label.scale.x = TEXT_SCALE;
    marker_text_label.scale.y = TEXT_SCALE;
    marker_text_label.scale.z = TEXT_SCALE;
    marker_text_label.color.a = 0.9f;

    marker_velocity_arrow.header = _header;
    marker_velocity_arrow.ns = "velocity_dir";
    marker_velocity_arrow.color.r = 0.3f;
    marker_velocity_arrow.color.g = 0.9f;
    marker_velocity_arrow.color.b = 0.0f;
    marker_velocity_arrow.color.a = 1.f;

    marker_acc_arrow.header = _header;
    marker_acc_arrow.ns = "acc_dir";
    marker_acc_arrow.color.r = 0.9f;
    marker_acc_arrow.color.g = 0.9f;
    marker_acc_arrow.color.b = 0.0f;
    marker_acc_arrow.color.a = 1.f;

    marker_box.header = _header;
    marker_box.ns = "box";
    marker_box.color.r = colors[0].x;
    marker_box.color.g = colors[0].y;
    marker_box.color.b = colors[0].z;
    marker_box.scale.x = marker_box.scale.y = marker_box.scale.z = 0.03;

    marker_polygon.header = _header;
    marker_polygon.ns = "polygon";
    marker_polygon.color.r = 0.7f;
    marker_polygon.color.g = 0.5f;
    marker_polygon.color.b = 0.f;
    marker_polygon.scale.x = marker_polygon.scale.y = marker_polygon.scale.z = 0.05;

    marker_cube.header = _header;
    marker_cube.ns = "cube";

    //	std::string link_mode[4] ={"Bary", "Center", "S-center", "Corner"};

    int marker_id = 0;
    for (unsigned int i = 0; i < percept_list.size(); ++i)
    {
        const common::Obstacle::Ptr &perceptron = percept_list[i];

        //-------------------------------box----------------------------

        draw_polygon(perceptron, marker_id, marker_polygon, 0.8, 1.0);

        draw_box(perceptron, marker_id, marker_box, 0.5, 1.0);

        std::string text_box =
            num2str<float>(perceptron->distance, 1) + " (" + num2str<float>(perceptron->geo_size.x, 1) + " " +
            num2str<float>(perceptron->geo_size.y, 1) + " " + num2str<float>(perceptron->geo_size.z, 1) + ")"; //0.2 meter as chassis

        common::Point3 pos0 = perceptron->geo_center;
        pos0.z += perceptron->geo_size.z * 0.5f + 0.2f;
        draw_text(pos0, text_box, marker_id, marker_text_box, 1.0);

        marker_array->markers.push_back(marker_box);
        marker_array->markers.push_back(marker_polygon);
        marker_array->markers.push_back(marker_text_box);

        //--------------------------------tracking------------------------------
        float velocity = perceptron->velocity.norm();
        draw_velocity_arrow(perceptron, marker_id, marker_velocity_arrow, 0.8);
        draw_acc_arrow(perceptron, marker_id, marker_acc_arrow, 0.8);

        common::Point3 pos1 = perceptron->geo_center;
        pos1.z += perceptron->geo_size.z * 0.5f + 0.7f;

        float angle_vel = perceptron->angle_velocity / PI_OVER_180;

        std::string text_track =
            "<" + num2str<int>(perceptron->tracker_id, 0) + ">" + num2str<float>(velocity * 3.6f, 1) + "km/h";
        //        			+ "-" + num2str<float>(perceptron->acceleration.norm(), 1) + "m/s^2"
        //        + ">>" + num2str<float>(perceptron->association_score, 2);

        //			+" >> " + num2str<float>(perceptron->angle_velocity/PI_OVER_180, 2); //association_score

        //num2str<float>(angle_vel, 1);
        //+ " / " + num2str<float>(perceptron->sequence_robustness, 3);
        draw_text(pos1, text_track, marker_id, marker_text_track, 0.8);

        marker_array->markers.push_back(marker_velocity_arrow);
        marker_array->markers.push_back(marker_acc_arrow);
        marker_array->markers.push_back(marker_text_track);

        //--------------------------------classification---------------------------
        int type = static_cast<int>(perceptron->type);
        common::Point3 color = colors[type];
        bool is_bgd = (type == 0);
        //cube
        marker_cube.color.r = color.x;
        marker_cube.color.g = color.y;
        marker_cube.color.b = color.z;

        draw_cube(perceptron, marker_id, marker_cube, is_bgd ? 0.3 : 0.6, 1.0);

        marker_text_label.color.r = color.x;
        marker_text_label.color.g = color.y;
        marker_text_label.color.b = color.z;

        common::Point3 pos2 = perceptron->geo_center;
        pos2.z += perceptron->geo_size.z * 0.5f + 1.2f;

        std::string text_label = info[type] + ">>" + num2str<float>(perceptron->type_confidence, 2);

        draw_text(pos2, text_label, marker_id, marker_text_label, is_bgd ? 0 : 0.8);

        marker_array->markers.push_back(marker_cube);
        marker_array->markers.push_back(marker_text_label);

        marker_id++;
    }

    if (marker_id < marker_per_num_)
    {
        for (int i = marker_id; i < marker_per_num_; ++i)
        {
            marker_box.id = i;
            marker_box.color.a = 0.f;
            marker_array->markers.push_back(marker_box);

            marker_polygon.id = i;
            marker_polygon.color.a = 0.f;
            marker_array->markers.push_back(marker_polygon);

            marker_cube.id = i;
            marker_cube.color.a = 0.f;
            marker_array->markers.push_back(marker_cube);

            marker_text_box.id = i;
            marker_text_box.color.a = 0.f;
            marker_array->markers.push_back(marker_text_box);

            marker_text_track.id = i;
            marker_text_track.color.a = 0.f;
            marker_array->markers.push_back(marker_text_track);

            marker_velocity_arrow.id = i;
            marker_velocity_arrow.color.a = 0.f;
            marker_array->markers.push_back(marker_velocity_arrow);

            marker_acc_arrow.id = i;
            marker_acc_arrow.color.a = 0.f;
            marker_array->markers.push_back(marker_acc_arrow);

            marker_text_label.id = i;
            marker_text_label.color.a = 0.f;
            marker_array->markers.push_back(marker_text_label);
        }
    }

    marker_per_num_ = marker_id;
    pub_percept_info.publish(marker_array);
}

void DrawRviz::draw_cube(const common::Obstacle::Ptr &obj, const int &marker_id,
                         visualization_msgs::Marker &marker,
                         float alpha, float scale)
{
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    BBox box(Eigen::Vector3f(obj->geo_center.x, obj->geo_center.y, obj->geo_center.z), Eigen::Vector3f(obj->geo_size.x, obj->geo_size.y, obj->geo_size.z), Eigen::Vector3f(obj->geo_direction.x, obj->geo_direction.y, obj->geo_direction.z));
    float box_size = box.volume();
    if (box_size > 0)
    {
        marker.color.a = alpha;

        BBox box_s = box.scale(scale);

        marker.pose.position.x = box_s.center(0);
        marker.pose.position.y = box_s.center(1);
        marker.pose.position.z = box_s.center(2);

        marker.scale.x = box_s.size(0);
        marker.scale.y = box_s.size(1);
        marker.scale.z = box_s.size(2);

        tf::Quaternion quat = tf::createQuaternionFromYaw(box.angle);
        tf::quaternionTFToMsg(quat, marker.pose.orientation);
    }
    else
    {
        marker.color.a = 0;
    }
}

void DrawRviz::draw_polygon(const common::Obstacle::Ptr &obj, const int &marker_id,
                            visualization_msgs::Marker &marker,
                            float alpha, float scale)
{

    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    BBox box(Eigen::Vector3f(obj->geo_center.x, obj->geo_center.y, obj->geo_center.z), Eigen::Vector3f(obj->geo_size.x, obj->geo_size.y, obj->geo_size.z), Eigen::Vector3f(obj->geo_direction.x, obj->geo_direction.y, obj->geo_direction.z));
    float box_size = box.volume();
    if (box_size > 0)
    {

        tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, 0);
        tf::quaternionTFToMsg(quat, marker.pose.orientation);

        marker.color.a = alpha;
        std::vector<geometry_msgs::Point> cub_points;

        std::vector<common::Point3> corners;
        const std::vector<common::Point3> &polygon = obj->polygon;
        size_t p_size = polygon.size();
        float p_low = obj->geo_center.z - obj->geo_size.z * 0.5f;
        float p_high = obj->geo_center.z + obj->geo_size.z * 0.5f;

        for (unsigned int i = 0; i < p_size; ++i)
        {
            geometry_msgs::Point pts;
            pts.x = polygon[i].x;
            pts.y = polygon[i].y;
            pts.z = p_low;
            cub_points.push_back(pts);
        }

        for (unsigned int i = 0; i < p_size; ++i)
        {
            geometry_msgs::Point pts;
            pts.x = polygon[i].x;
            pts.y = polygon[i].y;
            pts.z = p_high;
            cub_points.push_back(pts);
        }

        for (unsigned int i = 0; i < p_size; ++i)
        {
            unsigned int next = i + 1;
            next = next < p_size ? next : 0;

            marker.points.push_back(cub_points[i]);
            marker.points.push_back(cub_points[next]);
        }

        for (unsigned int i = 0; i < p_size; ++i)
        {
            unsigned int next = i + 1;
            next = next < p_size ? next : 0;

            marker.points.push_back(cub_points[i + p_size]);
            marker.points.push_back(cub_points[next + p_size]);
        }

        for (unsigned int i = 0; i < p_size; ++i)
        {
            marker.points.push_back(cub_points[i]);
            marker.points.push_back(cub_points[i + p_size]);
        }
    }
    else
    {
        marker.color.a = 0;
    }
}

void DrawRviz::draw_box(const common::Obstacle::Ptr &obj, const int &marker_id,
                        visualization_msgs::Marker &marker,
                        float alpha, float scale)
{
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    BBox box(Eigen::Vector3f(obj->geo_center.x, obj->geo_center.y, obj->geo_center.z), Eigen::Vector3f(obj->geo_size.x, obj->geo_size.y, obj->geo_size.z), Eigen::Vector3f(obj->geo_direction.x, obj->geo_direction.y, obj->geo_direction.z));
    float box_size = box.volume();
    if (box_size > 0)
    {

        tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, 0);
        tf::quaternionTFToMsg(quat, marker.pose.orientation);

        marker.color.a = alpha;
        std::vector<geometry_msgs::Point> cub_points;

        std::vector<Eigen::Vector3f> corners;
        box.corners(corners);

        for (int i = 0; i < 8; ++i)
        {
            geometry_msgs::Point pts;
            pts.x = corners[i](0);
            pts.y = corners[i](1);
            pts.z = corners[i](2);
            cub_points.push_back(pts);
        }

        marker.points.push_back(cub_points[0]);
        marker.points.push_back(cub_points[1]);
        marker.points.push_back(cub_points[1]);
        marker.points.push_back(cub_points[2]);
        marker.points.push_back(cub_points[2]);
        marker.points.push_back(cub_points[3]);
        marker.points.push_back(cub_points[3]);
        marker.points.push_back(cub_points[0]);
        // horizontal high points for lines
        marker.points.push_back(cub_points[4]);
        marker.points.push_back(cub_points[5]);
        marker.points.push_back(cub_points[5]);
        marker.points.push_back(cub_points[6]);
        marker.points.push_back(cub_points[6]);
        marker.points.push_back(cub_points[7]);
        marker.points.push_back(cub_points[7]);
        marker.points.push_back(cub_points[4]);
        // vertical points for lines
        marker.points.push_back(cub_points[0]);
        marker.points.push_back(cub_points[4]);
        marker.points.push_back(cub_points[1]);
        marker.points.push_back(cub_points[5]);
        marker.points.push_back(cub_points[2]);
        marker.points.push_back(cub_points[6]);
        marker.points.push_back(cub_points[3]);
        marker.points.push_back(cub_points[7]);
    }
    else
    {
        marker.color.a = 0;
    }
}

void DrawRviz::draw_velocity_arrow(const common::Obstacle::Ptr &obj, const int &marker_id,
                                   visualization_msgs::Marker &marker, float alpha)
{
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    BBox box(Eigen::Vector3f(obj->geo_center.x, obj->geo_center.y, obj->geo_center.z), Eigen::Vector3f(obj->geo_size.x, obj->geo_size.y, obj->geo_size.z), Eigen::Vector3f(obj->geo_direction.x, obj->geo_direction.y, obj->geo_direction.z));
    float box_size = box.volume();
    if (box_size > 0)
    {
        marker.color.a = alpha;

        float arrow_length = obj->velocity.norm();
        float main_direction = atan2f(obj->velocity.y, obj->velocity.x);

        marker.scale.x = sqrtf(arrow_length + 1.0) - 1.0;
        if (static_cast<int>(obj->type) == 0)
        {
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
        }
        else
        {
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
        }

        tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., main_direction);
        tf::quaternionTFToMsg(quat, marker.pose.orientation);
        marker.pose.position.x = obj->anchor.x;
        marker.pose.position.y = obj->anchor.y;
        marker.pose.position.z = obj->anchor.z;
    }
    else
    {
        marker.color.a = 0;
    }
}

void DrawRviz::draw_acc_arrow(const common::Obstacle::Ptr &obj, const int &marker_id,
                              visualization_msgs::Marker &marker, float alpha)
{
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    BBox box(Eigen::Vector3f(obj->geo_center.x, obj->geo_center.y, obj->geo_center.z), Eigen::Vector3f(obj->geo_size.x, obj->geo_size.y, obj->geo_size.z), Eigen::Vector3f(obj->geo_direction.x, obj->geo_direction.y, obj->geo_direction.z));
    float box_size = box.volume();
    if (box_size > 0)
    {
        marker.color.a = alpha;

        float arrow_length = obj->acceleration.norm();
        float main_direction = atan2f(obj->acceleration.y, obj->acceleration.x);

        marker.scale.x = sqrtf(arrow_length + 1.0) - 1.0;
        if (static_cast<int>(obj->type) == 0)
        {
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
        }
        else
        {
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
        }

        tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., main_direction);
        tf::quaternionTFToMsg(quat, marker.pose.orientation);
        marker.pose.position.x = obj->geo_center.x;
        marker.pose.position.y = obj->geo_center.y;
        marker.pose.position.z = obj->geo_center.z;
    }
    else
    {
        marker.color.a = 0;
    }
}

void DrawRviz::draw_text(const common::Point3 &pos, const std::string &info, const int &marker_id,
                         visualization_msgs::Marker &marker, float alpha)
{
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pos.x;
    marker.pose.position.y = pos.y;
    marker.pose.position.z = pos.z;

    tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, 0);
    tf::quaternionTFToMsg(quat, marker.pose.orientation);

    marker.color.a = alpha;

    marker.text = info;
}

} // namespace perception
} // namespace robosense
