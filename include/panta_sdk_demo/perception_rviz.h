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
#ifndef ROBOSENSE_DRAWRVIZ_H
#define ROBOSENSE_DRAWRVIZ_H

#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <tf/tf.h>
#include <time.h>
#include <pcl_ros/point_cloud.h>
#include "panta_common/msg/rs_msg/obstacle_msg.h"
#include "panta_common/msg/rs_msg/freespace_msg.h"
// #include "panta_perception/common/basic_types.h"

namespace robosense {
namespace perception {

class alignas(16) DrawRviz {
	public:

    typedef std::shared_ptr<DrawRviz> Ptr;
    typedef std::shared_ptr<const DrawRviz> ConstPtr;

    DrawRviz();

	void show_freespace(const ros::Publisher& pub_cloud, const std_msgs::Header &_header,
                        const std::vector<common::FreeSpace::Ptr> &freespaces,const float& range = 60.);

	void show_percept(const ros::Publisher &pub_percept_info, const std_msgs::Header &_header,
	                  const std::vector<common::Obstacle::Ptr> &percept_list);

	void draw_box(const common::Obstacle::Ptr& obj, const int &marker_id, visualization_msgs::Marker &marker, float alpha,
	              float scale = 1.0);

	void draw_polygon(const common::Obstacle::Ptr& obj, const int &marker_id, visualization_msgs::Marker &marker, float alpha,
				  float scale = 1.0);

	void draw_cube(const common::Obstacle::Ptr& obj, const int &marker_id, visualization_msgs::Marker &marker, float alpha,
	               float scale = 1.0);

	void draw_text(const common::Point3 &pos, const std::string &info, const int &marker_id, visualization_msgs::Marker &marker,
	               float alpha = 1.0);

	void draw_velocity_arrow(const common::Obstacle::Ptr& obj, const int &marker_id, visualization_msgs::Marker &marker,
		float alpha = 1.0);

    void draw_acc_arrow(const common::Obstacle::Ptr& obj, const int &marker_id, visualization_msgs::Marker &marker,
                             float alpha = 1.0);

	pcl::PCLHeader convertROSHeader2PCLHeader(const std_msgs::Header&_header);

	std_msgs::Header convertPCLHeader2ROSHeader(const pcl::PCLHeader &_header);

	void generateColors(std::vector<common::Point3> &colors, int num);

	
	int marker_per_num_;
	std::vector<common::Point3> colors_;

};

}
}
#endif //ROBOSENSE_DRAWRVIZ_H