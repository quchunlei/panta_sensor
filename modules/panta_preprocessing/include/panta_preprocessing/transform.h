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
#ifndef RS_TRANSFORM_H
#define RS_TRANSFORM_H

#include <iostream>
//#include <Eigen/Eigen>
#include <pcl/common/transforms.h>
namespace robosense
{
namespace preprocessing
{

struct _Pose
{
  union
  {
    struct
    {
      double x;
      double y;
      double z;
      double roll;
      double pitch;
      double yaw;
    };
    double data[6];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 Pose : public _Pose
{
public:
  Pose();
  ~Pose() = default;
  Pose(const double& x, const double& y, const double& z, const double& roll,
       const double& pitch, const double& yaw);
  Eigen::Matrix<double, 6, 1> getVector6dMap();
  Eigen::Affine3d getAffine3d() const;
  Pose& operator=(const Eigen::Affine3d& aff);
  Pose& operator=(const Pose& pose);
};
std::ostream& operator<<(std::ostream& os, const Pose& p);

Eigen::Vector3d getEulerAngles(const Eigen::Matrix3d& r);
Eigen::Matrix3d getRotation(const double roll, const double pitch,
                            const double yaw);

/**
 * @brief 6 dof pose to 4*4 transform matrix
 * @param[in] pose
 * @return  transform_matrix
 */
Eigen::Matrix4d poseToMatrix(const double& pose_x, const double& pose_y,
                             const double& pose_z, const double& pose_roll,
                             const double& pose_pitch, const double& pose_yaw);

Eigen::Matrix4d poseToMatrix(robosense::preprocessing::Pose& in_pose);

std::array<double, 6> poseToArray(robosense::preprocessing::Pose& in_pose);


/**
 * @brief 4*4 transform matrix to  6 dof pose
 * @param[in]  transform_matrix
 * @param[in] pose
 */
void matrixToPose(Eigen::Matrix4d& transform_matrix, double& pose_x,
                  double& pose_y, double& pose_z, double& pose_roll,
                  double& pose_pitch, double& pose_yaw);

void matrixToPose(Eigen::Matrix4d& transform_matrix,
                  Eigen::Matrix<double, 6, 1>& pose);

template <typename PointT, typename Scalar>
void transformPointCloud(
    const pcl::PointCloud<PointT>& cloud_in, pcl::PointCloud<PointT>& cloud_out,
    const Eigen::Transform<Scalar, 3, Eigen::Affine>& transform);

template <typename PointT, typename Scalar>
void transformPointCloud(const pcl::PointCloud<PointT>& cloud_in,
                         pcl::PointCloud<PointT>& cloud_out,
                         const Eigen::Matrix<Scalar, 4, 4>& transform);

typedef std::array<double, 3> rs_Vector3d;
typedef std::array<double, 9> rs_Matirx3d;

rs_Vector3d getRsMatrix(const Eigen::Vector3d& eig_vec);
rs_Matirx3d getRsMatrix(Eigen::Matrix3d& eig_mat);

}  // namespace preprocessing
}  // namespace robosense
#endif
