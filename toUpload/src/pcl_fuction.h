#ifndef PCL_FUCTION_H
#define PCL_FUCTION_H

// #include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>

#include <QDebug>

// class PCL_Fuction
// {
// public:
//     PCL_Fuction();
// private:
// };


//------------------------------------------filter--------------------------------
// pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filter_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
//                                                      float leaf_size);
void pcl_filter_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                      float leaf_size);
//------------------------------------registration--------------------------------------------
Eigen::Matrix4f pcl_registration_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rst,
                                     double paraA,double paraB,double paraC,int paraD);
// Function to manually pick point pairs
void manualAlign(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_src,
                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_tgt,
                 std::vector<int>& points1, std::vector<int>& points2,
                 Eigen::Matrix4f& transformation_matrix,
                 bool scale);
#endif // PCL_FUCTION_H
