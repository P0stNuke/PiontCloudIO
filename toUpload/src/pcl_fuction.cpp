#include "pcl_fuction.h"

// PCL_Fuction::PCL_Fuction() {}

//------------------------------------filter--------------------------------------------

//voxel
// void pcl_filter_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,float leaf_size)
// {
//     pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
//     voxel_grid.setLeafSize(leaf_size,leaf_size,leaf_size);
//     voxel_grid.setInputCloud(cloud_in);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ> ()) ;
//     voxel_grid.filter(*cloud_out);

//     return cloud_out;
// }

void pcl_filter_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                      float leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(leaf_size,leaf_size,leaf_size);
    voxel_grid.setInputCloud(cloud_in);
    voxel_grid.filter(*cloud_out);

    return;
}


//------------------------------------registration--------------------------------------------
Eigen::Matrix4f pcl_registration_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rst,
                                     double paraA,double paraB,double paraC,int paraD)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    //kdTree 加速搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
    tree1->setInputCloud(cloud_src);
    tree2->setInputCloud(cloud_tgt);
    icp.setSearchMethodSource(tree1);
    icp.setSearchMethodTarget(tree2);

    //参数设置
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_tgt);
    icp.setMaxCorrespondenceDistance(paraA);//对应的最大距离
    icp.setTransformationEpsilon(paraB);//设置两次变化矩阵之间的差值
    icp.setEuclideanFitnessEpsilon(paraC);//前后两次误差大小，当误差值小于这个值停止迭代
    icp.setMaximumIterations(paraD);//最大迭代次数
    icp.align(*cloud_rst);

    Eigen::Matrix4f transformation = icp.getFinalTransformation();

    // qDebug() << "ICP has converged:" << icp.hasConverged() << " score: " <<
    //     icp.getFitnessScore() << std::endl;
    // qDebug() << icp.getFinalTransformation() << std::endl;

    return transformation;
}

// Function to manually pick point pairs
void manualAlign(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_src,
                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_tgt,
                 std::vector<int>& points1, std::vector<int>& points2,
                 Eigen::Matrix4f& transformation_matrix, bool scale) {
    // Ensure we have the same number of points in both vectors
    if (points1.size() != points2.size()) {
        std::cerr << "Error: Point vectors are of unequal sizes." << std::endl;
        return;
    }

    // Create matrices for the points
    Eigen::Matrix<float, 3, Eigen::Dynamic> src(3, points1.size());
    Eigen::Matrix<float, 3, Eigen::Dynamic> tgt(3, points2.size());

    for (size_t i = 0; i < points1.size(); ++i) {
        src.col(i) = cloud_src->points[points1[i]].getVector3fMap();
        tgt.col(i) = cloud_tgt->points[points2[i]].getVector3fMap();
    }

    // Compute the transformation using Singular Value Decomposition (SVD)
    // true 表示考虑缩放
    Eigen::Matrix4f T = Eigen::umeyama(src, tgt, scale);
    transformation_matrix = T;
}
