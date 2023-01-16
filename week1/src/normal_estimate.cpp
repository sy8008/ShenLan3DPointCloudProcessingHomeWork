#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <thread>
#include <chrono>
#include <pcl/filters/voxel_grid.h>
using namespace std;

int main(int argc, char** argv) {

    string cloud_path = "/home/sy/Shenlan_3D_pointCloud_processing/week1/data/airplane_0627.pcd";
    // display the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_path, *src_cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read pcd file \n");
        return (-1);
    }

    // 创建滤波对象
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(src_cloud);
    // 设置体素栅格的大小为 1x1x1cm
    filter.setLeafSize(0.01f, 0.01f, 0.01f);
    filter.filter(*cloud);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree; //创建KDtree
    kdtree.setInputCloud(cloud); // 设置要搜索的点云，建立KDTree
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normals->resize(cloud->size());
    int k = 30; //设置搜索的近邻点数

    for(int i = 0; i < cloud->size(); i++){
        pcl::PointXYZ searchPoint;
        searchPoint.x = cloud->points[i].x;
        searchPoint.y = cloud->points[i].y;
        searchPoint.z = cloud->points[i].z;
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);
        int num_neighbours = kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance);
        Eigen::Vector3d pt_mean(0.,0.,0);
        // vector<Eigen::Vector3d> neighbour_pts;
        Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Zero();
        if ( num_neighbours > 0)
        {
            // neighbour_pts.resize(num_neighbours);
            for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
                pt_mean += Eigen::Vector3d(cloud->points[pointIdxNKNSearch[i]].x, cloud->points[pointIdxNKNSearch[i]].y, cloud->points[pointIdxNKNSearch[i]].z);
                // neighbour_pts[i] = Eigen::Vector3d(cloud->points[pointIdxNKNSearch[i]].x, cloud->points[pointIdxNKNSearch[i]].y, cloud->points[pointIdxNKNSearch[i]].z);
            }
            pt_mean /= static_cast<double>(pointIdxNKNSearch.size());
            for(size_t i = 0; i < pointIdxNKNSearch.size(); ++i){
                Eigen::Vector3d neighbour_pt(cloud->points[pointIdxNKNSearch[i]].x, cloud->points[pointIdxNKNSearch[i]].y, cloud->points[pointIdxNKNSearch[i]].z);
                neighbour_pt -= pt_mean;
                cov_mat += neighbour_pt * neighbour_pt.transpose();
                // neighbour_pts[i] -= pt_mean;
            }

            // compute SVD of covariance matrix
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3d U = svd.matrixU();
            normals->points[i].normal_x = U(0, 2);
            normals->points[i].normal_y = U(1, 2);
            normals->points[i].normal_z = U(2, 2);
            
            
        }
    }

    // pcl::visualization::CloudViewer viewer("Cloud Viewer");
    // viewer.showCloud(cloud);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    int v1(0);
    // vviewer->addCoordinateSystem(1.0,v1);
    viewer->addPointCloud<pcl::PointXYZ>(cloud,"cloud",v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud"); // 设置点云大小
    viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals, 50, 0.1, "normals",v1);
    while (!viewer-> wasStopped()) {
        viewer->spinOnce();
            
    }


    return 0;
}

