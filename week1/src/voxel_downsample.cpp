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
#include <unordered_map>
using namespace std;

class myVoxelDownSampler{

public:
myVoxelDownSampler(double voxel_grid_ize):grid_size(voxel_grid_ize),Dx(0),Dy(0),Dz(0),
x_max(FLT_MIN),y_max(FLT_MIN),z_max(FLT_MIN),x_min(FLT_MAX),y_min(FLT_MAX),z_min(FLT_MAX)
{
    src_cloud = nullptr;
    
}

void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
    src_cloud = cloud;
}

void downsample(pcl::PointCloud<pcl::PointXYZ>& output_cloud){
    calcVoxelDimension();
    unordered_map<int,vector<int>> umap;
    // output_cloud.resize(Dx * Dy * Dz);
    for(int i = 0; i < src_cloud->size(); i++){
        int hx = (src_cloud->points[i].x - x_min) / grid_size;
        int hy = (src_cloud->points[i].y - y_min) / grid_size;
        int hz = (src_cloud->points[i].z - z_min) / grid_size;
        int h = hx + hy * Dx + hz * Dx * Dy;
        umap[h].push_back(i);
    }

    for(auto & p : umap){
        int sz = p.second.size();
        int select_idx = p.second[rand() % sz];
        output_cloud.points.push_back(src_cloud->points[select_idx]);
    }

}   

private:

void calcVoxelDimension(){
    for(const auto & p : src_cloud->points){
        x_max = std::max(x_max,p.x);
        y_max = std::max(y_max,p.y);
        z_max = std::max(z_max,p.z);
        x_min = std::min(x_min,p.x);
        y_min = std::min(y_min,p.y);
        z_min = std::min(z_min,p.z);
    }
    Dx = (x_max - x_min) / grid_size;
    Dy = (y_max - y_min) / grid_size;
    Dz = (z_max - z_min) / grid_size;
}

int Dx;
int Dy;
int Dz;
float x_max,y_max,z_max;
float x_min,y_min,z_min;
double grid_size;
pcl::PointCloud<pcl::PointXYZ>::ConstPtr src_cloud;

};


int main(int argc, char** argv) {

    string cloud_path = "/home/sy/Shenlan_3D_pointCloud_processing/week1/data/airplane_0627.pcd";
    // display the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr my_downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);

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

    myVoxelDownSampler my_sampler(0.01f);
    my_sampler.setInputCloud(src_cloud);
    my_sampler.downsample(*my_downsampled_cloud);

    cout << "my downsample output cloud size: " << my_downsampled_cloud->size() << endl;
    cout << "my downsample output cloud size: " << cloud->size() << endl;

    // pcl::visualization::CloudViewer viewer("Cloud Viewer");
    // viewer.showCloud(cloud);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    int v1(0);
    // vviewer->addCoordinateSystem(1.0,v1);
    viewer->addPointCloud<pcl::PointXYZ>(my_downsampled_cloud,"cloud",v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud"); // 设置点云大小
    while (!viewer-> wasStopped()) {
        viewer->spinOnce();
            
    }


    return 0;
}

