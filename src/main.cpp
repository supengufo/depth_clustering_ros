//
// Created by alex on 2020/4/12.
//
#include <depth_cluster.h>
#include <iostream>

void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn;
    laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_ptr, *laserCloudIn);


    sensor_msgs::PointCloud2 laserCloudTemp;
    pcl::toROSMsg(*ground_points, laserCloudTemp);
    laserCloudTemp.header.stamp = cloud_ptr->header.stamp;
    laserCloudTemp.header.frame_id = cloud_ptr->header.frame_id;
    ground_pub.publish(laserCloudTemp);

    auto cluster_indices = depthCluster.getMergedClustersIndex();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(*laserCloudIn, cluster_indices, *cluster_points);
    pcl::toROSMsg(*cluster_points, laserCloudTemp);
    laserCloudTemp.header.stamp = cloud_ptr->header.stamp;
    laserCloudTemp.header.frame_id = cloud_ptr->header.frame_id;
    point_pub.publish(laserCloudTemp);

};
int main(int argc, char **argv) {


    DepthCluster depthCluster(2, 0.2, 16, 20);
    depthCluster.setInputCloud(laserCloudIn);
    vector<vector<int>> clustersIndex = depthCluster.getClustersIndex();
    for (auto cluster_vec:clustersIndex) {
        int intensity = rand()%255;
        for (int i = 0; i < cluster_vec.size(); ++i) {
            laserCloudIn->points[cluster_vec[i]].intensity = intensity;
        }
    }

    auto ground_index = depthCluster.getGroundCloudIndices();
    int intensity = 100;
    for (int j : ground_index) {
        laserCloudIn->points[j].intensity = intensity;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(*laserCloudIn, ground_index, *ground_points);

}