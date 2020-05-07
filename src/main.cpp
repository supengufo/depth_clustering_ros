//
// Created by alex on 2020/4/12.
//
#include <ros/ros.h>
#include <depth_cluster.h>
#include <iostream>

DepthCluster depthCluster(2, 0.2, 16, 20);
ros::Publisher point_pub;
ros::Publisher ground_pub;

void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn;
    laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_ptr, *laserCloudIn);

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
    ros::init(argc, argv, "depth_cluster");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    ros::Subscriber point_sub = nh.subscribe("velodyne_points", 1, callback);
    point_pub = nh.advertise<sensor_msgs::PointCloud2>("cluster_result", 1);
    ground_pub = nh.advertise<sensor_msgs::PointCloud2>("ground_result", 1);
    ros::spin();
}