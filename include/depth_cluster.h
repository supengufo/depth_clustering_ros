//
// Created by alex on 2020/4/12.
//

#ifndef SRC_DEPTH_CLUSTER_H
#define SRC_DEPTH_CLUSTER_H
//std
#include <unordered_map>
#include <queue>
#include <ctime>
#include <set>
//ROS
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
//Eigen
#include <Eigen/Dense>
//using Eigen::MatrixXf;
//using Eigen::JacobiSVD;
//using Eigen::VectorXf;

//self
#include "pointinfo.h"

using namespace std;
using namespace pcl;


class DepthCluster {
public:
    /**
     *
     * @param vertcal_resolution sensor vertical resolution
     * @param horizontal_resolution sensor horizontal resolution
     * @param lidar_lines lidar lines,usually 16,32,64,128
     * @param cluster_size Point clouds larger than this number are considered as a cluster
     */
    DepthCluster(float vertcal_resolution, float horizontal_resolution, int lidar_lines, int cluster_size);
    /**
     *  init some parameters,depend on different sensors
     */
    void initParams();
    /**
     *
     * @param msg input pointcloud msg
     */
    void setInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &msg);
    /**
     *
     * @param msg
     * @param label_image
     * @return
     */
    vector<int> exactGroundPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &msg, vector<vector<int>> &label_image);

    vector<int> exactGroundCloudIndices(pcl::PointCloud<pcl::PointXYZI>::Ptr &msg, vector<vector<int>> &label_image, Eigen::Vector4f &ground_params);

    Eigen::Vector4f estimatePlaneParams(pcl::PointCloud<pcl::PointXYZI>::Ptr &initial_ground_points);
    void extractInitialGroundPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &initial_ground_points);
    /**
     *
     * @param depth_image
     * @param label_image
     * @param cloud_msg
     */
    void labelComponents(const vector<vector<PointInfo>> &depth_image, vector<vector<int>> &label_image, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_msg);
    /**
     * convert 3D points seach to 2D,instead of using kd-tree and more faster
     * @param cloud_fused_ptr input cloud ptr
     * @return 2D vector image stored every point depth and index
     */
    vector<vector<PointInfo>> generateDepthImage(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_fused_ptr);
    /**
     *
     * @param depth_image
     * @param target_point
     * @param neigh_point
     * @return if the neighbour point belong to the same cluster
     */
    bool judgmentCondition(const vector<vector<PointInfo>> &depth_image, const pair<int, int> &target_point, const pair<int, int> &neigh_point);
    bool calculateCoordinate(const pcl::PointXYZI &point, int &row, int &col);
    /**
     *
     * @param pt
     * @return make sure the searched point belong to the search region
     */
    bool warpPoint(pair<int, int> &pt);

    /**
     *
     * @return 2D clusters index in pointcloud
     */
    vector<vector<int>> getClustersIndex();

    vector<int> getMergedClustersIndex();
    /**
     *
     * @return all the ground points indices
     */
    vector<int> getGroundCloudIndices();

    /**
     *  reset some parameters
     */
    void paramsReset();
private:
    /**
     *  ground move parameters
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr sorted_Pointcloud_;
    vector<int> ground_points_indices_;
    float sensor_height_;
    /**
     * cluster parameters
     */
    //lidar params need to initialize in class construction function
    float vertcal_resolution_, horizontal_resolution_;
    int lidar_lines_;
    int cluster_size_;
    //needed to be initialize in initial function
    int image_rows_, image_cols_;
    int vertical_angle_min_;
    int vertical_angle_max_;

    vector<vector<int>> clusters_indices_vec_;
};


#endif //SRC_DEPTH_CLUSTER_H
