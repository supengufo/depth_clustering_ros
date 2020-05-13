# depth_clustre_ros

[TOC]



This is a point cloud clustering segmentation algorithm, including the removal of ground point clouds and the segmentation of point clouds.



<img src="doc/clusters.gif" alt="clusters" style="zoom:50%;" />

<img src="doc/ground.gif" alt="ground" style="zoom:50%;" />

## Requirement

- [PCL](https://github.com/PointCloudLibrary/pcl)
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
https://github.com/supengufo/depth_clustre_ros
cd ..
caktin_make
```

## Run

run the launch file:

```
cd ~/catkin_ws
source devel/setup.bash(or setup.zsh)
roslaunch depth_cluster_ros depth_cluster_ros.launch
```



## Principles

### Ground Remove

If special terrain such as slopes and depressions is not considered, it is a common ground segmentation idea to treat the ground as a complete plane and fit the equation ![](http://latex.codecogs.com/gif.latex?Ax+By+Cz+D=0)

However, for a simple ground fitting, there are many ways to realize it, such as <a href="https://www.codecogs.com/eqnedit.php?latex=Ax&plus;By&plus;Cz&plus;D=0" target="_blank"><img src="https://latex.codecogs.com/gif.latex?Ax&plus;By&plus;Cz&plus;D=0" title="Ax+By+Cz+D=0" /></a> form (This means that the final minimum fitting equation is <a href="https://www.codecogs.com/eqnedit.php?latex=Ax=0" target="_blank"><img src="https://latex.codecogs.com/gif.latex?Ax=0" title="Ax=0" /></a>),or <a href="https://www.codecogs.com/eqnedit.php?latex=\frac{A}{D}x&plus;\frac{B}{D}y&plus;\frac{C}{D}z&plus;1=0" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\frac{A}{D}x&plus;\frac{B}{D}y&plus;\frac{C}{D}z&plus;1=0" title="\frac{A}{D}x+\frac{B}{D}y+\frac{C}{D}z+1=0" /></a> which is ![](http://latex.codecogs.com/gif.latex? Ax=b).

Here we use the properties of the plane equation, the first three coefficients (A,B,C) of which are the normal vectors of the plane. And the normal vector can be obtained by principal component analysis (PCA) ,which is more simply to obtain the final ground equation.

**1. exact N lowest points**

In this step, we first need to sort all the points, and then take the n lowest point of  z axis as the ground point. Calculate the coordinate mean of these n points and record it as<a href="https://www.codecogs.com/eqnedit.php?latex=\bar&space;X&space;=&space;(\bar&space;x,&space;\bar&space;y,&space;\bar&space;z)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\bar&space;X&space;=&space;(\bar&space;x,&space;\bar&space;y,&space;\bar&space;z)" title="\bar X = (\bar x, \bar y, \bar z)" /></a> .

**2. PCA**

Then calculate the covariance matrix M of the n lowest points.(<a href="https://www.codecogs.com/eqnedit.php?latex=M&space;\in&space;\mathbb{R}^{3*3}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?M&space;\in&space;\mathbb{R}^{3*3}" title="M \in \mathbb{R}^{3*3}" /></a> ).

Because of the particularity of plane point cloud distribution, the singular value corresponding to the direction of the normal vector of M matrix should be the smallest when it is processed by principal component analysis(PCA).We write it down as n. And <a href="https://www.codecogs.com/eqnedit.php?latex=n=(A,B,C)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?n=(A,B,C)" title="n=(A,B,C)" /></a>.

So,we can obtain the paramter D by substituting n and <a href="https://www.codecogs.com/eqnedit.php?latex=\bar&space;x" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\bar&space;x" title="\bar x" /></a> into planar equation.

![](http://39.107.30.202:8080/s/R2rRzKFBpokZDzM/preview)

### Cloud Segmentation

<img src="doc/cluster_3d.gif" alt="ground"  />



First, convert all the 3D points coordinates into 2D depth image;

<img src="doc/depth_image.gif" alt="ground"  />

Then,use Breadth-First-Search(BFS) to extraction point cloud clusters. The algorithm flow chart is as follows:

<img src="doc/cluster_algo.png" alt="ground"  />



## Acknowledgements

The main idea of point cloud segmentation is based on depth_cluster, in which the filtering threshold condition and neighborhood search are modified;

The segmentation of ground point clouds is based on Zermas' paper, although principal component analysis is already a very common method to extract ground.

Thank them for their work! 



All these two papers can be found in the `doc` folder.

```
@Article{bogoslavskyi17pfg,
title   = {Efficient Online Segmentation for Sparse 3D Laser Scans},
author  = {I. Bogoslavskyi and C. Stachniss},
journal = {PFG -- Journal of Photogrammetry, Remote Sensing and Geoinformation Science},
year    = {2017},
pages   = {1--12},
url     = {https://link.springer.com/article/10.1007%2Fs41064-016-0003-y},
}
```

```
@inproceedings{Zermas2017Fast,
  title={Fast segmentation of 3D point clouds: A paradigm on LiDAR data for autonomous vehicle applications},
  author={Zermas, Dimitris and Izzat, Izzat and Papanikolopoulos, Nikolaos},
  booktitle={IEEE International Conference on Robotics and Automation},
  year={2017},
}
```

