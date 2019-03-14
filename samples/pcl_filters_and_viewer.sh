#!/bin/sh

cd ~/.slam_results

if [ ! -f pointcloud.pcd ]
then
  echo "-------------------------------------------------------"
  echo "Run this script at the same directory of pointcloud.pcd"
  echo "(usually at ~/.slam_results)"
  echo "-------------------------------------------------------"

  exit 1
fi

pcl_outlier_removal pointcloud.pcd pointcloud.inlier.pcd -method statistical -mean_k 4 -std_dev_mul -0.2
pcl_voxel_grid pointcloud.inlier.pcd pointcloud.filtered.pcd -leaf 0.05,0.05,0.05

pcl_viewer pointcloud.filtered.pcd
