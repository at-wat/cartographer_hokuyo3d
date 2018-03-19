#!/bin/bash

touch /slam_results/.lock

roslaunch cartographer_hokuyo3d hokuyo3d_single.launch record:=true bag_file:=/slam_results/slam_recorded.bag &
pid=$!

sleep 5

rostopic echo -n1 /move_base_simple/goal

sleep 1

rosservice call /finish_trajectory "trajectory_id: 0"
rosservice call /write_state "filename: '/slam_results/pose_graph.pbstream'"

sleep 1

kill -SIGTERM $pid
wait $pid

sleep 1

roslaunch cartographer_hokuyo3d asset_writer.launch \
  config_dir:=/opt/ros/${ROS_DISTRO}/share/cartographer_hokuyo3d/configuration_files \
  bag_filenames:=/slam_results/slam_recorded.bag \
  output_file_prefix:=/slam_results/ \
  pose_graph_filename:=/slam_results/pose_graph.pbstream \
  urdf_filename:=/opt/ros/${ROS_DISTRO}/share/cartographer_hokuyo3d/configuration_files/hokuyo3d.urdf

chmod og+r /slam_results/pointcloud.pcd

rm /slam_results/.lock
