--  Copyright 2017 cartographer_hokuyo3d authors
--  All rights reserved.

--  Redistribution and use in source and binary forms, with or without
--  modification, are permitted provided that the following conditions are met:

--  * Redistributions of source code must retain the above copyright
--  notice, this list of conditions and the following disclaimer.
--  * Redistributions in binary form must reproduce the above copyright
--  notice, this list of conditions and the following disclaimer in the
--  documentation and/or other materials provided with the distribution.
--  * Neither the name of the copyright holder nor the names of its 
--  contributors may be used to endorse or promote products derived from 
--  this software without specific prior written permission.

--  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
--  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
--  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
--  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
--  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
--  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
--  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
--  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
--  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
--  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
--  POSSIBILITY OF SUCH DAMAGE.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "hokuyo3d_imu",
  published_frame = "hokuyo3d",
  odom_frame = "hokuyo3d",
  provide_odom_frame = false,
  use_odometry = false,
  use_laser_scan = false,
  use_multi_echo_laser_scan = false,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
}

MAX_3D_LASER_RANGE = 35.

--------------------------------
-- Trajectory builder settings.

TRAJECTORY_BUILDER_3D.laser_max_range = MAX_3D_LASER_RANGE
TRAJECTORY_BUILDER_3D.laser_min_range = 1.0
TRAJECTORY_BUILDER_3D.scans_per_accumulation = 4

TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 10.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = MAX_3D_LASER_RANGE

TRAJECTORY_BUILDER_3D.kalman_local_trajectory_builder.pose_tracker.imu_gravity_time_constant = 0.5
TRAJECTORY_BUILDER_3D.kalman_local_trajectory_builder.pose_tracker.imu_gravity_variance = 1.0

-- Local matching settings.
TRAJECTORY_BUILDER_3D.kalman_local_trajectory_builder.real_time_correlative_scan_matcher.linear_search_window = 0.4
TRAJECTORY_BUILDER_3D.kalman_local_trajectory_builder.real_time_correlative_scan_matcher.angular_search_window = math.rad(5.)

--------------------------------
-- Map builder settings.

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 8

MAP_BUILDER.sparse_pose_graph.optimization_problem.huber_scale = 5e2
MAP_BUILDER.sparse_pose_graph.optimize_every_n_scans = 20
MAP_BUILDER.sparse_pose_graph.constraint_builder.sampling_ratio = 0.2
MAP_BUILDER.sparse_pose_graph.global_sampling_ratio = 0.1
MAP_BUILDER.sparse_pose_graph.optimization_problem.ceres_solver_options.max_num_iterations = 12
MAP_BUILDER.sparse_pose_graph.constraint_builder.adaptive_voxel_filter = TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter

-- Set min_score according to the histogram from cartographer.
MAP_BUILDER.sparse_pose_graph.constraint_builder.min_score = 0.65
MAP_BUILDER.sparse_pose_graph.constraint_builder.global_localization_min_score = 0.7

-- Global constraint settings.
MAP_BUILDER.sparse_pose_graph.constraint_builder.max_constraint_distance= 10.0
MAP_BUILDER.sparse_pose_graph.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 10.0
MAP_BUILDER.sparse_pose_graph.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 2.0
MAP_BUILDER.sparse_pose_graph.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(30.)

return options
