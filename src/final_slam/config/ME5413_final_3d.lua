-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  -- Add a hidden parameter 'publish_tracked_pose' for 'evo' plotting
  publish_tracked_pose = true,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = true,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

--------------------------------------------------------------------------------
-- This config can not work
--------------------------------------------------------------------------------
-- MAP_BUILDER.use_trajectory_builder_3d = true
-- MAP_BUILDER.num_background_threads = 7
-- -- TRAJECTORY_BUILDER_3D.use_imu_data = true
-- TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 160

-- -- Local SLAM
-- -- TRAJECTORY_BUILDER_3D.submaps.num_range_data = 10

-- -- Global SLAM
-- POSE_GRAPH.optimization_problem.huber_scale = 5e2
-- POSE_GRAPH.optimize_every_n_nodes = 320
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
-- POSE_GRAPH.constraint_builder.min_score = 0.62
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66


--------------------------------------------------------------------------------
-- This config can work
--------------------------------------------------------------------------------
-- MAP_BUILDER.use_trajectory_builder_3d = true
--    TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
--    TRAJECTORY_BUILDER_3D.min_range = 0.3
--   --  defualt 60 
--    TRAJECTORY_BUILDER_3D.max_range = 60. --30
--    TRAJECTORY_BUILDER_2D.min_z = -0.8
--    TRAJECTORY_BUILDER_2D.max_z = 1.0
--    TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true 
--    TRAJECTORY_BUILDER_3D.submaps.num_range_data = 65  --注意和optimize_every_n_nodes对应！ --30 --65
--    TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.01 
   
--    TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 0.1
   
   
--    MAP_BUILDER.num_background_threads = 5 
--    POSE_GRAPH.optimization_problem.huber_scale = 5e2
 
 
--    POSE_GRAPH.optimize_every_n_nodes = 130 
 
--    POSE_GRAPH.constraint_builder.sampling_ratio = 0.03  
--    POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 16
--    POSE_GRAPH.constraint_builder.min_score = 0.95
--    POSE_GRAPH.constraint_builder.global_localization_min_score = 0.95
 
--    POSE_GRAPH.optimization_problem.fix_z_in_3d = true
--    POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = false
--    POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1
--    POSE_GRAPH.optimization_problem.odometry_translation_weight = 1
--    POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1
--    POSE_GRAPH.max_num_final_iterations = 200


--------------------------------------------------------------------------------
-- Reorginized config
--------------------------------------------------------------------------------
MAP_BUILDER.use_trajectory_builder_3d = true
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.min_range = 0.2  -- defualt 60 
TRAJECTORY_BUILDER_3D.max_range = 30.0 -- 30
TRAJECTORY_BUILDER_2D.min_z = -0.8
TRAJECTORY_BUILDER_2D.max_z = 3.0
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 500 -- 注意和optimize_every_n_nodes对应！ --30 --65
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.01
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 0.1

MAP_BUILDER.num_background_threads = 10
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 1000
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 16
POSE_GRAPH.constraint_builder.min_score = 0.95
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.95
POSE_GRAPH.optimization_problem.fix_z_in_3d = true
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = false -- true
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1
POSE_GRAPH.max_num_final_iterations = 200

return options
