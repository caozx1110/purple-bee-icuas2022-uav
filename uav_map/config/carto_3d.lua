include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map_carto",            --发布的地图名称
  tracking_frame = "red/imu",         --被carto算法追踪的frame
  published_frame = "red/base_link",   --发布的坐标的ID
  odom_frame = "odom_carto",          --仅仅在provide_odom_frame为真的时候使用
  provide_odom_frame = true,    --发布在map frame中的坐标？ <没太理解>
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,      --使用里程计
  use_nav_sat = false,      --使用GPS
  use_landmarks = false,    --使用地标 <无人机场地没有地标>
  num_laser_scans = 0,                  --激光雷达传感器的数量
  num_multi_echo_laser_scans = 0,       --多回波激光雷达传感器的数量
  num_subdivisions_per_laser_scan = 1,  --将每个接收到的多回波激光扫描分割成的点云数量
  num_point_clouds = 1,                 --订阅的点云个数
  lookup_transform_timeout_sec = 0.2,   --用于使用tf2查找转换的超时
  submap_publish_period_sec = 0.3,      --发布子图的时间间隔
  pose_publish_period_sec = 5e-3,       --发布位置的时间间隔5e-3对应200Hz
  trajectory_publish_period_sec = 30e-3,--发布轨迹的时间间隔
  rangefinder_sampling_ratio = 1.,      --范围查找器消息的固定比率采样 <不懂这是啥东西>
  odometry_sampling_ratio = 1.,         --里程计消息的固定采样比率
  fixed_frame_pose_sampling_ratio = 1., --固定帧采样比率 <不懂啥意思>
  imu_sampling_ratio = 1.,              --imu采样比率
  landmarks_sampling_ratio = 1.,        --地标消息的固定采样比率 <应该用不着,除非场地上贴了地标>
}

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 160

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 320
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

return options