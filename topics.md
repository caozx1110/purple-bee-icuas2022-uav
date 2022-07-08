|                            topic                             |         Publisher          |                       Subscriber                       |                        type                        |                      说明                      |
| :----------------------------------------------------------: | :------------------------: | :----------------------------------------------------: | :------------------------------------------------: | :--------------------------------------------: |
|                            /clock                            |          /gazebo           |                     需要用到时钟的                     |                rosgraph_msgs/Clock                 |                gazebo模拟的时钟                |
|                         /diagnostics                         |        /red/mavros         |                                                        |       diagnostic_msgs/<br />DiagnosticArray        |                      未知                      |
|                     /gazebo/link_states                      |          /gazebo           |                                                        |               gazebo_msgs/LinkStates               |           模型控件之间的连接位姿信息           |
|                     /gazebo/model_states                     |          /gazebo           |                                                        |              gazebo_msgs/ModelStates               |          地面，UAV，边界，二维码，球           |
|             /gazebo/<br />parameter_descriptions             |          /gazebo           |                                                        |    dynamic_reconfigure/<br />ConfigDescription     |                  descriptions                  |
|               /gazebo/<br />parameter_updates                |          /gazebo           |                                                        |             dynamic_reconfigure/Config             |                     config                     |
|              /gazebo/<br />performance_metrics               |          /gazebo           |                                                        |        gazebo_msgs/<br />PerformanceMetrics        |                    性能指标                    |
|                    /gazebo/set_link_state                    |                            |                        /gazebo                         |               gazebo_msgs/LinkState                |                       /                        |
|                   /gazebo/set_model_state                    |                            |                        /gazebo                         |               gazebo_msgs/ModelState               |                       /                        |
|                      /gazebo/wind_speed                      |                            |                        /gazebo                         |               rotors_comm/WindSpeed                |                       /                        |
|                         /motor_speed                         |          /gazebo           |                                                        |                  std_msgs/Float32                  |                    电机速度                    |
|                    /red/ball/magnet/gain                     |                            |                        /gazebo                         |                  std_msgs/Float32                  |                "data: 0.0" 松开                |
|                     /red/ball/magnet/mfs                     |          /gazebo           |                                                        |             sensor_msgs/MagneticField              |                   磁场传感器                   |
|                   /red/ball/magnet/wrench                    |          /gazebo           |                                                        |         geometry_msgs/<br />WrenchStamped          |                      未知                      |
|                      /red/ball/odometry                      |          /gazebo           |                                                        |                 nav_msgs/Odometry                  |                   球的里程计                   |
|                        /red/ball/pose                        |          /gazebo           |                 /red/spawn_ball_at_uav                 |             geometry_msgs/PoseStamped              |                    球的姿态                    |
|             /red/ball/<br />pose_with_covariance             |          /gazebo           |                                                        |   geometry_msgs/<br />PoseWithCovarianceStamped    |                 带协方差的位姿                 |
|                      /red/ball/position                      |          /gazebo           |                                                        |          geometry_msgs/<br />PointStamped          |                    球的位姿                    |
|                     /red/ball/transform                      |          /gazebo           |                                                        |        geometry_msgs/<br />TransformStamped        |                  坐标转换信息                  |
|              /red/ball/<br />velocity_relative               |          /gazebo           |                                                        |             geometry_msgs/TwistStamped             |            相对速度（球相对UAV？）             |
|             /red/camera/<br />color/camera_info              |          /gazebo           |                                                        |               sensor_msgs/CameraInfo               |                    相机参数                    |
|              /red/camera/<br />color/image_raw               |          /gazebo           |                                                        |                 sensor_msgs/Image                  |                 rgb相机的image                 |
|      /red/camera/<br />color/image_raw/<br />compressed      |          /gazebo           |                                                        |         sensor_msgs/<br />CompressedImage          |                    压缩图像                    |
| /red/camera/<br />color/image_raw/<br />compressed/<br />parameter_descriptions |          /gazebo           |                                                        |    dynamic_reconfigure/<br />ConfigDescription     |          压缩图像配置参数相关（未知）          |
| /red/camera/<br />color/image_raw/<br />compressed/<br />parameter_updates |          /gazebo           |                                                        |             dynamic_reconfigure/Config             |                    配置参数                    |
|   /red/camera/<br />color/image_raw/<br />compressedDepth    |          /gazebo           |                                                        |         sensor_msgs<br />/CompressedImage          | 深度信息的压缩图像（灰度图）事实上，照片不存在 |
| /red/camera/<br />color/image_raw/<br />compressedDepth/<br />parameter_descriptions |             /              |                                                        |                         /                          |                       /                        |
| /red/camera/<br />color/image_raw/<br />compressedDepth/<br />parameter_updates |             /              |                                                        |                         /                          |                       /                        |
|        /red/camera/<br />color/image_raw/<br />theora        |          /gazebo           |                                                        |        theora_image_transport/<br />Packet         |            通过theora技术压缩的图像            |
| /red/camera/<br />color/image_raw/<br />theora/parameter_descriptions |             /              |                                                        |                         /                          |                       /                        |
| /red/camera/<br />color/image_raw/<br />theora/parameter_updates |             /              |                                                        |                         /                          |                       /                        |
|             /red/camera/<br />depth/camera_info              |          /gazebo           |                                                        |               sensor_msgs/CameraInfo               |                  深度相机内参                  |
|              /red/camera/<br />depth/image_raw               |          /gazebo           |                                                        |                 sensor_msgs/Image                  |          深度相机的深度图像（灰度图）          |
|          /red/camera/<br />depth_registered/points           |          /gazebo           |                                                        |              sensor_msgs/PointCloud2               |                   深度点云图                   |
|         /red/camera_ir/<br />parameter_descriptions          |             /              |                                                        |                         /                          |                       /                        |
|            /red/camera_ir/<br />parameter_updates            |             /              |                                                        |                         /                          |                       /                        |
|                       /red/carrot/pose                       | /red/carrot_reference_node |                                                        |             geometry_msgs/PoseStamped              |                                                |
|                      /red/carrot/status                      | /red/carrot_reference_node | /red/pid_cascade_node<br />/red/toppra_uav_ros_tracker |                  std_msgs/String                   |                                                |
|                    /red/carrot/trajectory                    | /red/carrot_reference_node | /red/pid_cascade_node<br />/red/toppra_uav_ros_tracker | trajectory_msgs<br />/MultiDOFJointTrajectoryPoint |                                                |
|                     /red/carrot/velocity                     |   /red/pid_cascade_node    |                                                        |               geometry_msgs/Vector3                |                                                |
|                       /red/carrot/yaw                        | /red/carrot_reference_node |                 /red/pid_cascade_node                  |                  std_msgs/Float64                  |                                                |
|       /red/cascade_config/<br />parameter_descriptions       |             /              |                                                        |                         /                          |                       /                        |
|         /red/cascade_config/<br />parameter_updates          |             /              |                                                        |                         /                          |                       /                        |
|                    /red/challenge_started                    |     /rostopic_[某个id]     |                                                        |                   std_msgs/Bool                    |             比赛环境是否正常之类的             |
|            /red/gazebo/<br />command/motor_speed             |          /gazebo           |                        /gazebo                         |          mav_msgs/Actuators（自定义类型）          |             四个旋翼各自的电机转速             |
/red/motor_speed|/gazebo|None|mav_msgs/Actuators|--
/red/odometry|/gazebo|/red/pid_cascade_node,/red/carrot_reference_node|nav_msgs/Odometry|--
/red/pose|/gazebo|/red/spawn_ball_at_uav|geometry_msgs/PoseStamped|--
/red/pose_with_covariance|/gazebo|None|geometry_msgs/PoseWithCovarianceStamped|--
/red/position|/gazebo|None|geometry_msgs/PointStamped|--
/red/position_hold/trajectory|/red/toppra_uav_ros_tracker|/red/carrot_reference_node|trajectory_msgs/MultiDOFJointTrajectoryPoint|--
/red/toppra_raw_trajectory|/red/topp_trajectory_gen|None|trajectory_msgs/JointTrajectory|--
/red/toppra_raw_waypoints|/red/topp_trajectory_gen|None|trajectory_msgs/JointTrajectory|--
/red/tracker/input_pose|None|/red/toppra_uav_ros_tracker|geometry_msgs/PoseStamped|--
/red/tracker/input_trajectory|None|/red/toppra_uav_ros_tracker|trajectory_msgs/MultiDOFJointTrajectory|--
/red/tracker/path|/red/toppra_uav_ros_tracker|None|trajectory_msgs/nav_msgs/Path|--
/red/tracker/status|/red/toppra_uav_ros_tracker|None|std_msgs/String|--
/red/transform|/gazebo|None|geometry_msgs/TransformStamped|--
/red/uav/euler_setpoint |/red/pid_cascade_node|None|geometry_msgs/Vector3|--
/red/uav/velocity|/red/pid_cascade_node|None|geometry_msgs/Vector3|--
/red/uav/yaw|/red/carrot_reference_node|None|std_msgs/Float64|--
/red/uav_magnet/gain|None|/gazebo|std_msgs/Float32|--
/red/uav_magnet/mfs|/gazebo|None|sensor_msgs/MagneticField|--
/red/uav_magnet/wrench|/gazebo|None|geometry_msgs/WrenchStamped|--
/red/velocity_relative|/gazebo|None|geometry_msgs/TwistStamped|--




+ rgb相机参数example

```yaml
header: 
  seq: 0	# int
  stamp: 
    secs: 4069
    nsecs: 104000000
  frame_id: "red/camera"
height: 480
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [381.36246688113556, 0.0, 320.5, 0.0, 381.36246688113556, 240.5, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [381.36246688113556, 0.0, 320.5, -0.0, 0.0, 381.36246688113556, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False

```

