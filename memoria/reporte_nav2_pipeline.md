# 📊 Reporte de Introspección del Middleware ROS 2

## 1. Nodos Activos (75)

- `/amcl`
- `/behavior_server`
- `/bt_navigator`
- `/bt_navigator_navigate_through_poses_rclcpp_node`
- `/bt_navigator_navigate_to_pose_rclcpp_node`
- `/bumper_contact_bridge`
- `/buttons_msg_bridge`
- `/camera_bridge`
- `/camera_stf`
- `/cliff_front_left_bridge`
- `/cliff_front_right_bridge`
- `/cliff_side_left_bridge`
- `/cliff_side_right_bridge`
- `/clock_bridge`
- `/cmd_vel_bridge`
- `/collision_monitor`
- `/controller_manager`
- `/controller_server`
- `/diffdrive_controller`
- `/dock_state_publisher`
- `/docking_server`
- `/global_costmap/global_costmap`
- `/gz_ros_control`
- `/hazards_vector_publisher`
- `/hmi_buttons_msg_bridge`
- `/hmi_display_msg_bridge`
- `/hmi_led_msg_bridge`
- `/interface_buttons_node`
- `/ir_intensity_front_center_left_bridge`
- `/ir_intensity_front_center_right_bridge`
- `/ir_intensity_front_left_bridge`
- `/ir_intensity_front_right_bridge`
- `/ir_intensity_left_bridge`
- `/ir_intensity_right_bridge`
- `/ir_intensity_side_left_bridge`
- `/ir_intensity_vector_publisher`
- `/joint_state_broadcaster`
- `/joint_state_publisher`
- `/kidnap_estimator_publisher`
- `/lidar_bridge`
- `/lifecycle_manager_localization`
- `/lifecycle_manager_navigation`
- `/local_costmap/local_costmap`
- `/map_server`
- `/mock_publisher`
- `/motion_control`
- `/odom_base_tf_bridge`
- `/planner_server`
- `/pose_bridge`
- `/pose_republisher_node`
- `/robot_state`
- `/robot_state_publisher`
- `/route_server`
- `/rplidar_stf`
- `/rviz2`
- `/rviz2`
- `/rviz2`
- `/rviz_navigation_dialog_action_client`
- `/sensors_node`
- `/smoother_server`
- `/tf2_echo`
- `/tf2_echo`
- `/tf_odom_std_dock_link_publisher`
- `/transform_listener_impl_5dab7079aab0`
- `/transform_listener_impl_5dc47278d400`
- `/transform_listener_impl_5dc473073f70`
- `/transform_listener_impl_5e7ca91e2430`
- `/transform_listener_impl_609babad8480`
- `/transform_listener_impl_643f641bfba0`
- `/turtlebot4_gz_hmi_node`
- `/turtlebot4_node`
- `/ui_mgr`
- `/velocity_smoother`
- `/waypoint_follower`
- `/wheel_status_publisher`

## 2. Mapeo Detallado de Nodos e Interfaces

| Nodo | Tópicos Suscritos (Inputs) | Tópicos Publicados (Outputs) | Servicios Ofrecidos |
| :--- | :--- | :--- | :--- |
| `/amcl` | /bond<br>/clock<br>/initialpose<br>/map... | /amcl/transition_event<br>/amcl_pose<br>/bond<br>/parameter_events... | /amcl/change_state<br>/amcl/describe_parameters<br>/amcl/get_available_states<br>/amcl/get_available_transitions... |
| `/behavior_server` | /clock<br>/cmd_vel_teleop<br>/local_costmap/costmap_raw<br>/local_costmap/costmap_raw_updates... | /behavior_server/transition_event<br>/cmd_vel_nav<br>/parameter_events<br>/rosout | /behavior_server/change_state<br>/behavior_server/describe_parameters<br>/behavior_server/get_available_states<br>/behavior_server/get_available_transitions... |
| `/bt_navigator` | /clock<br>/goal_pose<br>/odom<br>/parameter_events... | /bt_navigator/transition_event<br>/parameter_events<br>/rosout | /bt_navigator/change_state<br>/bt_navigator/describe_parameters<br>/bt_navigator/get_available_states<br>/bt_navigator/get_available_transitions... |
| `/bt_navigator_navigate_through_poses_rclcpp_node` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /bt_navigator_navigate_through_poses_rclcpp_node/describe_parameters<br>/bt_navigator_navigate_through_poses_rclcpp_node/get_parameter_types<br>/bt_navigator_navigate_through_poses_rclcpp_node/get_parameters<br>/bt_navigator_navigate_through_poses_rclcpp_node/get_type_description... |
| `/bt_navigator_navigate_to_pose_rclcpp_node` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /bt_navigator_navigate_to_pose_rclcpp_node/describe_parameters<br>/bt_navigator_navigate_to_pose_rclcpp_node/get_parameter_types<br>/bt_navigator_navigate_to_pose_rclcpp_node/get_parameters<br>/bt_navigator_navigate_to_pose_rclcpp_node/get_type_description... |
| `/bumper_contact_bridge` | /clock<br>/parameter_events | /bumper_contact<br>/parameter_events<br>/rosout | /bumper_contact_bridge/describe_parameters<br>/bumper_contact_bridge/get_parameter_types<br>/bumper_contact_bridge/get_parameters<br>/bumper_contact_bridge/get_type_description... |
| `/buttons_msg_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /buttons_msg_bridge/describe_parameters<br>/buttons_msg_bridge/get_parameter_types<br>/buttons_msg_bridge/get_parameters<br>/buttons_msg_bridge/get_type_description... |
| `/camera_bridge` | /clock<br>/parameter_events | /oakd/rgb/preview/camera_info<br>/oakd/rgb/preview/depth<br>/oakd/rgb/preview/depth/points<br>/oakd/rgb/preview/image_raw... | /camera_bridge/describe_parameters<br>/camera_bridge/get_parameter_types<br>/camera_bridge/get_parameters<br>/camera_bridge/get_type_description... |
| `/camera_stf` | /parameter_events | /parameter_events<br>/rosout<br>/tf_static | /camera_stf/describe_parameters<br>/camera_stf/get_parameter_types<br>/camera_stf/get_parameters<br>/camera_stf/get_type_description... |
| `/cliff_front_left_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /cliff_front_left_bridge/describe_parameters<br>/cliff_front_left_bridge/get_parameter_types<br>/cliff_front_left_bridge/get_parameters<br>/cliff_front_left_bridge/get_type_description... |
| `/cliff_front_right_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /cliff_front_right_bridge/describe_parameters<br>/cliff_front_right_bridge/get_parameter_types<br>/cliff_front_right_bridge/get_parameters<br>/cliff_front_right_bridge/get_type_description... |
| `/cliff_side_left_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /cliff_side_left_bridge/describe_parameters<br>/cliff_side_left_bridge/get_parameter_types<br>/cliff_side_left_bridge/get_parameters<br>/cliff_side_left_bridge/get_type_description... |
| `/cliff_side_right_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /cliff_side_right_bridge/describe_parameters<br>/cliff_side_right_bridge/get_parameter_types<br>/cliff_side_right_bridge/get_parameters<br>/cliff_side_right_bridge/get_type_description... |
| `/clock_bridge` | /parameter_events | /clock<br>/parameter_events<br>/rosout | /clock_bridge/describe_parameters<br>/clock_bridge/get_parameter_types<br>/clock_bridge/get_parameters<br>/clock_bridge/get_type_description... |
| `/cmd_vel_bridge` | /clock<br>/diffdrive_controller/cmd_vel<br>/parameter_events | /cmd_vel<br>/parameter_events<br>/rosout | /cmd_vel_bridge/describe_parameters<br>/cmd_vel_bridge/get_parameter_types<br>/cmd_vel_bridge/get_parameters<br>/cmd_vel_bridge/get_type_description... |
| `/collision_monitor` | /clock<br>/cmd_vel_smoothed<br>/local_costmap/published_footprint<br>/parameter_events... | /cmd_vel<br>/collision_monitor/collision_points_marker<br>/collision_monitor/transition_event<br>/collision_monitor_state... | /collision_monitor/change_state<br>/collision_monitor/describe_parameters<br>/collision_monitor/get_available_states<br>/collision_monitor/get_available_transitions... |
| `/controller_manager` | /clock<br>/parameter_events<br>/robot_description | /controller_manager/activity<br>/controller_manager/introspection_data/full<br>/controller_manager/introspection_data/names<br>/controller_manager/introspection_data/values... | /controller_manager/cleanup_controller<br>/controller_manager/configure_controller<br>/controller_manager/describe_parameters<br>/controller_manager/get_logger_levels... |
| `/controller_server` | /bond<br>/clock<br>/odom<br>/parameter_events... | /bond<br>/cmd_vel_nav<br>/controller_server/transition_event<br>/optimal_trajectory... | /controller_server/change_state<br>/controller_server/describe_parameters<br>/controller_server/get_available_states<br>/controller_server/get_available_transitions... |
| `/diffdrive_controller` | /clock<br>/diffdrive_controller/cmd_vel<br>/parameter_events | /diffdrive_controller/transition_event<br>/odom<br>/parameter_events<br>/rosout... | /diffdrive_controller/describe_parameters<br>/diffdrive_controller/get_logger_levels<br>/diffdrive_controller/get_parameter_types<br>/diffdrive_controller/get_parameters... |
| `/dock_state_publisher` | /clock<br>/joint_states<br>/parameter_events | /parameter_events<br>/rosout<br>/standard_dock_description<br>/tf... | /dock_state_publisher/describe_parameters<br>/dock_state_publisher/get_parameter_types<br>/dock_state_publisher/get_parameters<br>/dock_state_publisher/get_type_description... |
| `/docking_server` | /clock<br>/detected_dock_pose<br>/local_costmap/costmap_raw<br>/local_costmap/costmap_raw_updates... | /cmd_vel<br>/dock_pose<br>/docking_server/transition_event<br>/docking_trajectory... | /docking_server/change_state<br>/docking_server/describe_parameters<br>/docking_server/get_available_states<br>/docking_server/get_available_transitions... |
| `/global_costmap/global_costmap` | /clock<br>/global_costmap/footprint<br>/map<br>/parameter_events | /global_costmap/costmap<br>/global_costmap/costmap_raw<br>/global_costmap/costmap_raw_updates<br>/global_costmap/costmap_updates... | /global_costmap/clear_around_global_costmap<br>/global_costmap/clear_around_pose_global_costmap<br>/global_costmap/clear_entirely_global_costmap<br>/global_costmap/clear_except_global_costmap... |
| `/gz_ros_control` | /parameter_events | /parameter_events<br>/rosout | /gz_ros_control/describe_parameters<br>/gz_ros_control/get_parameter_types<br>/gz_ros_control/get_parameters<br>/gz_ros_control/get_type_description... |
| `/hazards_vector_publisher` | /clock<br>/parameter_events | /hazard_detection<br>/parameter_events<br>/rosout | /hazards_vector_publisher/describe_parameters<br>/hazards_vector_publisher/get_parameter_types<br>/hazards_vector_publisher/get_parameters<br>/hazards_vector_publisher/get_type_description... |
| `/hmi_buttons_msg_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /hmi_buttons_msg_bridge/describe_parameters<br>/hmi_buttons_msg_bridge/get_parameter_types<br>/hmi_buttons_msg_bridge/get_parameters<br>/hmi_buttons_msg_bridge/get_type_description... |
| `/hmi_display_msg_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /hmi_display_msg_bridge/describe_parameters<br>/hmi_display_msg_bridge/get_parameter_types<br>/hmi_display_msg_bridge/get_parameters<br>/hmi_display_msg_bridge/get_type_description... |
| `/hmi_led_msg_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /hmi_led_msg_bridge/describe_parameters<br>/hmi_led_msg_bridge/get_parameter_types<br>/hmi_led_msg_bridge/get_parameters<br>/hmi_led_msg_bridge/get_type_description... |
| `/interface_buttons_node` | /clock<br>/parameter_events | /interface_buttons<br>/parameter_events<br>/rosout | /interface_buttons_node/describe_parameters<br>/interface_buttons_node/get_parameter_types<br>/interface_buttons_node/get_parameters<br>/interface_buttons_node/get_type_description... |
| `/ir_intensity_front_center_left_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /ir_intensity_front_center_left_bridge/describe_parameters<br>/ir_intensity_front_center_left_bridge/get_parameter_types<br>/ir_intensity_front_center_left_bridge/get_parameters<br>/ir_intensity_front_center_left_bridge/get_type_description... |
| `/ir_intensity_front_center_right_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /ir_intensity_front_center_right_bridge/describe_parameters<br>/ir_intensity_front_center_right_bridge/get_parameter_types<br>/ir_intensity_front_center_right_bridge/get_parameters<br>/ir_intensity_front_center_right_bridge/get_type_description... |
| `/ir_intensity_front_left_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /ir_intensity_front_left_bridge/describe_parameters<br>/ir_intensity_front_left_bridge/get_parameter_types<br>/ir_intensity_front_left_bridge/get_parameters<br>/ir_intensity_front_left_bridge/get_type_description... |
| `/ir_intensity_front_right_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /ir_intensity_front_right_bridge/describe_parameters<br>/ir_intensity_front_right_bridge/get_parameter_types<br>/ir_intensity_front_right_bridge/get_parameters<br>/ir_intensity_front_right_bridge/get_type_description... |
| `/ir_intensity_left_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /ir_intensity_left_bridge/describe_parameters<br>/ir_intensity_left_bridge/get_parameter_types<br>/ir_intensity_left_bridge/get_parameters<br>/ir_intensity_left_bridge/get_type_description... |
| `/ir_intensity_right_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /ir_intensity_right_bridge/describe_parameters<br>/ir_intensity_right_bridge/get_parameter_types<br>/ir_intensity_right_bridge/get_parameters<br>/ir_intensity_right_bridge/get_type_description... |
| `/ir_intensity_side_left_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /ir_intensity_side_left_bridge/describe_parameters<br>/ir_intensity_side_left_bridge/get_parameter_types<br>/ir_intensity_side_left_bridge/get_parameters<br>/ir_intensity_side_left_bridge/get_type_description... |
| `/ir_intensity_vector_publisher` | /clock<br>/parameter_events | /ir_intensity<br>/parameter_events<br>/rosout | /ir_intensity_vector_publisher/describe_parameters<br>/ir_intensity_vector_publisher/get_parameter_types<br>/ir_intensity_vector_publisher/get_parameters<br>/ir_intensity_vector_publisher/get_type_description... |
| `/joint_state_broadcaster` | /clock<br>/parameter_events | /dynamic_joint_states<br>/joint_state_broadcaster/transition_event<br>/joint_states<br>/parameter_events... | /joint_state_broadcaster/describe_parameters<br>/joint_state_broadcaster/get_logger_levels<br>/joint_state_broadcaster/get_parameter_types<br>/joint_state_broadcaster/get_parameters... |
| `/joint_state_publisher` | /clock<br>/robot_description | /joint_states<br>/parameter_events<br>/rosout | /joint_state_publisher/describe_parameters<br>/joint_state_publisher/get_parameter_types<br>/joint_state_publisher/get_parameters<br>/joint_state_publisher/get_type_description... |
| `/kidnap_estimator_publisher` | /clock<br>/hazard_detection<br>/parameter_events | /kidnap_status<br>/parameter_events<br>/rosout | /kidnap_estimator_publisher/describe_parameters<br>/kidnap_estimator_publisher/get_parameter_types<br>/kidnap_estimator_publisher/get_parameters<br>/kidnap_estimator_publisher/get_type_description... |
| `/lidar_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout<br>/scan | /lidar_bridge/describe_parameters<br>/lidar_bridge/get_parameter_types<br>/lidar_bridge/get_parameters<br>/lidar_bridge/get_type_description... |
| `/lifecycle_manager_localization` | /bond<br>/clock<br>/parameter_events | /bond<br>/diagnostics<br>/parameter_events<br>/rosout | /lifecycle_manager_localization/describe_parameters<br>/lifecycle_manager_localization/get_parameter_types<br>/lifecycle_manager_localization/get_parameters<br>/lifecycle_manager_localization/get_type_description... |
| `/lifecycle_manager_navigation` | /bond<br>/clock<br>/parameter_events | /bond<br>/diagnostics<br>/parameter_events<br>/rosout | /lifecycle_manager_navigation/describe_parameters<br>/lifecycle_manager_navigation/get_parameter_types<br>/lifecycle_manager_navigation/get_parameters<br>/lifecycle_manager_navigation/get_type_description... |
| `/local_costmap/local_costmap` | /clock<br>/local_costmap/footprint<br>/map<br>/parameter_events... | /local_costmap/clearing_endpoints<br>/local_costmap/costmap<br>/local_costmap/costmap_raw<br>/local_costmap/costmap_raw_updates... | /local_costmap/clear_around_local_costmap<br>/local_costmap/clear_around_pose_local_costmap<br>/local_costmap/clear_entirely_local_costmap<br>/local_costmap/clear_except_local_costmap... |
| `/map_server` | /bond<br>/clock<br>/parameter_events | /bond<br>/map<br>/map_server/transition_event<br>/parameter_events... | /map_server/change_state<br>/map_server/describe_parameters<br>/map_server/get_available_states<br>/map_server/get_available_transitions... |
| `/mock_publisher` | /clock<br>/parameter_events | /parameter_events<br>/rosout<br>/slip_status | /mock_publisher/describe_parameters<br>/mock_publisher/get_parameter_types<br>/mock_publisher/get_parameters<br>/mock_publisher/get_type_description... |
| `/motion_control` | /clock<br>/cmd_vel<br>/cmd_vel_unstamped<br>/dock_status... | /diffdrive_controller/cmd_vel<br>/parameter_events<br>/rosout<br>/wheel_status | /e_stop<br>/motion_control/describe_parameters<br>/motion_control/get_parameter_types<br>/motion_control/get_parameters... |
| `/odom_base_tf_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout<br>/tf | /odom_base_tf_bridge/describe_parameters<br>/odom_base_tf_bridge/get_parameter_types<br>/odom_base_tf_bridge/get_parameters<br>/odom_base_tf_bridge/get_type_description... |
| `/planner_server` | /clock<br>/parameter_events | /parameter_events<br>/plan<br>/planner_server/transition_event<br>/rosout | /planner_server/change_state<br>/planner_server/describe_parameters<br>/planner_server/get_available_states<br>/planner_server/get_available_transitions... |
| `/pose_bridge` | /clock<br>/parameter_events | /parameter_events<br>/rosout | /pose_bridge/describe_parameters<br>/pose_bridge/get_parameter_types<br>/pose_bridge/get_parameters<br>/pose_bridge/get_type_description... |
| `/pose_republisher_node` | /clock<br>/joint_states<br>/parameter_events | /dynamic_joint_states<br>/parameter_events<br>/rosout<br>/sim_ground_truth_dock_pose... | /pose_republisher_node/describe_parameters<br>/pose_republisher_node/get_parameter_types<br>/pose_republisher_node/get_parameters<br>/pose_republisher_node/get_type_description... |
| `/robot_state` | /clock<br>/dock_status<br>/odom<br>/parameter_events | /battery_state<br>/parameter_events<br>/rosout<br>/stop_status | /robot_state/describe_parameters<br>/robot_state/get_parameter_types<br>/robot_state/get_parameters<br>/robot_state/get_type_description... |
| `/robot_state_publisher` | /clock<br>/joint_states<br>/parameter_events | /parameter_events<br>/robot_description<br>/rosout<br>/tf... | /robot_state_publisher/describe_parameters<br>/robot_state_publisher/get_parameter_types<br>/robot_state_publisher/get_parameters<br>/robot_state_publisher/get_type_description... |
| `/route_server` | /clock<br>/global_costmap/costmap_raw<br>/global_costmap/costmap_raw_updates<br>/parameter_events... | /parameter_events<br>/plan<br>/rosout<br>/route_graph... | /route_server/DynamicEdgesScorer/adjust_edges<br>/route_server/ReroutingService/reroute<br>/route_server/change_state<br>/route_server/describe_parameters... |
| `/rplidar_stf` | /parameter_events | /parameter_events<br>/rosout<br>/tf_static | /rplidar_stf/describe_parameters<br>/rplidar_stf/get_parameter_types<br>/rplidar_stf/get_parameters<br>/rplidar_stf/get_type_description... |
| `/rviz2` | /clock<br>/downsampled_costmap<br>/downsampled_costmap_updates<br>/global_costmap/costmap... | /clicked_point<br>/initialpose<br>/parameter_events<br>/rosout | /rviz2/describe_parameters<br>/rviz2/get_parameter_types<br>/rviz2/get_parameters<br>/rviz2/get_type_description... |
| `/rviz2` | /clock<br>/downsampled_costmap<br>/downsampled_costmap_updates<br>/global_costmap/costmap... | /clicked_point<br>/initialpose<br>/parameter_events<br>/rosout | /rviz2/describe_parameters<br>/rviz2/get_parameter_types<br>/rviz2/get_parameters<br>/rviz2/get_type_description... |
| `/rviz2` | /clock<br>/downsampled_costmap<br>/downsampled_costmap_updates<br>/global_costmap/costmap... | /clicked_point<br>/initialpose<br>/parameter_events<br>/rosout | /rviz2/describe_parameters<br>/rviz2/get_parameter_types<br>/rviz2/get_parameters<br>/rviz2/get_type_description... |
| `/rviz_navigation_dialog_action_client` | /parameter_events | /parameter_events<br>/rosout<br>/waypoints | /rviz_navigation_dialog_action_client/describe_parameters<br>/rviz_navigation_dialog_action_client/get_parameter_types<br>/rviz_navigation_dialog_action_client/get_parameters<br>/rviz_navigation_dialog_action_client/get_type_description... |
| `/sensors_node` | /bumper_contact<br>/clock<br>/joint_states<br>/parameter_events... | /dock_status<br>/ir_opcode<br>/mouse<br>/parameter_events... | /sensors_node/describe_parameters<br>/sensors_node/get_parameter_types<br>/sensors_node/get_parameters<br>/sensors_node/get_type_description... |
| `/smoother_server` | /bond<br>/clock<br>/global_costmap/costmap_raw<br>/global_costmap/costmap_raw_updates... | /bond<br>/parameter_events<br>/plan_smoothed<br>/rosout... | /smoother_server/change_state<br>/smoother_server/describe_parameters<br>/smoother_server/get_available_states<br>/smoother_server/get_available_transitions... |
| `/tf2_echo` | *(ninguno)* | *(ninguno)* | *(ninguno)* |
| `/tf2_echo` | *(ninguno)* | *(ninguno)* | *(ninguno)* |
| `/tf_odom_std_dock_link_publisher` | /parameter_events | /parameter_events<br>/rosout<br>/tf_static | /tf_odom_std_dock_link_publisher/describe_parameters<br>/tf_odom_std_dock_link_publisher/get_parameter_types<br>/tf_odom_std_dock_link_publisher/get_parameters<br>/tf_odom_std_dock_link_publisher/get_type_description... |
| `/transform_listener_impl_5dab7079aab0` | *(ninguno)* | *(ninguno)* | *(ninguno)* |
| `/transform_listener_impl_5dc47278d400` | /parameter_events<br>/tf<br>/tf_static | /rosout | /transform_listener_impl_5dc47278d400/get_type_description |
| `/transform_listener_impl_5dc473073f70` | /parameter_events<br>/tf<br>/tf_static | /rosout | /transform_listener_impl_5dc473073f70/get_type_description |
| `/transform_listener_impl_5e7ca91e2430` | /clock<br>/parameter_events<br>/tf<br>/tf_static | /rosout | /transform_listener_impl_5e7ca91e2430/get_type_description |
| `/transform_listener_impl_609babad8480` | /clock<br>/parameter_events<br>/tf<br>/tf_static | /rosout | /transform_listener_impl_609babad8480/get_type_description |
| `/transform_listener_impl_643f641bfba0` | *(ninguno)* | *(ninguno)* | *(ninguno)* |
| `/turtlebot4_gz_hmi_node` | /hmi/display<br>/parameter_events | /hmi/buttons<br>/parameter_events<br>/rosout | /turtlebot4_gz_hmi_node/describe_parameters<br>/turtlebot4_gz_hmi_node/get_parameter_types<br>/turtlebot4_gz_hmi_node/get_parameters<br>/turtlebot4_gz_hmi_node/get_type_description... |
| `/turtlebot4_node` | /battery_state<br>/dock_status<br>/hmi/buttons<br>/hmi/display/message... | /function_calls<br>/hmi/display<br>/ip<br>/parameter_events... | /turtlebot4_node/describe_parameters<br>/turtlebot4_node/get_parameter_types<br>/turtlebot4_node/get_parameters<br>/turtlebot4_node/get_type_description... |
| `/ui_mgr` | /clock<br>/cmd_audio<br>/cmd_lightring<br>/parameter_events | /parameter_events<br>/rosout | /ui_mgr/describe_parameters<br>/ui_mgr/get_parameter_types<br>/ui_mgr/get_parameters<br>/ui_mgr/get_type_description... |
| `/velocity_smoother` | /clock<br>/cmd_vel_nav<br>/parameter_events | /cmd_vel_smoothed<br>/parameter_events<br>/rosout<br>/velocity_smoother/transition_event | /velocity_smoother/change_state<br>/velocity_smoother/describe_parameters<br>/velocity_smoother/get_available_states<br>/velocity_smoother/get_available_transitions... |
| `/waypoint_follower` | /clock<br>/parameter_events | /parameter_events<br>/rosout<br>/waypoint_follower/transition_event | /waypoint_follower/change_state<br>/waypoint_follower/describe_parameters<br>/waypoint_follower/get_available_states<br>/waypoint_follower/get_available_transitions... |
| `/wheel_status_publisher` | /clock<br>/dynamic_joint_states<br>/parameter_events | /parameter_events<br>/rosout<br>/wheel_ticks<br>/wheel_vels | /wheel_status_publisher/describe_parameters<br>/wheel_status_publisher/get_parameter_types<br>/wheel_status_publisher/get_parameters<br>/wheel_status_publisher/get_type_description... |

## 3. Acciones Activas

- `/assisted_teleop`
- `/audio_note_sequence`
- `/backup`
- `/compute_and_track_route`
- `/compute_path_through_poses`
- `/compute_path_to_pose`
- `/compute_route`
- `/dock`
- `/dock_robot`
- `/drive_arc`
- `/drive_distance`
- `/drive_on_heading`
- `/follow_gps_waypoints`
- `/follow_path`
- `/follow_waypoints`
- `/led_animation`
- `/navigate_through_poses`
- `/navigate_to_pose`
- `/navigate_to_position`
- `/rotate_angle`
- `/smooth_path`
- `/spin`
- `/undock`
- `/undock_robot`
- `/wait`
- `/wall_follow`
