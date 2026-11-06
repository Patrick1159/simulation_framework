cmake_minimum_required(VERSION 3.0.2)
project(sup_exp)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
  mpc_nav
  umap
)

catkin_package()

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY configs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/configs)

catkin_install_python(PROGRAMS
  scripts/frame_bridge.py
  scripts/analyze_frame_errors.py
  scripts/analyze_issue1_tracking_drift.py
  scripts/analyze_coarse_candidate_drop.py
  scripts/fake_bbox_publisher.py
  scripts/stage0_verify.py
  scripts/stage1_verify.py
  scripts/stage2_analyze.py
  scripts/relay_with_features.py
  scripts/perfect_measurement.py
  scripts/umap_to_base.py
  scripts/single_static_obstacle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
