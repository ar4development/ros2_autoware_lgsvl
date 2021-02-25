#!/bin/bash

echo 'Starting the container..'
source /opt/ros/$ROS_DISTRO/setup.bash
source /ad_stack/AutowareAuto/install/setup.bash
source /ad_stack/lgsvl_msgs/install/setup.bash
source /ad_stack/ros2-lgsvl-bridge/install/setup.bash
source /ad_stack/aw_object_detection/packages/aw_lgsvl/install/setup.bash
echo 'Sourcing done..'
mkdir /ad_stack/logs
cd /ad_stack/logs
lgsvl_bridge > lgsvl_bridge.log 2>&1 &
echo 'Bridge started..'
ros2 run point_cloud_filter_transform_nodes point_cloud_filter_transform_node_exe --ros-args --params-file /ad_stack/aw_object_detection/parameters/transform_node_param.yaml -r __ns:=/ar4development  -r __node:=filter_transform_vlp16 -r points_in:=/ar4development/points_raw > points_transformation.log 2>&1 &
echo 'Primary point transformation node started..'
ros2 run ray_ground_classifier_nodes ray_ground_classifier_cloud_node_exe --ros-args --params-file /ad_stack/aw_object_detection/parameters/ground_vs_objects_node_param.yaml -r __node:=ground_vs_objects -r __ns:=/ar4development -r points_in:=/ar4development/points_filtered > ground_non-ground.log 2>&1 &
echo 'Ground vs non-ground segregation node started..'
ros2 run euclidean_cluster_nodes euclidean_cluster_node_exe --ros-args --params-file /ad_stack/aw_object_detection/parameters/clusterization_node_param.yaml -r __ns:=/ar4development -r __node:=object_detection_node -r points_in:=/ar4development/points_nonground > clusterization.log 2>&1 &
echo 'Clusterization node started..'
ros2 run aw_lgsvl converter_node --ros-args --params-file /ad_stack/aw_object_detection/parameters/conversion.yaml > detections_conversion.log 2>&1
echo 'Autoware-to-LGSVL detections convertion node started..'
bash




