cmake_minimum_required(VERSION 3.5)
project(person_detection_simulation)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(turtlebot3_description REQUIRED)
find_package(turtlebot3 REQUIRED)
find_package(navigation2 REQUIRED)
find_package(gazebo REQUIRED)
find_package(cloudwatch_logger REQUIRED)
find_package(kinesis_video_streamer REQUIRED)
find_package(kinesis_video_msgs REQUIRED)
find_package(h264_video_encoder REQUIRED)
find_package(health_metric_collector REQUIRED)

#find_package(catkin REQUIRED COMPONENTS
#  gazebo_ros
#  turtlebot3_description   # required to install .rviz model
#)


install(DIRECTORY launch routes config
  DESTINATION share/${PROJECT_NAME}
)

# Copy the rviz model for easier access in AWS RoboMaker RViz
install(FILES ${turtlebot3_description_DIR}/../rviz/model.rviz
  DESTINATION share/${PROJECT_NAME}/rviz
  RENAME turtlebot3_model.rviz
)

ament_package()
