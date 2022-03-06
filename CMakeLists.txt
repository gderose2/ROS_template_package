cmake_minimum_required(VERSION 3.0.2)
project(template_package)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
generate_dynamic_reconfigure_options(
  cfg/PublishNodeDynCfg.cfg
  cfg/SubscribeNodeDynCfg.cfg
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES template_package
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport roscpp rospy sensor_msgs std_msgs
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


##################
## Executasbles ##
##################

# Publisher Node
add_executable(publisher_node src/publisher.cpp)

add_dependencies(publisher_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(publisher_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

# Subscriber Node
add_executable(subscriber_node src/subscriber.cpp)

add_dependencies(subscriber_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(subscriber_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
