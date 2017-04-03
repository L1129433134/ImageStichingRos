cmake_minimum_required(VERSION 2.8.3)
project(iris_automation_test)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  image_transport
  roslaunch
)
find_package( OpenCV REQUIRED )
find_package(Boost REQUIRED COMPONENTS thread)
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iris_automation_test
#  CATKIN_DEPENDS cv_bridge roscpp std_msgs
#  DEPENDS system_lib
)
add_library(node_merged_video
    src/node_merged_videos.cpp src/main.cpp)
target_link_libraries(node_merged_video  ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS}  ${Boost_INCLUDE_DIRS})

add_definitions(-std=c++11)

include_directories( include
  ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)


add_executable(node_merged_videos src/node_merged_videos.cpp src/main.cpp)
target_link_libraries(node_merged_videos  ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS}  ${Boost_INCLUDE_DIRS})
roslaunch_add_file_check(launch)

catkin_add_gtest(myUnitTest tests/utest.cpp)
target_link_libraries(myUnitTest  ${catkin_LIBRARIES} node_merged_video)


