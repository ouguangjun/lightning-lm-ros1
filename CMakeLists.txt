cmake_minimum_required(VERSION 3.5)
project(lightning)

set(CMAKE_BUILD_TYPE "Release") 

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

# -------------------------------------------------------------
# 导入第三方库，同时做一些全局编译选项配置
add_definitions("-DPCL_NO_PRECOMPILE")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fopenmp -fPIC -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O2 -fopenmp -fPIC -g -ggdb")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -ggdb -g -fopenmp -fPIC -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -g")


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)

add_subdirectory(thirdparty/livox_ros_driver)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/packages.cmake)

find_package(catkin REQUIRED COMPONENTS
    roscpp std_msgs geometry_msgs sensor_msgs nav_msgs std_srvs
    tf2 tf2_ros message_filters pcl_ros pcl_conversions
    message_generation livox_ros_driver
)

add_service_files(
  DIRECTORY srv
  FILES
  LocCmd.srv
  SaveMap.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs std_srvs tf2 tf2_ros pcl_conversions message_filters livox_ros_driver message_runtime)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# livox_ros_driver is a separate catkin package; do not add_subdirectory here

add_subdirectory(src)
