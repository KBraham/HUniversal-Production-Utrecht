##############################################################
# CMakeLists.txt for a ROS package within the REXOS build system.
#
# Created by Koen Braham & Daan Veltman, 2012-10-18
#
# 2012-10-18 Initial version
#
#############################################################

cmake_minimum_required(VERSION 2.4.6)
# Include all rosbuild macros
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Initialize ROS package to be in our build system.
rexos_rospack_init("crateTrackerNode" true true)

# Add executables as documented by ROS
rosbuild_add_executable(crateTrackerNode src/CrateTrackerNode.cpp)
rosbuild_add_executable(crateEventPublisher src/CrateEventPublisher.cpp)

# MUST be called after lcv_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()