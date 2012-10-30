##############################################################
# CMakeLists.txt for a ROS package within the REXOS build system.
#
# Created by Dennis Koole, 2012-10-29
#
# 2012-29-10 Initial version
#
#############################################################

cmake_minimum_required(VERSION 2.4.6)
# Include all rosbuild macros
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Initialize ROS package to be in our build system.
rexos_rospack_init("environmentNode" true true)

# Add executables as documented by ROS
file(GLOB sources src/*.cpp)
rosbuild_add_executable(EnvironmentNode ${sources})

# Add dependencies to our libraries and other ROS packages in the same directory as this one.
add_dependencies(EnvironmentNode tinyxpath)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake
include_directories(${TINYXPATH})

target_link_libraries(EnvironmentNode tinyxpath)

# MUST be called after lcv_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()