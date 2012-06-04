cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#common commands for building c++ executables and libraries
rosbuild_add_library(sensors src/i2cfile.cpp
                             src/avr.cpp
                             src/adxl345.cpp
                             src/itg3200.cpp
                             src/hmc5883l.cpp
                             src/micromag.cpp
                             src/bmp085.cpp)
rosbuild_add_library(estimators src/ekfAttQuat.cpp
                                src/ekfAtt.cpp
                                src/ekfYaw.cpp
                                src/ekfPos.cpp
                                src/kalman.cpp)

rosbuild_add_executable(fmFusion src/fmFusion.cpp)
target_link_libraries(fmFusion sensors estimators)

# rosbuild_add_executable(fmMag src/fmMag.cpp)
# target_link_libraries(fmMag sensors estimators)

#rosbuild_add_executable(fmKalmanAndRaw src/fmKalmanAndRaw.cpp)
#target_link_libraries(fmKalmanAndRaw sensors estimators)
