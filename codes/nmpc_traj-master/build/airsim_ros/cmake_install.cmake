# Install script for directory: /home/bitzoo/nmpc_traj/src/airsim_ros

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/bitzoo/nmpc_traj/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/airsim_ros/msg" TYPE FILE FILES
    "/home/bitzoo/nmpc_traj/src/airsim_ros/msg/GimbalAngleEulerCmd.msg"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/msg/GimbalAngleQuatCmd.msg"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/msg/GPSYaw.msg"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/msg/CarControls.msg"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/msg/CarState.msg"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/msg/Altimeter.msg"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/msg/Environment.msg"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/msg/AngleRateThrottle.msg"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/msg/Circle.msg"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/msg/CirclePoses.msg"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/msg/TreePoses.msg"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/msg/RotorPWM.msg"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/msg/Predloop.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/airsim_ros/srv" TYPE FILE FILES
    "/home/bitzoo/nmpc_traj/src/airsim_ros/srv/SetGPSPosition.srv"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/srv/Takeoff.srv"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/srv/TakeoffGroup.srv"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/srv/Land.srv"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/srv/LandGroup.srv"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/srv/SetLocalPosition.srv"
    "/home/bitzoo/nmpc_traj/src/airsim_ros/srv/TopicHz.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/airsim_ros/cmake" TYPE FILE FILES "/home/bitzoo/nmpc_traj/build/airsim_ros/catkin_generated/installspace/airsim_ros-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/bitzoo/nmpc_traj/devel/include/airsim_ros")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/bitzoo/nmpc_traj/devel/share/roseus/ros/airsim_ros")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/bitzoo/nmpc_traj/devel/share/common-lisp/ros/airsim_ros")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/bitzoo/nmpc_traj/devel/share/gennodejs/ros/airsim_ros")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/bitzoo/nmpc_traj/devel/lib/python3/dist-packages/airsim_ros")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/bitzoo/nmpc_traj/devel/lib/python3/dist-packages/airsim_ros")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/bitzoo/nmpc_traj/build/airsim_ros/catkin_generated/installspace/airsim_ros.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/airsim_ros/cmake" TYPE FILE FILES "/home/bitzoo/nmpc_traj/build/airsim_ros/catkin_generated/installspace/airsim_ros-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/airsim_ros/cmake" TYPE FILE FILES
    "/home/bitzoo/nmpc_traj/build/airsim_ros/catkin_generated/installspace/airsim_rosConfig.cmake"
    "/home/bitzoo/nmpc_traj/build/airsim_ros/catkin_generated/installspace/airsim_rosConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/airsim_ros" TYPE FILE FILES "/home/bitzoo/nmpc_traj/src/airsim_ros/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/airsim_ros" TYPE FILE FILES "/home/bitzoo/nmpc_traj/src/airsim_ros/README.md")
endif()

