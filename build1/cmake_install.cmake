# Install script for directory: /home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libapriltags.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libapriltags.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libapriltags.so"
         RPATH "/usr/local/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/build1/lib/libapriltags.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libapriltags.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libapriltags.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libapriltags.so"
         OLD_RPATH "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/build1/lib:/home/yao/tool/opencv-3.4.0/build/lib:/usr/local/lib:"
         NEW_RPATH "/usr/local/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libapriltags.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/AprilTags" TYPE FILE FILES
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/Tag25h7.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/Quad.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/Tag36h11_other.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/Tag36h9.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/TagDetection.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/TagFamily.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/Segment.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/~$gDetection.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/GrayModel.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/FloatImage.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/pch.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/XYWeight.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/UnionFindSimple.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/Homography33.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/TagDetector.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/Tag16h5_other.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/Tag36h11.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/Tag36h8.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/GLine2D.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/Gridder.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/Edge.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/Tag16h5.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/Gaussian.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/Tag25h9.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/~$gDetector.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/GLineSegment2D.h"
    "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/AprilTags/MathUtil.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/build1/lib/pkgconfig/apriltags.pc")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/build1/example/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/yao/projects/code_backup/circle_fit_distance_module/3rdparty/apriltag/apriltags/build1/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
