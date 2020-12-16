cmake_minimum_required(VERSION 2.8.3)
project(gnss_localizer)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gnss
  nmea_msgs
  roscpp
  std_msgs
  tf
  autoware_msgs
  autoware_config_msgs
  autoware_system_msgs
  pcl_ros
)

catkin_package()

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(fix2tfpose
  nodes/fix2tfpose/fix2tfpose.cpp
)
target_link_libraries(fix2tfpose ${catkin_LIBRARIES})
add_dependencies(fix2tfpose ${catkin_EXPORTED_TARGETS})

add_executable(nmea2tfpose
  nodes/nmea2tfpose/nmea2tfpose_core.cpp
  nodes/nmea2tfpose/nmea2tfpose_node.cpp
)

target_include_directories(nmea2tfpose PRIVATE nodes/nmea2tfpose ${catkin_INCLUDE_DIRS})
target_link_libraries(nmea2tfpose ${catkin_LIBRARIES})
add_dependencies(nmea2tfpose ${catkin_EXPORTED_TARGETS})

add_executable(nmea2tfpose_RTK
        nodes/nmea2tfpose/nmea2tfpose_core_RTK.cpp
        nodes/nmea2tfpose/nmea2tfpose_node.cpp
        )
target_include_directories(nmea2tfpose_RTK PRIVATE nodes/nmea2tfpose ${catkin_INCLUDE_DIRS})
target_link_libraries(nmea2tfpose_RTK ${catkin_LIBRARIES})
add_dependencies(nmea2tfpose_RTK ${catkin_EXPORTED_TARGETS})

add_executable(gnss_selector
        nodes/gnss_selector/gnss_selector.cpp
        )
target_link_libraries(gnss_selector ${catkin_LIBRARIES})
add_dependencies(gnss_selector ${catkin_EXPORTED_TARGETS})

add_executable(RTK_localizer
        nodes/RTK_localizer/gnss_matching.cpp
        nodes/RTK_localizer/gnss_matching_history.cpp
        )
target_link_libraries(RTK_localizer ${catkin_LIBRARIES})
add_dependencies(RTK_localizer ${catkin_EXPORTED_TARGETS})

install(TARGETS nmea2tfpose fix2tfpose nmea2tfpose_RTK gnss_selector RTK_localizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(rosunit REQUIRED)

  add_rostest_gtest(nmea_test test/nmea_test.test test/nmea_test.cpp)
  target_link_libraries(nmea_test ${catkin_LIBRARIES})
  add_dependencies(nmea_test ${catkin_EXPORTED_TARGETS})
endif()
