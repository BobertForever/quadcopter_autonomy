cmake_minimum_required(VERSION 2.8.3)
project(ardrone_fly)
find_package(catkin REQUIRED COMPONENTS 
    ardrone_autonomy
    cmvision
    geometry_msgs
    sensor_msgs
    std_msgs
    rosleap
	topic_tools)
catkin_package()

add_executable(kalmanBallTrack src/kalmanBallTrack.cpp)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(kalmanBallTrack ${OpenCV_LIBRARIES})
target_link_libraries(kalmanBallTrack ${catkin_LIBRARIES})
install(TARGETS kalmanBallTrack RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(kalmanBallTrack ${catkin_EXPORTED_TARGETS})

add_executable(kalmanHoopTrack src/kalmanHoopTrack.cpp)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(kalmanHoopTrack ${OpenCV_LIBRARIES})
target_link_libraries(kalmanHoopTrack ${catkin_LIBRARIES})
install(TARGETS kalmanHoopTrack RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(kalmanHoopTrack ${catkin_EXPORTED_TARGETS})

add_executable(ballFollow src/ballFollow.cpp)
target_link_libraries(ballFollow ${catkin_LIBRARIES})
install(TARGETS ballFollow RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(ballFollow ${catkin_EXPORTED_TARGETS})

add_executable(wiimoteFly src/wiimoteFly.cpp)
target_link_libraries(wiimoteFly ${catkin_LIBRARIES})
install(TARGETS wiimoteFly RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(wiimoteFly ${catkin_EXPORTED_TARGETS})

add_executable(leapFly src/leapFly.cpp)
target_link_libraries(leapFly ${catkin_LIBRARIES})
install(TARGETS leapFly RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(leapFly ${catkin_EXPORTED_TARGETS})

add_executable(ballPosFly src/ballPosFly.cpp)
target_link_libraries(ballPosFly ${catkin_LIBRARIES})
install(TARGETS ballPosFly RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(ballPosFly ${catkin_EXPORTED_TARGETS})

add_executable(hoopTrickFly src/hoopTrickFly.cpp)
target_link_libraries(hoopTrickFly ${catkin_LIBRARIES})
install(TARGETS hoopTrickFly RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(hoopTrickFly ${catkin_EXPORTED_TARGETS})

add_executable(wiiMuxSwitch src/wiiMuxSwitch.cpp)
target_link_libraries(wiiMuxSwitch ${catkin_LIBRARIES})
install(TARGETS wiiMuxSwitch RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(wiiMuxSwitch ${catkin_EXPORTED_TARGETS})


