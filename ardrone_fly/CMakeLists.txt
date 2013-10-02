cmake_minimum_required(VERSION 2.8.3)
project(ardrone_fly)
find_package(catkin REQUIRED COMPONENTS 
    ardrone_autonomy
    cmvision
    geometry_msgs
    sensor_msgs
    std_msgs)
catkin_package()

add_executable(ballFollow src/ballFollow.cpp)
target_link_libraries(ballFollow ${catkin_LIBRARIES})
install(TARGETS ballFollow RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(ballFollow ${catkin_EXPORTED_TARGETS})

add_executable(wiimoteFly src/wiimoteFly.cpp)
target_link_libraries(wiimoteFly ${catkin_LIBRARIES})
install(TARGETS wiimoteFly RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(wiimoteFly ${catkin_EXPORTED_TARGETS})
