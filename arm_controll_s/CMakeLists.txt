cmake_minimum_required(VERSION 2.8.3)
project(arm_controll_s)

find_package(catkin REQUIRED COMPONENTS
  
  dynamixel_sdk
  roscpp
  sensor_msgs
  std_msgs
  dynamixel_workbench_toolbox
)


catkin_package(

)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  )

 add_executable(${PROJECT_NAME}_node src/arm_controll.cpp)

target_link_libraries(${PROJECT_NAME}_node
${Dynamixel_sdk_LIBRARIES}
${catkin_LIBRARIES}
 )