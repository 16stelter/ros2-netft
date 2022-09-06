cmake_minimum_required(VERSION 3.3)
project(netft_control)

find_package(ament_cmake REQUIRED)
find_package(force_torque_sensor_broadcaster REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread program_options)
find_package(CURL REQUIRED)
find_package(tinyxml_vendor REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(netft_rdt_driver)

include_directories(
 include
)

## Declare a cpp library
add_library(netft_rdt_driver src/netft_rdt_driver.cpp)
target_link_libraries(netft_rdt_driver 
  lib/${PROJECT_NAME}
  ${Boost_LIBRARIES} 
  ${CURL_LIBRARIES} 
  ${TinyXML_LIBRARIES})
ament_target_dependencies(netft_rdt_driver diagnostic_msgs diagnostic_updater geometry_msgs hardware_interface rclcpp)


## Declare a cpp executable
add_executable(netft_node src/netft_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(netft_node netft_rdt_driver)
ament_target_dependencies(netft_node diagnostic_msgs std_srvs diagnostic_updater geometry_msgs hardware_interface rclcpp)


# NetFT ROS controller
add_executable(netft_ros_driver src/netft_ros_driver.cpp)
target_link_libraries(netft_ros_driver netft_rdt_driver lib/${PROJECT_NAME} ${Boost_LIBRARIES})
ament_target_dependencies(netft_ros_driver diagnostic_msgs diagnostic_updater geometry_msgs hardware_interface rclcpp)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS netft_rdt_driver netft_node netft_ros_driver
   ARCHIVE DESTINATION lib/${PROJECT_NAME}
   LIBRARY DESTINATION lib/${PROJECT_NAME}
   RUNTIME DESTINATION bin/${PROJECT_NAME}
 )

ament_package()
