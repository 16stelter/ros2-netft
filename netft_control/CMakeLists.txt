cmake_minimum_required(VERSION 3.3)
project(netft_control)

find_package(ament_cmake REQUIRED)
find_package(force_torque_sensor_controller REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread program_options)
find_package(CURL REQUIRED)
find_package(TinyXML REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(netft_rdt_driver)
ament_export_dependencies(diagnostic_msgs diagnostic_updater geometry_msgs hardware_interface rclcpp)

include_directories(
 include ${include}
)

## Declare a cpp library
add_library(netft_rdt_driver src/netft_rdt_driver.cpp)
target_link_libraries(netft_rdt_driver 
  ${lib}
  ${Boost_LIBRARIES} 
  ${CURL_LIBRARIES} 
  ${TinyXML_LIBRARIES})

## Declare a cpp executable
add_executable(netft_node src/netft_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(netft_node netft_rdt_driver)

# NetFT ROS controller
add_executable(netft_ros_driver src/netft_ros_driver.cpp)
target_link_libraries(netft_ros_driver netft_rdt_driver ${lib} ${Boost_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS netft_rdt_driver netft_node netft_ros_driver
   ARCHIVE DESTINATION ${lib}
   LIBRARY DESTINATION ${lib}
   RUNTIME DESTINATION ${bin}
 )

ament_package()
