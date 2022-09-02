cmake_minimum_required(VERSION 3.3)
project(force_torque_sensor_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(controller_interface REQUIRED)

add_library(${PROJECT_NAME} SHARED
        src/force_torque_sensor_controller.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(
        ${PROJECT_NAME}
        controller_interface
        geometry_msgs
        hardware_interface
        realtime_tools
        rclcpp)


target_link_libraries(${PROJECT_NAME} lib/${PROJECT_NAME})


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

# Intall plugins
install(FILES force_torque_sensor_plugin.xml
              config/force_torque_sensor_controller.yaml
              launch/force_torque_sensor_controller.launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
