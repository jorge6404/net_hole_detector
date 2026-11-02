cmake_minimum_required(VERSION 3.21)
project(net_hole_detector_rust)

find_package(ament_cmake REQUIRED)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

include(r2r_cargo.cmake)

# Paquetes de ROS que usa el crate (sus mensajes los genera r2r al compilar)
r2r_cargo(rcl                          # API de C de ROS 2
          rcl_action                   # r2r también la necesita
          std_msgs
          sensor_msgs
          rosgraph_msgs                # /clock para use_sim_time
          net_hole_detector_msgs_rust  # nuestros mensajes
         )

# El ejecutable que compila cargo (perfil colcon) es el nodo
install(PROGRAMS
  ${CMAKE_SOURCE_DIR}/target/colcon/net_hole_detector_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_node.launch.py TIMEOUT 180)
endif()

ament_package()
