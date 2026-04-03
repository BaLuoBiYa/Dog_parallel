cmake_minimum_required(VERSION 3.8)
project(dog_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(dog_tools REQUIRED)
find_package(dog_trajectory REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/leg.cpp
  src/trajectoryTracker.cpp
)
ament_target_dependencies(${PROJECT_NAME} 
    rclcpp
    controller_interface
    pinocchio 
    Eigen3
    dog_tools
    dog_trajectory)
target_link_libraries(${PROJECT_NAME}
    pinocchio::pinocchio
    Eigen3::Eigen
)

add_executable(PinocchionTest
  src/test/PinocchionTest.cpp
)
ament_target_dependencies(PinocchionTest
  rclcpp
  pinocchio
)
target_link_libraries(PinocchionTest
    pinocchio::pinocchio
    Eigen3::Eigen
)

add_executable(standTest
    src/test/standTest.cpp
)
ament_target_dependencies(standTest
    rclcpp
    dog_tools
    dog_trajectory
    Eigen3
    ament_index_cpp
)
target_link_libraries(standTest
    Eigen3::Eigen
    yaml-cpp
)

add_executable(walkTest
    src/test/walkTest.cpp
)
ament_target_dependencies(walkTest
    rclcpp
    dog_tools
    dog_trajectory
    Eigen3
    ament_index_cpp
)
target_link_libraries(walkTest
    Eigen3::Eigen
    yaml-cpp
)

add_executable(jumpTest
    src/test/jumpTest.cpp
)
ament_target_dependencies(jumpTest
    rclcpp
    dog_tools
    dog_trajectory
    Eigen3
    ament_index_cpp
)
target_link_libraries(jumpTest
    Eigen3::Eigen
    yaml-cpp
)

add_executable(jumpLegacyTest
    src/test/jumpLegacyTest.cpp
)
ament_target_dependencies(jumpLegacyTest
    rclcpp
    dog_tools
    dog_trajectory
    Eigen3
    ament_index_cpp
)
target_link_libraries(jumpLegacyTest
    Eigen3::Eigen
    yaml-cpp
)

install(
  TARGETS ${PROJECT_NAME}
        PinocchionTest
        standTest
        walkTest
        jumpTest
        jumpLegacyTest
  DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY parameters/
  DESTINATION share/${PROJECT_NAME}/parameters
)

ament_export_include_directories(include)
ament_export_dependencies(
    rclcpp
    controller_interface
    pinocchio 
    Eigen3
    dog_tools
    dog_trajectory
)
ament_package()
