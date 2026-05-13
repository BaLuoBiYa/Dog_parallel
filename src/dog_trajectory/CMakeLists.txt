cmake_minimum_required(VERSION 3.8)
project(dog_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(dog_tools REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/standTrajectory.cpp
  src/walkTrajectory.cpp
  src/jumpTrajectory.cpp
)
target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    dog_tools
)

install(
  DIRECTORY include/
  DESTINATION include/
)
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(
    rclcpp
    dog_tools
    yaml-cpp
)

ament_package()
