cmake_minimum_required(VERSION 3.8)
project(dog_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(dog_tools REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/standTrajectory.cpp
  src/trajectoryInterface.cpp
)
ament_target_dependencies(${PROJECT_NAME} 
    rclcpp
    dog_tools)

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_package()
