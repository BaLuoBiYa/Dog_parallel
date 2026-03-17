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
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/standTrajectory.cpp
  src/trajectoryInterface.cpp
)
target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME} 
    rclcpp
    dog_tools)

    
add_executable(bezierTest
  src/test/bezierTest.cpp
)
ament_target_dependencies(bezierTest
    rclcpp
    dog_tools
    Eigen3)
target_link_libraries(bezierTest
  Eigen3::Eigen
)

install(
  TARGETS ${PROJECT_NAME}
    bezierTest
  DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_package()
