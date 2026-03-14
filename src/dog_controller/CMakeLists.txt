cmake_minimum_required(VERSION 3.8)
project(dog_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include)

add_library(${PROJECT_NAME}
  src/model.cpp
  src/PinocchionTest.cpp
)

target_link_libraries(${PROJECT_NAME}
    pinocchio::pinocchio
    Eigen3::Eigen
)

add_executable(PinocchionTest
  src/PinocchionTest.cpp
)
target_link_libraries(PinocchionTest
    ${PROJECT_NAME}
)

ament_target_dependencies(PinocchionTest 
    rclcpp
    pinocchio 
    Eigen3)

install(
  TARGETS ${PROJECT_NAME}
        PinocchionTest
  DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
