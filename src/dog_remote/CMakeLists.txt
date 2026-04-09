cmake_minimum_required(VERSION 3.8)
project(dog_remote)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(dog_msg REQUIRED)
find_package(dog_tools REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
    src/twistToGait.cpp
    src/sbusReader.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "geometry_msgs"
  "dog_msg"
  "dog_tools"
)
target_link_libraries(${PROJECT_NAME}
    Boost::system
)

add_executable(subsTester
    src/subsTester.cpp)
ament_target_dependencies(subsTester
    rclcpp
)
target_link_libraries(subsTester
    ${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
    subsTester
  DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
    rclcpp
    dog_tools
    dog_msg
)

ament_package()
