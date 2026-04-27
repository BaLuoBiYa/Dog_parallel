cmake_minimum_required(VERSION 3.8)
project(dog_remote)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(dog_msg REQUIRED)
find_package(dog_tools REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)


include_directories(include)

add_library(${PROJECT_NAME} SHARED
    src/sbusReader.cpp
    src/sbusReader.cpp
    src/serialConfig.cpp
    src/commandManager.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "geometry_msgs"
  "std_srvs"
  "dog_msg"
  "dog_tools"
)

target_link_libraries(${PROJECT_NAME}
    Boost::system
    yaml-cpp
)

add_executable(sbusTester
    src/test/sbusTester.cpp)
ament_target_dependencies(sbusTester
    rclcpp
)
target_link_libraries(sbusTester
    ${PROJECT_NAME}
)

add_executable(twistToGaitTester
    src/test/twistToGaitTester.cpp)
ament_target_dependencies(twistToGaitTester
    rclcpp
)
target_link_libraries(twistToGaitTester
    ${PROJECT_NAME}
)

# add_executable(commandManager
#     src/commandManager.cpp)
# ament_target_dependencies(commandManager
#     rclcpp
#     std_srvs
#     dog_msg
# )
# target_link_libraries(commandManager
#     ${PROJECT_NAME}
# )

install(
  TARGETS ${PROJECT_NAME}
    sbusTester
    twistToGaitTester
    # commandManager
  DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
    rclcpp
    dog_tools
    dog_msg
)

ament_package()
