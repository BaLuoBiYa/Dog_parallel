cmake_minimum_required(VERSION 3.8)
project(dog_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
# find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(dog_tools REQUIRED)
find_package(dog_trajectory REQUIRED)
find_package(dog_msg REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/leg.cpp
  src/trajectoryTracker.cpp
  src/trajectoryTrackerSingle.cpp
  src/gaitManager.cpp
)
ament_export_libraries(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} 
    rclcpp
    controller_interface
    Eigen3
    dog_tools
    dog_trajectory
    dog_msg)
target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    yaml-cpp
)

# add_executable(PinocchionTest
#   src/test/PinocchionTest.cpp
# )
# ament_target_dependencies(PinocchionTest
#   rclcpp
#   pinocchio
# )
# target_link_libraries(PinocchionTest
#     pinocchio::pinocchio
#     Eigen3::Eigen
# )

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

add_executable(legTest 
    src/test/legTest.cpp)
ament_target_dependencies(legTest
    rclcpp
    dog_tools
    Eigen3
)
target_link_libraries(legTest
    Eigen3::Eigen
    ${PROJECT_NAME}
)

add_executable(legCli
    src/test/legCli.cpp)
ament_target_dependencies(legCli
    rclcpp
    dog_tools
    Eigen3
)
target_link_libraries(legCli
    Eigen3::Eigen
    ${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(controller_interface dog_controllerInterface.xml)

install(
  TARGETS ${PROJECT_NAME}
        # PinocchionTest
        standTest
        walkTest
        jumpTest
        jumpLegacyTest
        legTest
                legCli
  DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_export_dependencies(
    rclcpp
    controller_interface
    pinocchio 
    Eigen3
    dog_tools
    dog_trajectory
    dog_msg
)
ament_package()
