cmake_minimum_required(VERSION 3.8)
project(dog_hardware)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|i[3-6]86)$")
    set(ARCH_LIB
        "${CMAKE_CURRENT_SOURCE_DIR}/include/lib/libUnitreeMotorSDK_Linux64.so")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|armv7l|armhf)$")
    set(ARCH_LIB
        "${CMAKE_CURRENT_SOURCE_DIR}/include/lib/libUnitreeMotorSDK_Arm64.so")
else()
    message(FATAL_ERROR "unsupported arch ${CMAKE_SYSTEM_PROCESSOR}")
endif()
# 根据平台选择库文件

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(dog_remote REQUIRED)
find_package(dog_tools REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
    src/base.cpp
    src/baseSingle.cpp
    src/goM8010.cpp
    src/serial.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "pluginlib"
  "hardware_interface"
  "dog_remote"
  "dog_tools"
)
target_link_libraries(${PROJECT_NAME}
  ${ARCH_LIB}
  yaml-cpp::yaml-cpp
  Boost::system
)

add_executable(example_goM8010_6_motor
  src/example_goM8010_6_motor.cpp
)
target_link_libraries(example_goM8010_6_motor
    ${ARCH_LIB}
)

add_executable(zeroSetter 
    src/zeroSetter.cpp)
ament_target_dependencies(zeroSetter
    rclcpp
    yaml-cpp
    ament_index_cpp
)
target_link_libraries(zeroSetter
    yaml-cpp::yaml-cpp
    ${PROJECT_NAME}
)

add_executable(motorTester
    src/motorTester.cpp)
ament_target_dependencies(motorTester
    rclcpp
    std_msgs
)
target_link_libraries(motorTester
    ${PROJECT_NAME}
)

# 注释掉缺少实现的测试程序
# add_executable(DM_Motor_test
#     src/test/DM_Motor_test.cpp
# )
# ament_target_dependencies(DM_Motor_test
#     rclcpp
# )
# target_link_libraries(DM_Motor_test
#     ${PROJECT_NAME}
# )

# add_executable(DJI_Motor_test
#     src/test/DJI_2005_Motor_test.cpp
# )
# ament_target_dependencies(DJI_Motor_test
#     rclcpp
# )
# target_link_libraries(DJI_Motor_test
#     ${PROJECT_NAME}
# )

# add_executable(canTester src/test/canTester.cpp)
# ament_target_dependencies(canTester
#     rclcpp
# )
# target_link_libraries(canTester
#     ${PROJECT_NAME}
# )

pluginlib_export_plugin_description_file(hardware_interface dog_hardwareInterface.xml)


install(
  PROGRAMS ${ARCH_LIB}
  DESTINATION lib
)

install(
  TARGETS ${PROJECT_NAME}
        example_goM8010_6_motor
        zeroSetter
        motorTester
  DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  pluginlib
  hardware_interface
)
ament_package()
