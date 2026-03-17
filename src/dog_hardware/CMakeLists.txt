cmake_minimum_required(VERSION 3.8)
project(dog_hardware)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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

include_directories(include)

add_library(${PROJECT_NAME} SHARED
    src/Base.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "pluginlib"
  "hardware_interface"
)
target_link_libraries(${PROJECT_NAME}
  ${ARCH_LIB}
)

add_executable(example_goM8010_6_motor
  src/example_goM8010_6_motor.cpp
)
target_link_libraries(example_goM8010_6_motor
    ${ARCH_LIB}
)

pluginlib_export_plugin_description_file(hardware_interface dog_hardwareInterface.xml)

install(
    TARGETS ${PROJECT_NAME}
        example_goM8010_6_motor
    DESTINATION lib
)

install(
  TARGETS ${PROJECT_NAME}
        example_goM8010_6_motor
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
