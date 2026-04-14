cmake_minimum_required(VERSION 3.8)
project(dog_voice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(QWEN_LIB
        "${CMAKE_CURRENT_SOURCE_DIR}/include/qwen3_tts/lib/libqwen3tts.so")
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/voiceGenerator.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "std_msgs"
)
target_link_libraries(${PROJECT_NAME}
  ${ARCH_LIB}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_package()
