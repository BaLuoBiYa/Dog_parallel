cmake_minimum_required(VERSION 3.8)
project(dog_voice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(SDL2 REQUIRED)

set(QWEN_LIB
        "${CMAKE_CURRENT_SOURCE_DIR}/include/qwen3_tts/lib/libqwen3tts.so")
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/voiceGenerater.cpp
  src/voicePlayer.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "std_msgs"
  "SDL2"
)
target_link_libraries(${PROJECT_NAME}
  ${QWEN_LIB}
)

add_executable(dogSpeaker
    src/dogSpeaker.cpp)
ament_target_dependencies(dogSpeaker
    rclcpp
    ament_index_cpp
)
target_link_libraries(dogSpeaker
    ${PROJECT_NAME}

)

install(
  TARGETS ${PROJECT_NAME}
    dogSpeaker
  DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model
)
ament_package()
