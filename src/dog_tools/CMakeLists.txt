cmake_minimum_required(VERSION 3.8)
project(dog_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
    src/pid.cpp
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
