cmake_minimum_required(VERSION 3.8)
project(dog_debugger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(dog_controller REQUIRED)
find_package(pybind11 REQUIRED)

include_directories(include)

pybind11_add_module(legPy
  src/pythonlib/legBindings.cpp
)
target_include_directories(legPy 
    PRIVATE 
    ${dog_controller_INCLUDE_DIRS}
)
target_link_libraries(legPy 
    PRIVATE  
    Eigen3::Eigen 
    ${dog_controller_LIBRARIES}
)

install(
  TARGETS legPy
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
