import sys

filename = "src/dog_remote/CMakeLists.txt"
with open(filename, "r") as f:
    content = f.read()

# 1. Add CanManager.cpp to library
if "src/CanManager.cpp" not in content:
    content = content.replace("src/commandManager.cpp", "src/commandManager.cpp\n    src/CanManager.cpp")

# 2. Fix canTester
if "src/test/canTester.cpp src/CanManager.cpp" in content:
    content = content.replace("src/test/canTester.cpp src/CanManager.cpp", "src/test/canTester.cpp")

# 3. Add exports
if "ament_export_include_directories" not in content:
    content = content.replace("ament_export_dependencies(", "ament_export_include_directories(include)\n"
                                                                "ament_export_libraries(${PROJECT_NAME})\n"
                                                                "ament_export_dependencies(")

with open(filename, "w") as f:
    f.write(content)
