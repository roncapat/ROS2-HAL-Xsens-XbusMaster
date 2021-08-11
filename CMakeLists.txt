cmake_minimum_required(VERSION 3.3)
project(hal_xsens_xmb)

set(CMAKE_VERBOSE_MAKEFILE OFF)

add_definitions(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

add_library(cmt STATIC 
    src/libcmt/cmt1.cpp  
    src/libcmt/cmt2.cpp  
    src/libcmt/cmt3.cpp  
    src/libcmt/cmtmessage.cpp  
    src/libcmt/cmtpacket.cpp  
    src/libcmt/cmtscan.cpp  
    src/libcmt/xsens_std.cpp  
    src/libcmt/xsens_time.cpp)
target_compile_options(cmt PUBLIC -Wall -Wextra -Wno-attributes -Wno-attribute-warning)
target_include_directories(cmt PUBLIC include/libcmt)
target_link_libraries(cmt -lrt)

execute_process(COMMAND python3-config --extension-suffix OUTPUT_VARIABLE PYBIND11_SUFFIX)
string(STRIP ${PYBIND11_SUFFIX} PYBIND11_SUFFIX)

execute_process(COMMAND python3 -m pybind11 --includes OUTPUT_VARIABLE PYBIND11_INCLUDES)
string(STRIP ${PYBIND11_INCLUDES} PYBIND11_INCLUDES)
string(REPLACE "-I" " " PYBIND11_INCLUDES_LIST ${PYBIND11_INCLUDES})
string(STRIP ${PYBIND11_INCLUDES_LIST} PYBIND11_INCLUDES_LIST)
string(REPLACE " " ";" PYBIND11_INCLUDES_LIST ${PYBIND11_INCLUDES_LIST})

add_library(xsens_XMB SHARED src/xsens_XMB.cpp)
set_target_properties(xsens_XMB PROPERTIES PREFIX "")  
set_target_properties(xsens_XMB PROPERTIES SUFFIX ${PYBIND11_SUFFIX})
target_compile_options(cmt PUBLIC -O3 -mtune=native -march=native -DNDEBUG -Wall -Wextra -Wno-attributes)
target_include_directories(cmt PUBLIC include "$<BUILD_INTERFACE:${PYBIND11_INCLUDES_LIST}>")
target_link_libraries(xsens_XMB cmt)

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

# Install python scripts
install(PROGRAMS scripts/imu_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/libcmt
  DESTINATION include
  FILES_MATCHING PATTERN "*.h")

install(
  TARGETS cmt
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS xsens_XMB
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

# Export package dependencies
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclpy)
ament_export_libraries(${PROJECT_NAME})

ament_package()