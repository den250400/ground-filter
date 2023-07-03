cmake_minimum_required(VERSION 3.0.0)
project(ground-filter VERSION 0.1.0 LANGUAGES C CXX)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_STANDARD 11)

find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ground_filter src/main.cpp src/simple_hdl_viewer.cpp)
target_link_libraries(ground_filter ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
