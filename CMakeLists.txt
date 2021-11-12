#cmake_minimum_required(VERSION 2.8)
#project(my_gtsam CXX C)
#
#add_compile_options(-std=c++11)
#set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_NAME}/build/..)
#
#aux_source_directory(${PROJECT_SOURCE_DIR}  source_cpp)
#
#find_package(OpenCV 4.1 REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
#
#find_package(GTSAM REQUIRED)
#include_directories(${GTSAM_INCLUDE_DIR})
#set(GTSAM_LIBRARIES gtsam)
#
#find_package(GTSAMCMakeTools)
#include(GtsamMakeConfigFile)
#include(GtsamBuildTypes)
#include(GtsamTesting)
#
#
#
#
#add_executable(my_gtsam  )
#target_link_libraries(my_gtsam ${OpenCV_LIBS})


cmake_minimum_required(VERSION 2.8)

project(my_gtsam)
add_compile_options(-std=c++11 )
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/..)
aux_source_directory(${PROJECT_SOURCE_DIR}/yaml_load_c yaml_load_c_src)

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(GTSAM REQUIRED QUIET)
link_directories(${GTSAM_LIBRARY_DIRS})
include_directories("/usr/local/include/eigen3")


find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
aux_source_directory(${PROJECT_SOURCE_DIR}/sam1.cpp create_factor_src)
add_executable(create_factor_exe ${create_factor_src})
target_link_libraries(create_factor_exe ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} -ltbb -lgtsam -pthread)
