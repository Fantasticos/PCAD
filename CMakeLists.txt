cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCAD_prj)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(/home/zhongyua/dlib/dlib/dlib dlib_build)
  
add_executable (PCAD_prj main.cpp)
target_link_libraries (PCAD_prj ${PCL_LIBRARIES})
target_link_libraries(PCAD_prj dlib::dlib)
add_subdirectory(pcad)
target_link_libraries(PCAD_prj PCAD_LIB)