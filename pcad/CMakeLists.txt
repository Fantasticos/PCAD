add_library(PCAD_LIB
PCAD.cpp
PCAD.h
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#target_link_libraries (PCAD_LIB ${PCL_LIBRARIES})
target_include_directories(PCAD_LIB PUBLIC .)
include_directories(/home/zhongyua/dlib/dlib)