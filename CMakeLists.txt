cmake_minimum_required(VERSION 3.11)

PROJECT(view3d)
SET(MY_DIR ${view3d_SOURCE_DIR})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_MODULE_PATH "~/workspace/opencv-3.4.1")

#set the default path for built executables to the "bin" directory
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

#SET(SOURCES
#   include/view3d.h
#   src/view3d.cpp)

include_directories(include)
add_executable(view3d src/view3d.cpp)

# dependencies
FIND_PACKAGE(PkgConfig)   
#FIND_PACKAGE(PCL 1.3 REQUIRED)
FIND_PACKAGE(freenect2 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(VTK REQUIRED)
FIND_PACKAGE(PCL 1.3 REQUIRED COMPONENTS io common visualization) 


include_directories( ${PCL_INCLUDE_DIRS} )
  add_definitions( ${PCL_DEFINITIONS} )
  link_directories( ${PCL_LIBRARY_DIRS} )
  target_link_libraries(view3d 
			${PCL_COMMON_LIBRARIES} 
			${PCL_IO_LIBRARIES} 
			${PCL_VISUALIZATION_LIBRARIES}
			${PCL_LIBRARIES}
			${OpenCV_LIBS}
			${freenect2_LIBRARIES})

# Add includes
INCLUDE_DIRECTORIES(
  "${MY_DIR}/include"
  "${MY_DIR}/include/internal"
  ${PROJECT_BINARY_DIR} # for generated headers
)


