## Minimum version of cmake
cmake_minimum_required(VERSION 2.8.3)

## Name of the project
project(student_project)

## Compile as C++11
add_compile_options(-std=c++11)

## Required packages
find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )

## Specify locations of header files
include_directories(
 include 
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
)


add_library(undistort_img SHARED
   src/undistort_img.cpp
)

add_library(process_Map SHARED
   src/process_Map.cpp
)

add_library(PlanPath_Interface SHARED
   src/PlanPath_Interface.cpp
)

add_library(Voronoi SHARED
   src/Voronoi.cpp
)

add_library(Dijkstra SHARED
   src/Dijkstra.cpp
)

add_library(PathFinder SHARED
   src/PathFinder.cpp
)

add_library(Dubins SHARED
   src/Dubins.cpp
)

add_library(find_robot SHARED
   src/find_robot.cpp
)


## Add executables
add_executable(calibrate
  src/camera_calibration.cpp
  src/undistort_img.cpp
  src/process_Map.cpp
  src/Voronoi.cpp
  src/PlanPath_Interface.cpp
  src/Dijkstra.cpp
  src/clipper.cpp
  src/Dubins.cpp
  src/PathFinder.cpp
  src/find_robot.cpp
  
  )

## Add shared library
add_library(student SHARED
   src/student_interface.cpp
   
)



## Link with libraries 
target_link_libraries(student
  ${OpenCV_LIBRARIES}
  undistort_img 
  process_Map 
  PlanPath_Interface
  Voronoi
  PathFinder
  Dijkstra
  Dubins
  find_robot
)

target_link_libraries(calibrate
  ${OpenCV_LIBRARIES}  
)






