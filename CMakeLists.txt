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

## Add executables
add_executable(calibrate
  src/camera_calibration.cpp
  src/undistort_img.cpp
  src/process_map.cpp
  )

## Add shared library
add_library(student SHARED
   src/student_interface.cpp
)



## Link with libraries 
target_link_libraries(student
  ${OpenCV_LIBRARIES}  
)

target_link_libraries(calibrate
  ${OpenCV_LIBRARIES}  
)

#target_link_libraries(student
#  undistrort_img
#)






