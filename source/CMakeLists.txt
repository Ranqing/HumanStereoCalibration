cmake_minimum_required(VERSION 2.8.1 FATAL_ERROR)

set (PROJECT_NAME StereoCalibration)
project( ${PROJECT_NAME} )

option(WITH_OPENCV "" ON)

file(GLOB PROJECT_FILES
  "*.h"
  "*.cpp"
  "*.txt"
)

if(WITH_OPENCV)
	find_package(OpenCV 2.4.9 REQUIRED)
	include_directories( ${OpenCV_INCLUDE_DIRS} )
	link_directories( ${OpenCV_LIB_DIR} )
endif(WITH_OPENCV)

add_executable ( ${PROJECT_NAME} ${PROJECT_FILES})

if(WITH_OPENCV)
	target_link_libraries ( ${PROJECT_NAME} ${OpenCV_LIBS} )
endif(WITH_OPENCV)

