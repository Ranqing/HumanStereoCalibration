cmake_minimum_required(VERSION 2.8.1 FATAL_ERROR)

set (PROJECT_NAME StereoCalibration)
project( ${PROJECT_NAME} )

option(WITH_OPENCV "" ON)
option(WITH_PCL "" ON)

file(GLOB PROJECT_FILES
 "*.h"
 "*.cpp"
)

if(WITH_OPENCV)
	find_package(OpenCV REQUIRED)
	include_directories( ${OpenCV_INCLUDE_DIRS} )
	link_directories( ${OpenCV_LIB_DIR} )
endif(WITH_OPENCV)

if(WITH_PCL)
	find_package(PCL 1.8 REQUIRED)
        include_directories(${PCL_INCLUDE_DIRS})
        link_directories(${PCL_LIBRARY_DIRS})
        add_definitions(${PCL_DEFINITIONS})
endif(WITH_PCL)

add_executable ( ${PROJECT_NAME} ${PROJECT_FILES})

if(WITH_OPENCV)
	target_link_libraries ( ${PROJECT_NAME} ${OpenCV_LIBS} )
endif(WITH_OPENCV)

if(WITH_PCL)
	target_link_libraries ( ${PROJECT_NAME} ${PCL_LIBRARIES} )
endif(WITH_PCL)

