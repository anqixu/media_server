cmake_minimum_required(VERSION 2.8.3)
project(media_server)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -Wall -O2 -Wno-deprecated")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/CMAKE_MODULES/")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  image_transport
  camera_calibration_parsers
  cv_bridge
  message_generation
)

# Load Boost
set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED COMPONENTS system thread date_time filesystem)

# Load OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

# Load EXIV
find_package(Exiv2)
if(${EXIV2_FOUND})
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXIV2_DEFINITIONS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXIV2_DEFINITIONS}")
  add_definitions(-DHAS_EXIV2)
else()
  set(EXIV2_INCLUDE_DIR)
  remove_definitions(-DHAS_EXIV2)
endif()

#######################################
## Declare ROS messages and services ##
#######################################

#add_message_files(
#  FILES Message1.msg
#)

add_service_files(
  FILES
    GetMediaStatus.srv
    LoadImageList.srv
    LoadLoggedImageList.srv
    LoadVideoDevice.srv
    LoadVideoFile.srv
    SeekIndex.srv
    SetTimeMultiplier.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs std_srvs sensor_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs std_srvs sensor_msgs image_transport camera_calibration_parsers cv_bridge message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${EXIV2_INCLUDE_DIR}
)
link_directories(
  ${OpenCV_LIB_DIR}
)

# Define source and header files
FILE ( GLOB INPUT_SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/input/*.c*" )
FILE ( GLOB INPUT_HEADERS "${CMAKE_CURRENT_LIST_DIR}/include/media_server/*.h*" )


# Build projects
add_library(${PROJECT_NAME}_lib ${INPUT_SOURCES} ${INPUT_HEADERS})
target_link_libraries(${PROJECT_NAME}_lib ${OpenCV_LIBS} ${EXIV2_LIBRARIES} ${Boost_LIBRARIES})

add_executable(standalone_demo src/standalone_demo.cpp)
target_link_libraries(standalone_demo ${PROJECT_NAME}_lib)

add_executable(${PROJECT_NAME} src/media_server.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_lib standalone_demo ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ftag2.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
