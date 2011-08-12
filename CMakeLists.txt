SET ( PROJECT_NAME media_server )

CMAKE_MINIMUM_REQUIRED ( VERSION 2.4.6 )
INCLUDE ( $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake )
SET ( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMAKE_MODULES/" )

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# Define Windows XP target
IF ( MSVC )
  ADD_DEFINITIONS ( -D_WIN32_WINNT=0x0501 -D__MSVC__ )
ENDIF ()

# Set debug mode (-g) and enable all warnings (-Wall) for GNU compilers
IF ( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  SET ( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -Wno-deprecated -std=c++0x" )
  SET ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wno-deprecated -std=gnu++0x" )
ENDIF ()

#set the default path for built executables to the "bin" directory
SET ( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

#set the default path for built libraries to the "lib" directory
SET ( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

#uncomment if you have defined messages
#rosbuild_genmsg()

#uncomment if you have defined services
rosbuild_gensrv()

# Define header locations
FILE ( GLOB INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/include/*" )

# Define source and header files
FILE ( GLOB INPUT_SOURCES "${CMAKE_SOURCE_DIR}/src/input/*.c*" )
FILE ( GLOB INPUT_HEADERS "${CMAKE_SOURCE_DIR}/include/input/*.h*" )

# Load Boost
SET ( Boost_USE_MULTITHREADED ON )
ROSBUILD_ADD_BOOST_DIRECTORIES()

# Load EXIV
FIND_PACKAGE ( Exiv2 )
IF ( EXIV2_FOUND )
  SET ( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXIV2_DEFINITIONS}" )
  SET ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXIV2_DEFINITIONS}" )
  ADD_DEFINITIONS ( -DHAS_EXIV2 )
ELSE ( EXIV2_FOUND )
  SET ( EXIV2_INCLUDE_DIR )
  REMOVE_DEFINITIONS ( -DHAS_EXIV2 )
ENDIF ( EXIV2_FOUND )

# List include and linking directories
INCLUDE_DIRECTORIES (
  ${Boost_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIR}
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/include/input
  ${CMAKE_CURRENT_BINARY_DIR}
  ${EXIV2_INCLUDE_DIR}
)

LINK_DIRECTORIES (
#  ${Boost_LIBRARY_DIRS}
#  ${OpenCV_LIB_DIR}
)

# Build projects
ROSBUILD_ADD_LIBRARY ( ${PROJECT_NAME}_lib ${INPUT_SOURCES} ${INPUT_HEADERS} )
ROSBUILD_LINK_BOOST ( ${PROJECT_NAME}_lib system thread date_time filesystem )
TARGET_LINK_LIBRARIES ( ${PROJECT_NAME}_lib ${OpenCV_LIBS} ${EXIV2_LIBRARIES} )

ROSBUILD_ADD_EXECUTABLE ( standalone_demo ${CMAKE_SOURCE_DIR}/src/standalone_demo.cpp )
TARGET_LINK_LIBRARIES ( standalone_demo ${PROJECT_NAME}_lib )

ROSBUILD_ADD_EXECUTABLE ( ${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/media_server.cpp )
TARGET_LINK_LIBRARIES ( ${PROJECT_NAME} ${PROJECT_NAME}_lib )
