PROJECT("CameraCalibration")
SET(CAMERACALIBRATION_VERSION_MAJOR 0)
SET(CAMERACALIBRATION_VERSION_MINOR 1)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

OPTION(USE_SHARED_LIB "Link shared libraries" ON)

IF(MSVC)
   SET(BOOST_ROOT "C:/boost/boost_1_49_0")
   #   SET(Boost_USE_STATIC_LIBS ON)
   #   SET(Boost_USE_STATIC_RUNTIME ON)
ENDIF(MSVC)
FIND_PACKAGE(Boost COMPONENTS thread date_time REQUIRED)
IF(Boost_FOUND)
  MESSAGE("Boost Found and Enabled: ${Boost_INCLUDE_DIR}")
ENDIF(Boost_FOUND)

FIND_PACKAGE(OpenGL REQUIRED)

###############################################################################
# Pangolin: https://github.com/pinglin/Pangolin.git
FIND_PACKAGE(Pangolin REQUIRED)
INCLUDE_DIRECTORIES(${Pangolin_INCLUDE_DIRS})
LINK_DIRECTORIES(${Pangolin_LIBRARY_DIRS})
LINK_LIBRARIES(${Pangolin_LIBRARIES})

###############################################################################
# OpenCV: http://opencv.willowgarage.com/wiki/
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
LINK_LIBRARIES(${OpenCV_LIBS})

# Overide with cmake -DCMAKE_BUILD_TYPE=Debug {dir}
IF( NOT CMAKE_BUILD_TYPE )
   MESSAGE("Build type not set (defaults to release)")
   MESSAGE("-DCMAKE_BUILD_TYPE=Debug for debug")
   SET( CMAKE_BUILD_TYPE Release )
ENDIF()

## Set Special Compiler flags
IF(MSVC)
  SET(CMAKE_CXX_FLAGS "/EHs")
ELSEIF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "-Wall -Wno-error=deprecated-declarations -Werror")
ENDIF(MSVC)

#IF(MSVC)
#We statically link to reduce dependancies
#FOREACH(flag_var
#        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
#        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
#   IF(${flag_var} MATCHES "/MD")
#      string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
#   ENDIF(${flag_var} MATCHES "/MD")
#ENDFOREACH(flag_var)
#ENDIF(MSVC)

ADD_EXECUTABLE(opencv_calib opencv_calib.cpp)
ADD_EXECUTABLE(CameraClibrator CameraCalibrator.cpp CameraCalibrator.h)
