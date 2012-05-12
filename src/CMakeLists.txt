PROJECT("CameraCalibration")
SET(CAMERACALIBRATION_VERSION_MAJOR 0)
SET(CAMERACALIBRATION_VERSION_MINOR 1)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules/)
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

FIND_PACKAGE(GLEW REQUIRED)
IF(GLEW_FOUND)
    SET(HAVE_GLEW 1)
    #   IF(MSVC)
    #   ADD_DEFINITIONS(-DGLEW_STATIC)
    #   ENDIF(MSVC)
    MESSAGE("GLEW Found and Enabled")
ENDIF(GLEW_FOUND)

FIND_PACKAGE(FREEGLUT REQUIRED)
IF(FREEGLUT_FOUND)
   SET(HAVE_GLUT 1)
   #IF(MSVC)
   #   ADD_DEFINITIONS(-DFREEGLUT_STATIC)
   #ENDIF(MSVC)
   MESSAGE("Freeglut Found and Enabled")
ENDIF(FREEGLUT_FOUND)

FIND_PACKAGE(Pangolin REQUIRED)
IF(Pangolin_FOUND)
    SET(HAVE_PANGOLIN 1)
    MESSAGE("Pangolin Found and Enabled")
ENDIF(Pangolin_FOUND)

FIND_PACKAGE(OpenCV REQUIRED)
IF(OpenCV_FOUND)
    SET(HAVE_OPENCV 1)
    MESSAGE("OpenCV Found and Enabled")
ENDIF(OpenCV_FOUND)

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
TARGET_LINK_LIBRARIES(opencv_calib ${OpenCV_LIBS})

ADD_EXECUTABLE(CameraClibrator CameraCalibrator.cpp CameraCalibrator.h)
INCLUDE_DIRECTORIES(CameraClibrator ${PANGOLIN_INCLUDE_DIR}
                                 ${FREEGLUT_INCLUDE_DIR} 
                                 ${GLEW_INCLUDE_DIR} 
                                 ${Boost_INCLUDE_DIRS}
                                 ${OpenCV_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(CameraClibrator ${OpenCV_LIBS} ${PANGOLIN_LIBRARY} ${Boost_LIBRARIES} ${GLEW_LIBRARY} ${FREEGLUT_LIBRARY})
