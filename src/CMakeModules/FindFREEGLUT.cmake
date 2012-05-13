# Try to find the FREEGLUT library
#
# FREEGLUT_INCLUDE_DIR
# FREEGLUT_LIBRARY
# FREEGLUT_FOUND

IF(MSVC)

   FIND_PATH(FREEGLUT_INCLUDE_DIR GL/freeglut.h
      $ENV{PROGRAMFILES}/Freeglut/include
      DOC "The directory where GL/freeflut.h resides")

   IF(BUILD_SHARED_LIB OR USE_SHARED_LIB)
      IF(CMAKE_CL_64) 
         FIND_LIBRARY(FREEGLUT_LIBRARY
            NAMES freeglut
            PATHS
            $ENV{ProgramW6432}/Freeglut/lib
            DOC "The FREEGLUT 64-bit library")
      ELSE(CMAKE_CL_64)
         FIND_LIBRARY( FREEGLUT_LIBRARY
            NAMES freeglut
            PATHS
            $ENV{PROGRAMFILES}/Freeglut/lib
            DOC "The FREEGLUT 32-bit library")
      ENDIF(CMAKE_CL_64)
   ELSE(BUILD_SHARED_LIB OR USE_SHARED_LIB)
      IF(CMAKE_CL_64) 
         FIND_LIBRARY( FREEGLUT_LIBRARY
            NAMES freeglut_static
            PATHS
            $ENV{ProgramW6432}/Freeglut/lib
            DOC "The FREEGLUT 64-bit library")
      ELSE(CMAKE_CL_64)
         FIND_LIBRARY( FREEGLUT_LIBRARY
            NAMES freeglut_static
            PATHS
            $ENV{PROGRAMFILES}/Freeglut/lib
            DOC "The FREEGLUT 32-bit library")
      ENDIF(CMAKE_CL_64)
   ENDIF(BUILD_SHARED_LIB OR USE_SHARED_LIB)
   
ELSE(MSVC)

   FIND_PATH(
      FREEGLUT_INCLUDE_DIR GL/freeglut.h
      ${CMAKE_INCLUDE_PATH}
      $ENV{include}
      ${OPENGL_INCLUDE_DIR}
      /usr/include
      /usr/local/include)

   FIND_LIBRARY(
      FREEGLUT_LIBRARY
      NAMES freeglut_static freeglut glut
      PATH
      ${CMAKE_LIBRARY_PATH}
      $ENV{lib}
      /usr/lib
      /usr/local/lib)


ENDIF(MSVC)

IF(FREEGLUT_INCLUDE_DIR AND FREEGLUT_LIBRARY)
   SET(FREEGLUT_FOUND TRUE)
   IF(NOT FREEGLUT_FIND_QUIETLY)
      MESSAGE(STATUS "Freeglut Found")
   ENDIF(NOT FREEGLUT_FIND_QUIETLY)
ELSE(FREEGLUT_INCLUDE_DIR AND FREEGLUT_LIBRARY)
   SET(FREEGLUT_FOUND FALSE)
   IF(FREEGLUT_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Cannot Find Freeglut")
   ENDIF(FREEGLUT_FIND_REQUIRED)
ENDIF(FREEGLUT_INCLUDE_DIR AND FREEGLUT_LIBRARY)

