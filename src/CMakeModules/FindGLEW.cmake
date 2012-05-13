#
# Try to find GLEW library and include path.
# Once done this will define
#
# GLEW_FOUND
# GLEW_INCLUDE_DIR
# GLEW_LIBRARY
# 

IF(MSVC)

   IF(CMAKE_CL_64)

      FIND_PATH(GLEW_INCLUDE_DIR GL/glew.h
		$ENV{PROGRAMW6432}/GLEW/include
		DOC "The directory where GL/glew.h resides")
      FIND_LIBRARY(GLEW_LIBRARY
		NAMES glew GLEW glew32 glew32s
		PATHS
		$ENV{PROGRAMW6432}/GLEW/lib
		DOC "The GLEW library")

   ELSE(CMAKE_CL_64)
	FIND_PATH( GLEW_INCLUDE_DIR GL/alew.h
		$ENV{PROGRAMFILES}/GLEW/include
		DOC "The directory where GL/glew.h resides")
	FIND_LIBRARY( GLEW_LIBRARY
		NAMES glew GLEW glew32 glew32s
		PATHS
		$ENV{PROGRAMFILES}/GLEW/lib
		DOC "The GLEW library")
   ENDIF(CMAKE_CL_64)

ELSE(MSVC)
	FIND_PATH( GLEW_INCLUDE_DIR GL/glew.h
		/usr/include
		/usr/local/include
		/sw/include
		/opt/local/include
		DOC "The directory where GL/glew.h resides")
	FIND_LIBRARY( GLEW_LIBRARY
		NAMES GLEW glew
		PATHS
		/usr/lib64
		/usr/lib
		/usr/local/lib64
		/usr/local/lib
		/sw/lib
		/opt/local/lib
		DOC "The GLEW library")
ENDIF(MSVC)

IF(GLEW_INCLUDE_DIR AND GLEW_LIBRARY)
   SET(GLEW_FOUND TRUE)
   IF(NOT GLEW_FIND_QUIETLY)
      MESSAGE(STATUS "GLEW Found")   
   ENDIF(NOT GLEW_FIND_QUIETLY)
ELSE(GLEW_INCLUDE_DIR AND GLEW_LIBRARY)
   SET(GLEW_FOUND FALSE)
   IF(GLEW_FIND_REQUIRED)
   MESSAGE(FATAL_ERROR "Cannot Find GLEW")    
   ENDIF(GLEW_FIND_REQUIRED)
ENDIF(GLEW_INCLUDE_DIR AND GLEW_LIBRARY)

MARK_AS_ADVANCED( GLEW_FOUND )
