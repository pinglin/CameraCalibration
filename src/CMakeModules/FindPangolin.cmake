#
# Try to find Pangolin library and include path.
# Once done this will define
#
# PANGOLIN_FOUND
# PANGOLIN_INCLUDE_DIR
# PANGOLIN_LIBRARY
#

IF(MSVC)
        FIND_PATH(PANGOLIN_INCLUDE_DIR pangolin/pangolin.h 
                                       ../../Libraries/Pangolin/include)
        FIND_LIBRARY(PANGOLIN_LIB_RELEASE 
                     NAMES pangolin 
                     PATHS ../../Libraries/Pangolin/lib/Release)
        FIND_LIBRARY(PANGOLIN_LIB_DEBUG 
                     NAMES pangolind 
                     PATHS ../../Libraries/Pangolin/lib/Debug)
ELSE(MSVC)

        FIND_PATH(Pangolin_INCLUDE_DIR pangolin/pangolin.h 
                                       ../../Pangolin/include)
        FIND_LIBRARY(Pangolin_LIBRARY pangolin 
                                      ../../Pangolin/lib)


ENDIF(MSVC)

IF(PANGOLIN_INCLUDE_DIR AND PANGOLIN_LIB_RELEASE AND PANGOLIN_LIB_DEBUG)
   SET(PANGOLIN_FOUND TRUE)
   SET(PANGOLIN_LIBRARY optimized ${PANGOLIN_LIB_RELEASE} 
                        debug ${PANGOLIN_LIB_DEBUG}  
                        CACHE STRING
                        "Pangolin debug and release libraries"
                        FORCE)
   MESSAGE(STATUS "Pangolin Found")      
ELSEIF(PANGOLIN_INCLUDE_DIR AND PANGOLIN_LIB_RELEASE AND PANGOLIN_LIB_DEBUG)
   MESSAGE(FATAL_ERROR "Cannot Find Pangolin")   
ENDIF(PANGOLIN_INCLUDE_DIR AND PANGOLIN_LIB_RELEASE AND PANGOLIN_LIB_DEBUG)
