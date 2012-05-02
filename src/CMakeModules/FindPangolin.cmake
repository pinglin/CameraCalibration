IF(UNIX)
        FIND_PATH(Pangolin_INCLUDE_DIR pangolin/pangolin.h ../../Pangolin)
        FIND_LIBRARY(Pangolin_LIBRARY pangolin ../../Pangolin/pangolin)
ENDIF(UNIX)

IF(MSVC)
        FIND_PATH(Pangolin_INCLUDE_DIR pangolin/pangolin.h ../../Libraries/Pangolin)
        FIND_LIBRARY(Pangolin_LIB_RELEASE 
                     NAMES pangolin 
                     PATHS ../../Libraries/Pangolin/pangolin)
        FIND_LIBRARY(Pangolin_LIB_DEBUG 
                     NAMES pangolind 
                     PATHS ../../Libraries/Pangolin/pangolin)
ENDIF(MSVC)

IF(Pangolin_INCLUDE_DIR AND Pangolin_LIB_RELEASE AND Pangolin_LIB_DEBUG)
  SET(Pangolin_FOUND TRUE)
  SET(Pangolin_LIBRARY optimized ${Pangolin_LIB_RELEASE} debug ${Pangolin_LIB_DEBUG}  
      CACHE STRING
      "Pangolin debug and release libraries"
      FORCE)
ENDIF(Pangolin_INCLUDE_DIR AND Pangolin_LIB_RELEASE AND Pangolin_LIB_DEBUG)
