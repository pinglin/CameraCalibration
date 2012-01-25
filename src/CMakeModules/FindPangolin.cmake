IF(UNIX)
        FIND_PATH(Pangolin_INCLUDE_DIR pangolin/pangolin.h ../../Pangolin)
        FIND_LIBRARY(Pangolin_LIBRARY pangolin ../../Pangolin/pangolin)
ENDIF(UNIX)

#IF(WIN32)
#        FIND_PATH(NLOPT_INCLUDE_DIR nlopt.h "C:\\Users\\Pinglin\\Documents\\C Workspace\\nlopt")
#        FIND_LIBRARY(NLOPT_LIBRARY nlopt-0 "C:\\Users\\Pinglin\\Documents\\C Workspace\\nlopt")
#ENDIF(WIN32)

IF(Pangolin_INCLUDE_DIR AND Pangolin_LIBRARY)
  SET( Pangolin_FOUND TRUE )
ENDIF(Pangolin_INCLUDE_DIR AND Pangolin_LIBRARY)
