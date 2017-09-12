# - Find ORB_SLAM2 
#
# It sets the following variables:
#  ORB_SLAM2_FOUND         - Set to false, or undefined, if ORB_SLAM2 isn't found.
#  ORB_SLAM2_INCLUDE_DIRS  - The ORB_SLAM2 include directory.
#  ORB_SLAM2_LIBRARIES     - The ORB_SLAM2 library to link against.
#
#  Set ORB_SLAM2_ROOT_DIR environment variable as the path to ORB_SLAM2 root folder.

find_path(ORB_SLAM2_INCLUDE_DIR NAMES System.h PATHS $ENV{ORB_SLAM2_ROOT_DIR}/include)
find_library(ORB_SLAM2_LIBRARY NAMES ORB_SLAM2 PATHS $ENV{ORB_SLAM2_ROOT_DIR}/lib)
find_library(g2o_LIBRARY NAMES g2o PATHS $ENV{ORB_SLAM2_ROOT_DIR}/Thirdparty/g2o/lib)

IF (ORB_SLAM2_INCLUDE_DIR AND ORB_SLAM2_LIBRARY AND g2o_LIBRARY)
   SET(ORB_SLAM2_FOUND TRUE)
   SET(ORB_SLAM2_INCLUDE_DIRS ${ORB_SLAM2_INCLUDE_DIR} $ENV{ORB_SLAM2_ROOT_DIR})
   SET(ORB_SLAM2_LIBRARIES ${g2o_LIBRARY} ${ORB_SLAM2_LIBRARY})
ENDIF (ORB_SLAM2_INCLUDE_DIR AND ORB_SLAM2_LIBRARY AND g2o_LIBRARY)

IF (ORB_SLAM2_FOUND)
   # show which ORB_SLAM2 was found only if not quiet
   IF (NOT ORB_SLAM2_FIND_QUIETLY)
      MESSAGE(STATUS "Found ORB_SLAM2: ${ORB_SLAM2_LIBRARIES}")
   ENDIF (NOT ORB_SLAM2_FIND_QUIETLY)
ELSE (ORB_SLAM2_FOUND)
   # fatal error if ORB_SLAM2 is required but not found
   IF (ORB_SLAM2_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find ORB_SLAM2")
   ENDIF (ORB_SLAM2_FIND_REQUIRED)
ENDIF (ORB_SLAM2_FOUND)

