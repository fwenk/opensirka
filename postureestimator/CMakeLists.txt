cmake_minimum_required(VERSION 2.8)
project( OpenSirka )

SET( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
SET( SIRKA_EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
IF( NOT SIRKA_EIGEN3_INCLUDE_DIR )
  FIND_PACKAGE(Eigen3)
ELSE()
  INCLUDE_DIRECTORIES ( "${SIRKA_EIGEN3_INCLUDE_DIR}" )
ENDIF()

ADD_DEFINITIONS(
    -Wno-unused-local-typedef
    -fpermissive
    -std=c++17
)

enable_testing()

SET(ENV{PKG_CONFIG_PATH} "/opt/X11/lib/pkgconfig/")
FIND_PACKAGE(PkgConfig)

IF( NEON )
  ADD_DEFINITIONS(
    -mfpu=neon -mfloat-abi=softfp
  )
ENDIF()

IF( SSE )
  ADD_DEFINITIONS(
    -msse2
  )
ENDIF()

include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC GIT_SHA1)
MESSAGE(STATUS "Current git head: " ${GIT_SHA1})

FIND_PACKAGE(Boost REQUIRED COMPONENTS unit_test_framework program_options date_time filesystem signals system thread log)
SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_STATIC_RUNTIME OFF)
SET(Boost_USE_MULTITHREAD ON)
ADD_DEFINITIONS(
    -DBOOST_LOG_DYN_LINK
    -DBOOST_TEST_DYN_LINK
)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR} )
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} )

SET( GIT_HASH_SOURCE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/githash.cpp" )
configure_file("${GIT_HASH_SOURCE_FILE}.in" "${GIT_HASH_SOURCE_FILE}" @ONLY)

add_subdirectory(MathHelper)
add_subdirectory(Options)
add_subdirectory(OrientationFromMotion)
add_subdirectory(jsmviz)
add_subdirectory(Tests)

ADD_DEFINITIONS(
    -std=c++11
)

IF(NOT IMU_READING_PREFIX)
  SET(IMU_READING_PREFIX "/usr/local")
ENDIF(NOT IMU_READING_PREFIX)
MESSAGE(STATUS "IMU reading library prefix: " ${IMU_READING_PREFIX})

INCLUDE(${IMU_READING_PREFIX}/lib/imureading/imureading-targets.cmake)
INCLUDE_DIRECTORIES(${IMU_READING_PREFIX}/include/imureading)

add_executable( M2AccuReplay m2-accu-replay.cpp ${GIT_HASH_SOURCE_FILE})
target_link_libraries( M2AccuReplay imureading orientation_from_motion jsmviz ${Boost_LIBRARIES} options )

ADD_DEFINITIONS(
    -std=c++11
)
# Export stuff useful in other projects
INSTALL(DIRECTORY Options/ DESTINATION include/sirkasensorfusion FILES_MATCHING PATTERN *.h)
INSTALL(DIRECTORY nisprint/ DESTINATION include/sirkasensorfusion FILES_MATCHING PATTERN *.h)
INSTALL(EXPORT sirkasensorfusion-targets DESTINATION lib/sirkasensorfusion)
