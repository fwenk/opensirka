cmake_minimum_required(VERSION 2.8)
project( MotionShadowAdapter )

SET( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
SET( SIRKA_EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
IF( NOT SIRKA_EIGEN3_INCLUDE_DIR )
  FIND_PACKAGE(Eigen3)
ELSE()
  INCLUDE_DIRECTORIES ( "${SIRKA_EIGEN3_INCLUDE_DIR}" )
ENDIF()

ADD_DEFINITIONS(
    -Wno-unused-local-typedef
    -std=c++17
)

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

FIND_PACKAGE(Boost REQUIRED COMPONENTS program_options date_time filesystem system thread log)
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

add_executable( shadow main.cpp shadowClient.cpp shadowFile.cpp shadowFormat.cpp )
target_link_libraries( shadow ${Boost_LIBRARIES} )
