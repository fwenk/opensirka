CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
project( IMUReading )

enable_testing()

# SET(CMAKE_CXX_COMPILER clang++)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

# FIND_PACKAGE(CXSparse REQUIRED)
FIND_PACKAGE(Eigen3 3.3.0 REQUIRED)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR} )
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} )

MESSAGE(STATUS ${EIGEN3_VERSION})

ADD_DEFINITIONS(
    -std=c++11
)

file(GLOB imureading_src src/*.cpp src/*.h)
ADD_LIBRARY( imureading STATIC ${imureading_src})
INSTALL(TARGETS imureading DESTINATION lib EXPORT imureading-targets)
INSTALL(DIRECTORY src/ DESTINATION include/imureading FILES_MATCHING PATTERN *.h)
INSTALL(EXPORT imureading-targets DESTINATION lib/imureading)

add_subdirectory(tests)
