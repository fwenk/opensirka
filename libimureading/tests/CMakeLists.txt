FIND_PACKAGE(Boost REQUIRED COMPONENTS unit_test_framework program_options date_time filesystem signals system thread log)
SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_STATIC_RUNTIME OFF)
SET(Boost_USE_MULTITHREAD ON)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
ADD_DEFINITIONS(
    -DBOOST_LOG_DYN_LINK
    -DBOOST_TEST_DYN_LINK
    )

add_executable(test_loadresolutions test_loadresolutions.cpp)
target_link_libraries(test_loadresolutions ${Boost_LIBRARIES} imureading)
add_test(NAME test_loadresolutions COMMAND test_loadresolutions)

add_executable(test_noisedensities test_noisedensities.cpp)
target_link_libraries(test_noisedensities ${Boost_LIBRARIES} imureading)
add_test(NAME test_noisedensities COMMAND test_noisedensities)

add_executable(test_loadaccumulate test_loadaccumulate.cpp)
target_link_libraries(test_loadaccumulate ${Boost_LIBRARIES} imureading)
add_test(NAME test_loadaccumulate COMMAND test_loadaccumulate "${CMAKE_CURRENT_LIST_DIR}/data/sensor-0-accumulate.log")
