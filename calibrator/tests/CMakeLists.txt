FIND_PACKAGE(Boost REQUIRED COMPONENTS unit_test_framework)
SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_STATIC_RUNTIME OFF)
SET(Boost_USE_MULTITHREAD ON)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
ADD_DEFINITIONS(
    -DBOOST_LOG_DYN_LINK
    -DBOOST_TEST_DYN_LINK
    )

if(APPLE)
	SET(LAPACK ${ACCELERATE})
else(APPLE)
	SET(LAPACK ${LAPACK_LIBRARIES})
endif(APPLE)

add_executable(test_sphere_parameterization test_sphere_parameterization.cpp)
target_link_libraries(test_sphere_parameterization ${LAPACK} ${Boost_LIBRARIES} imureading pthread)
add_test(NAME test_sphere_parameterization COMMAND test_sphere_parameterization)

add_executable(test_packed_matrix test_packed_matrix.cpp)
target_link_libraries(test_packed_matrix ${LAPACK} ${Boost_LIBRARIES} imureading pthread)
add_test(NAME test_packed_matrix COMMAND test_packed_matrix)
