ADD_DEFINITIONS(
    -std=c++11
)

file(GLOB orientation_from_motion_src *.cpp *.h)
ADD_LIBRARY( orientation_from_motion STATIC ${orientation_from_motion_src})
target_link_libraries( orientation_from_motion math_helper )
