ADD_DEFINITIONS(
    -std=c++11
)

file(GLOB math_helper_src *.cpp *.h)
ADD_LIBRARY( math_helper STATIC ${math_helper_src})
