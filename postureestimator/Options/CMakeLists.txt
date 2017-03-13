# This library needs to have boost already configured.

ADD_DEFINITIONS(
    -std=c++11
)

file(GLOB options_src *.cpp *.h)
ADD_LIBRARY( options STATIC ${options_src})
target_link_libraries(options ${Boost_LIBRARIES})
INSTALL(TARGETS options DESTINATION lib EXPORT sirkasensorfusion-targets)
