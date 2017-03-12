ADD_DEFINITIONS(
  -std=c++11
  -fpermissive
)

file(GLOB jsmviz_src *.cpp *.h)

find_package(OpenSceneGraph REQUIRED osgDB osgUtil osgViewer osgText osgGA)

include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})

ADD_LIBRARY( jsmviz STATIC ${jsmviz_src})
target_link_libraries(jsmviz ${OPENSCENEGRAPH_LIBRARIES})
