project(optitrack CXX)

set(SOURCES
    # cmake-format: sortable
    optitrack_listener.cpp)

add_library(optitrack ${SOURCES})
