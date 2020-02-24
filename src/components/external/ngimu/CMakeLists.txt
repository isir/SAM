project(ngimu CXX)

set(SOURCES
    # cmake-format: sortable
    ngimu.cpp)

add_library(ngimu ${SOURCES})

target_link_libraries(ngimu Osc99)
