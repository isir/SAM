project(dac CXX)

set(SOURCES
    # cmake-format: sortable
    mcp4728.cpp)

add_library(dac ${SOURCES})
