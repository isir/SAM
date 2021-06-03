project(touchsensor CXX)

set(SOURCES
    # cmake-format: sortable
    at42qt1070.cpp)

add_library(touchsensor ${SOURCES})

target_link_libraries(touchsensor i2c)

