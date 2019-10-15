project(myoband CXX)

set(SOURCES
    # cmake-format: sortable
    myoLinux/gattclient.cpp
    myoLinux/myoclient.cpp
    myoLinux/serial.cpp
    myoband.cpp)

add_library(myoband ${SOURCES})

target_link_libraries(myoband utils)
