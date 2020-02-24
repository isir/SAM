project(Osc99 C CXX)

set(SOURCES
    # cmake-format: sortable
    OscAddress.c 
    OscCommon.c
    OscError.c
    OscBundle.cpp
    OscMessage.cpp
    OscPacket.cpp
    OscSlip.cpp)

add_library(Osc99 ${SOURCES})
