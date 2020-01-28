project(Osc99 C)

set(SOURCES
    # cmake-format: sortable
    OscAddress.c 
    OscBundle.c
    OscCommon.c
    OscError.c
    OscMessage.c
    OscPacket.c
    OscSlip.c)

add_library(Osc99 ${SOURCES})
