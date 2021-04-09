project(epos CXX)

set(SOURCES
    # cmake-format: sortable
    epos.cpp)

add_library(epos ${SOURCES})

#target_link_libraries(epos PUBLIC
#    /home/charlotte/EPOS_Linux_Library/lib/arm/v7/libEposCmd.so.6.6.2.0
#    /home/charlotte/EPOS_Linux_Library/lib/arm/v7/libftd2xx.so.1.4.8
#)
