project(hand CXX)

set(SOURCES
    # cmake-format: sortable
    touch_bionics_hand.cpp
    quantum_hand.cpp)

add_library(hand ${SOURCES})

target_link_libraries(hand dac)

