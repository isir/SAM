project(adc CXX)

set(SOURCES
    # cmake-format: sortable
    adafruit_ads1115.cpp)

add_library(adc ${SOURCES})

target_link_libraries(adc i2c)
