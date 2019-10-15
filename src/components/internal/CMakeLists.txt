project(internal CXX)

add_subdirectory(actuators)
add_subdirectory(adc)
add_subdirectory(dac)
add_subdirectory(gpio)
add_subdirectory(hand)

add_library(internal INTERFACE)

target_link_libraries(internal INTERFACE actuators adc dac gpio hand)
