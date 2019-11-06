project(external CXX)

add_subdirectory(myoband)
add_subdirectory(optitrack)
add_subdirectory(ximu)

add_library(external INTERFACE)

target_link_libraries(external INTERFACE myoband optitrack ximu)
