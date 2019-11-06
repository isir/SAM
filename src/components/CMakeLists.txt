project(components CXX)

add_subdirectory(external)
add_subdirectory(internal)

add_library(components INTERFACE)

target_link_libraries(components INTERFACE external internal)
