project(ux CXX)

set(SOURCES
    # cmake-format: sortable
    menu/menu_backend.cpp menu/menu_broker.cpp menu/menu_item.cpp)

add_subdirectory(mosquittopp)
add_subdirectory(Osc99)

add_library(ux ${SOURCES})

target_link_libraries(ux mosquittopp)
