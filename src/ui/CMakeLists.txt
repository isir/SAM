project(ui CXX)

set(SOURCES
    # cmake-format: sortable
    menu/menu_console.cpp
    menu/menu_frontend.cpp
    menu/menu_mqtt.cpp
    sound/buzzer.cpp
    visual/ledstrip.cpp)

add_library(ui ${SOURCES})

target_link_libraries(ui espeak)
