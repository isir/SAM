project(sam CXX)

set(SOURCES
    # cmake-format: sortable
    main.cpp
    sam/sam.cpp
    sam/samanager.cpp
    sam/system_monitor.cpp)

include_directories(. /opt/vc/include)
link_directories(/opt/vc/lib)

add_subdirectory(components)
add_subdirectory(control)
add_subdirectory(ui)
add_subdirectory(utils)
add_subdirectory(ux)

add_executable(sam ${SOURCES})

target_link_libraries(sam
                      components
                      control
                      ui
                      utils
                      ux
                      bcm2835)

install(TARGETS sam RUNTIME DESTINATION /opt/)
