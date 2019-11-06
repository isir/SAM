project(control CXX)

set(SOURCES
    # cmake-format: sortable
    algo/lawimu.cpp
    algo/lawjacobian.cpp
    algo/lawopti.cpp
    algo/myocontrol.cpp
    compensation_imu.cpp
    compensation_optitrack.cpp
    demo.cpp
    general_formulation.cpp
    matlab_receiver.cpp
    read_adc.cpp
    remote_computer_control.cpp
    voluntary_control.cpp)

add_library(control ${SOURCES})

target_link_libraries(control stdc++fs)
