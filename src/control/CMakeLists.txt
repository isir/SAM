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
    demo_imu.cpp
    controlimu.cpp
    jf_opti.cpp
    jfimu.cpp
    jfimu_v3.cpp
    jfimu_v4.cpp
    matlab_receiver.cpp
    remote_computer_control.cpp
    voluntary_control.cpp)

add_library(control ${SOURCES})

target_link_libraries(control stdc++fs)
