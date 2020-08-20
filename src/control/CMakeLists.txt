project(control CXX)

set(SOURCES
    # cmake-format: sortable
    algo/lawimu_wrist.cpp
    algo/lawjacobian.cpp
    algo/lawopti.cpp
    algo/lawimu_we.cpp
    algo/myocontrol.cpp
    compensation_imu.cpp
    compensation_optitrack.cpp
    cybathlon.cpp
    demo.cpp
    demo_imu.cpp
    jf_opti.cpp
    jfimu_sk.cpp
    jfimu_v3.cpp
    jfimu_v4.cpp
    matlab_receiver.cpp
    read_adc.cpp
    remote_computer_control.cpp
    recorddata.cpp
    test_imu.cpp)
    voluntary_control.cpp
    pushbuttons.cpp
    myo_2electrodes.cpp)

add_library(control ${SOURCES})

target_link_libraries(control stdc++fs)
