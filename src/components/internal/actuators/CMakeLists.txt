project(actuators CXX)

set(SOURCES
    # cmake-format: sortable
    actuator.cpp
    custom_elbow.cpp
    osmer_elbow.cpp
    pronosupination.cpp
    shoulder_rotator.cpp
    wrist_flexor.cpp
    wrist_rotator.cpp
    elbow_cybathlon.cpp)

add_subdirectory(roboclaw)

add_library(actuators ${SOURCES})

target_link_libraries(actuators roboclaw)
