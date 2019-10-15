project(roboclaw CXX)

set(SOURCES
    # cmake-format: sortable
    answer.cpp
    factory.cpp
    message.cpp
    roboclaw.cpp)

add_library(roboclaw ${SOURCES})
