project(utils CXX)

set(SOURCES
    # cmake-format: sortable
    interfaces/menu_user.cpp
    interfaces/mqtt_user.cpp
    log/logger.cpp
    log/safe_stream.cpp
    monitoring/abstract_monitor.cpp
    monitoring/cpu_freq_monitor.cpp
    monitoring/cpu_load_monitor.cpp
    monitoring/cpu_temp_monitor.cpp
    monitoring/vc_based_monitor.cpp
    named_object.cpp
    param.cpp
    serial_port.cpp
    socket.cpp
    threaded_loop.cpp
    watchdog.cpp
    worker.cpp)

add_library(utils ${SOURCES})

target_link_libraries(utils
                      pthread
                      vcos
                      vchiq_arm
                      vchostif)
