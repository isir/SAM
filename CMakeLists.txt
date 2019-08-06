project(SAM CXX)
cmake_minimum_required(VERSION 3.0)

message("arch= ${CMAKE_SYSTEM_PROCESSOR}")

set(SOURCES
    src/control/threaded_loop.cpp
    src/main.cpp
    src/control/algorithms/lawopti.cpp
    src/control/algorithms/myocontrol.cpp
    src/control/compensation_optitrack.cpp
    src/control/demo.cpp
    src/control/matlab_receiver.cpp
    src/control/remote_computer_control.cpp
    src/control/voluntary_control.cpp
    src/peripherals/actuators/actuator.cpp
    src/peripherals/actuators/custom_elbow.cpp
    src/peripherals/actuators/osmer_elbow.cpp
    src/peripherals/actuators/pronosupination.cpp
    src/peripherals/actuators/shoulder_rotator.cpp
    src/peripherals/actuators/wrist_flexor.cpp
    src/peripherals/actuators/wrist_rotator.cpp
    src/peripherals/adafruit_ads1115.cpp
    src/peripherals/buzzer.cpp
    src/peripherals/ledstrip.cpp
    src/peripherals/mcp4728.cpp
    src/peripherals/myoband/myoLinux/gattclient.cpp
    src/peripherals/myoband/myoLinux/myoclient.cpp
    src/peripherals/myoband/myoLinux/serial.cpp
    src/peripherals/myoband/myoband.cpp
    src/peripherals/roboclaw/answer.cpp
    src/peripherals/roboclaw/factory.cpp
    src/peripherals/roboclaw/message.cpp
    src/peripherals/roboclaw/roboclaw.cpp
    src/peripherals/touch_bionics/touch_bionics_hand.cpp
    src/peripherals/XIMU.cpp
    src/ui/menu_backend.cpp
    src/ui/menu_broker.cpp
    src/ui/menu_console.cpp
    src/ui/menu_frontend.cpp
    src/ui/menu_item.cpp
    src/ui/menu_mqtt.cpp
    src/ui/menu_user.cpp
    src/ui/mosquittopp/client.cpp
    src/ui/mosquittopp/connect_helper.cpp
    src/ui/mosquittopp/library_wrapper.cpp
    src/ui/mosquittopp/subscription.cpp
    src/ui/mqtt_user.cpp
    src/utils/log/logger.cpp
    src/utils/log/safe_stream.cpp
    src/utils/named_object.cpp
    src/utils/opti_listener.cpp
    src/utils/param.cpp
    src/utils/sam.cpp
    src/utils/samanager.cpp
    src/utils/serial_port.cpp
    src/utils/socket.cpp
    src/utils/system_monitor.cpp
    src/utils/watchdog.cpp
    )

    include_directories(src/)

    set(CC_FLAGS "-O2 -mcpu=cortex-a53 -mfloat-abi=hard -std=gnu++17")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CC_FLAGS}")

    add_executable(SAM ${SOURCES})

    target_link_libraries(SAM -lwiringPi -li2c -lboost_thread -lboost_system -lespeak -lmosquitto -lpthread -lstdc++fs)
