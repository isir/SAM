project(SAM CXX)
cmake_minimum_required(VERSION 3.0)

set(SOURCES
    # cmake-format: sortable
    src/components/external/myoband/myoLinux/gattclient.cpp
    src/components/external/myoband/myoLinux/myoclient.cpp
    src/components/external/myoband/myoLinux/serial.cpp
    src/components/external/myoband/myoband.cpp
    src/components/external/optitrack_listener.cpp
    src/components/external/ximu.cpp
    src/components/internal/actuators/actuator.cpp
    src/components/internal/actuators/custom_elbow.cpp
    src/components/internal/actuators/osmer_elbow.cpp
    src/components/internal/actuators/pronosupination.cpp
    src/components/internal/actuators/shoulder_rotator.cpp
    src/components/internal/actuators/wrist_flexor.cpp
    src/components/internal/actuators/wrist_rotator.cpp
    src/components/internal/adafruit_ads1115.cpp
    src/components/internal/mcp4728.cpp
    src/components/internal/roboclaw/answer.cpp
    src/components/internal/roboclaw/factory.cpp
    src/components/internal/roboclaw/message.cpp
    src/components/internal/roboclaw/roboclaw.cpp
    src/components/internal/touch_bionics/touch_bionics_hand.cpp
    src/control/algo/lawimu.cpp
    src/control/algo/lawjacobian.cpp
    src/control/algo/lawopti.cpp
    src/control/algo/myocontrol.cpp
    src/control/compensation_imu.cpp
    src/control/compensation_optitrack.cpp
    src/control/demo.cpp
    src/control/general_formulation.cpp
    src/control/matlab_receiver.cpp
    src/control/remote_computer_control.cpp
    src/control/voluntary_control.cpp
    src/main.cpp
    src/sam/sam.cpp
    src/sam/samanager.cpp
    src/sam/system_monitor.cpp
    src/ui/menu/menu_console.cpp
    src/ui/menu/menu_frontend.cpp
    src/ui/menu/menu_mqtt.cpp
    src/ui/sound/buzzer.cpp
    src/ui/visual/ledstrip.cpp
    src/utils/interfaces/menu_user.cpp
    src/utils/interfaces/mqtt_user.cpp
    src/utils/log/logger.cpp
    src/utils/log/safe_stream.cpp
    src/utils/monitoring/abstract_monitor.cpp
    src/utils/monitoring/cpu_freq_monitor.cpp
    src/utils/monitoring/cpu_load_monitor.cpp
    src/utils/monitoring/cpu_temp_monitor.cpp
    src/utils/monitoring/vc_based_monitor.cpp
    src/utils/named_object.cpp
    src/utils/param.cpp
    src/utils/serial_port.cpp
    src/utils/socket.cpp
    src/utils/threaded_loop.cpp
    src/utils/watchdog.cpp
    src/utils/worker.cpp
    src/ux/menu/menu_backend.cpp
    src/ux/menu/menu_broker.cpp
    src/ux/menu/menu_item.cpp
    src/ux/mosquittopp/client.cpp
    src/ux/mosquittopp/connect_factory.cpp
    src/ux/mosquittopp/connect_helper.cpp
    src/ux/mosquittopp/message.cpp
    src/ux/mosquittopp/subscription.cpp
    src/ux/mosquittopp/subscription_factory.cpp)

include_directories(src/)

add_definitions(-DMOSQUITTO_SERVER_IP="127.0.0.1" -DMOSQUITTO_SERVER_PORT=1883)
add_definitions(-DDEFAULT_CPU_CORE=2 -DDEFAULT_THREAD_PRIO=20)

set(CC_FLAGS "-O2 -mcpu=cortex-a53 -mfloat-abi=hard -std=gnu++17")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CC_FLAGS}")

add_executable(sam ${SOURCES})

target_link_libraries(sam
                      wiringPi
                      i2c
                      espeak
                      mosquitto
                      pthread
                      stdc++fs
                      vcos
                      vchiq_arm
                      vchostif)

install(TARGETS sam RUNTIME DESTINATION /home/root/)
