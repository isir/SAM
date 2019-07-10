QT -= gui
QT += network mqtt

CONFIG += console
CONFIG -= app_bundle

DEFINES += QT_DEPRECATED_WARNINGS

HEADERS += \
    src/control/algorithms/lawopti.h \
    src/control/algorithms/myocontrol.h \
    src/control/compensation_optitrack.h \
    src/control/demo.h \
    src/control/matlab_receiver.h \
    src/control/remote_computer_control.h \
    src/control/threaded_loop.h \
    src/control/voluntary_control.h \
    src/peripherals/actuators/actuator.h \
    src/peripherals/actuators/custom_elbow.h \
    src/peripherals/actuators/osmer_elbow.h \
    src/peripherals/actuators/pronosupination.h \
    src/peripherals/actuators/shoulder_rotator.h \
    src/peripherals/actuators/wrist_flexor.h \
    src/peripherals/actuators/wrist_rotator.h \
    src/peripherals/adafruit_ads1115.h \
    src/peripherals/buzzer.h \
    src/peripherals/ledstrip.h \
    src/peripherals/mcp4728.h \
    src/peripherals/myoband/myoLinux/bleapi.h \
    src/peripherals/myoband/myoLinux/bled112client.h \
    src/peripherals/myoband/myoLinux/buffer.h \
    src/peripherals/myoband/myoLinux/firstargument.h \
    src/peripherals/myoband/myoLinux/gattclient.h \
    src/peripherals/myoband/myoLinux/myoapi.h \
    src/peripherals/myoband/myoLinux/myoapi_p.h \
    src/peripherals/myoband/myoLinux/myoclient.h \
    src/peripherals/myoband/myoLinux/myolinux.h \
    src/peripherals/myoband/myoLinux/serial.h \
    src/peripherals/myoband/myoband.h \
    src/peripherals/roboclaw/cast_helper.h \
    src/peripherals/roboclaw/factory.h \
    src/peripherals/roboclaw/message.h \
    src/peripherals/roboclaw/roboclaw.h \
    src/peripherals/roboclaw/types.h \
    src/peripherals/touch_bionics/touch_bionics_hand.h \
    src/peripherals/XIMU.h \
    src/ui/console_input.h \
    src/ui/menu_backend.h \
    src/ui/menu_broker.h \
    src/ui/menu_console.h \
    src/ui/menu_frontend.h \
    src/ui/menu_item.h \
    src/ui/menu_mqtt.h \
    src/ui/menu_user.h \
    src/ui/mqtt_user.h \
    src/utils/check_ptr.h \
    src/utils/logger.h \
    src/utils/named_object.h \
    src/utils/opti_listener.h \
    src/utils/sam.h \
    src/utils/samanager.h \
    src/utils/serial_port.h \
    src/utils/settings.h \
    src/utils/system_monitor.h

SOURCES += \
    src/control/threaded_loop.cpp \
    src/main.cpp \
    src/control/algorithms/lawopti.cpp \
    src/control/algorithms/myocontrol.cpp \
    src/control/compensation_optitrack.cpp \
    src/control/demo.cpp \
    src/control/matlab_receiver.cpp \
    src/control/remote_computer_control.cpp \
    src/control/voluntary_control.cpp \
    src/peripherals/actuators/actuator.cpp \
    src/peripherals/actuators/custom_elbow.cpp \
    src/peripherals/actuators/osmer_elbow.cpp \
    src/peripherals/actuators/pronosupination.cpp \
    src/peripherals/actuators/shoulder_rotator.cpp \
    src/peripherals/actuators/wrist_flexor.cpp \
    src/peripherals/actuators/wrist_rotator.cpp \
    src/peripherals/adafruit_ads1115.cpp \
    src/peripherals/buzzer.cpp \
    src/peripherals/ledstrip.cpp \
    src/peripherals/mcp4728.cpp \
    src/peripherals/myoband/myoLinux/gattclient.cpp \
    src/peripherals/myoband/myoLinux/myoclient.cpp \
    src/peripherals/myoband/myoLinux/serial.cpp \
    src/peripherals/myoband/myoband.cpp \
    src/peripherals/roboclaw/factory.cpp \
    src/peripherals/roboclaw/message.cpp \
    src/peripherals/roboclaw/roboclaw.cpp \
    src/peripherals/touch_bionics/touch_bionics_hand.cpp \
    src/peripherals/XIMU.cpp \
    src/ui/console_input.cpp \
    src/ui/menu_backend.cpp \
    src/ui/menu_broker.cpp \
    src/ui/menu_console.cpp \
    src/ui/menu_frontend.cpp \
    src/ui/menu_mqtt.cpp \
    src/ui/menu_user.cpp \
    src/ui/mqtt_user.cpp \
    src/utils/logger.cpp \
    src/utils/named_object.cpp \
    src/utils/opti_listener.cpp \
    src/utils/sam.cpp \
    src/utils/samanager.cpp \
    src/utils/serial_port.cpp \
    src/utils/settings.cpp \
    src/utils/system_monitor.cpp

INCLUDEPATH += src/

QMAKE_CXXFLAGS += -O2 -mfloat-abi=hard

LIBS += -lwiringPi -li2c -lboost_thread -lboost_system

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
