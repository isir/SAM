QT -= gui
QT += network serialport mqtt

CONFIG += console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
        src/control/algorithms/lawopti.h \
        src/control/algorithms/myocontrol.h \
    src/control/basic_controller.h \
    src/control/compensation_optitrack.h \
        src/control/demo.h \
    src/control/matlab_receiver.h \
    src/control/remote_computer_control.h \
    src/control/voluntary_control.h \
    src/peripherals/actuators/actuator.h \
        src/peripherals/adafruit_ads1115.h \
        src/peripherals/buzzer.h \
    src/peripherals/actuators/custom_elbow.h \
    src/peripherals/actuators/osmer_elbow.h \
    src/peripherals/actuators/wrist_flexor.h \
    src/peripherals/actuators/wrist_rotator.h \
        src/peripherals/ledstrip.h \
        src/peripherals/mcp4728.h \
        src/peripherals/XIMU.h \
        src/peripherals/actuators/pronosupination.h \
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
    src/ui/console_input.h \
    src/ui/console_menu.h \
    src/ui/console_menu_item.h \
    src/utils/logger.h \
    src/utils/opti_listener.h \
    src/utils/sam.h \
    src/utils/samanager.h \
    src/utils/serial_port.h \
        src/utils/settings.h \
    src/utils/system_monitor.h \
    src/peripherals/actuators/shoulder_rotator.h

SOURCES += \
        src/control/algorithms/lawopti.cpp \
        src/control/algorithms/myocontrol.cpp \
    src/control/basic_controller.cpp \
    src/control/compensation_optitrack.cpp \
        src/control/demo.cpp \
    src/control/matlab_receiver.cpp \
    src/control/remote_computer_control.cpp \
    src/control/voluntary_control.cpp \
        src/main.cpp \
    src/peripherals/actuators/actuator.cpp \
        src/peripherals/adafruit_ads1115.cpp \
        src/peripherals/buzzer.cpp \
    src/peripherals/actuators/custom_elbow.cpp \
    src/peripherals/actuators/osmer_elbow.cpp \
    src/peripherals/actuators/wrist_flexor.cpp \
    src/peripherals/actuators/wrist_rotator.cpp \
        src/peripherals/ledstrip.cpp \
        src/peripherals/mcp4728.cpp \
        src/peripherals/XIMU.cpp \
        src/peripherals/actuators/pronosupination.cpp \
        src/peripherals/myoband/myoLinux/gattclient.cpp \
        src/peripherals/myoband/myoLinux/myoclient.cpp \
        src/peripherals/myoband/myoLinux/serial.cpp \
        src/peripherals/myoband/myoband.cpp \
        src/peripherals/roboclaw/factory.cpp \
        src/peripherals/roboclaw/message.cpp \
    src/peripherals/roboclaw/roboclaw.cpp \
        src/peripherals/touch_bionics/touch_bionics_hand.cpp \
    src/ui/console_input.cpp \
    src/ui/console_menu.cpp \
    src/utils/logger.cpp \
    src/utils/opti_listener.cpp \
    src/utils/samanager.cpp \
    src/utils/serial_port.cpp \
        src/utils/settings.cpp \
    src/utils/system_monitor.cpp \
    src/peripherals/actuators/shoulder_rotator.cpp

INCLUDEPATH += src/

QMAKE_CXXFLAGS += -O2

LIBS += -lwiringPi -li2c -lboost_thread -lboost_system

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
