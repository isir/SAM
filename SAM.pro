QT -= gui
QT += network serialport

CONFIG += c++17 console
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
        src/control/basiccontroller.h \
        src/control/compensationoptitrack.h \
        src/control/demo.h \
        src/control/voluntarycontrol.h \
        src/peripherals/adafruit_ads1115.h \
        src/peripherals/buzzer.h \
        src/peripherals/mcp4728.h \
        src/peripherals/XIMU.h \
        src/peripherals/helpers/osmerelbow.h \
        src/peripherals/helpers/pronosupination.h \
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
        src/peripherals/roboclaw/casthelper.h \
        src/peripherals/roboclaw/client.h \
        src/peripherals/roboclaw/factory.h \
        src/peripherals/roboclaw/message.h \
        src/peripherals/roboclaw/server.h \
        src/peripherals/roboclaw/types.h \
        src/peripherals/touch_bionics/touch_bionics_hand.h \
        src/ui/consoleinput.h \
        src/ui/consolemenu.h \
        src/ui/consolemenuitem.h \
        src/utils/optilistener.h

SOURCES += \
        src/control/algorithms/lawopti.cpp \
        src/control/algorithms/myocontrol.cpp \
        src/control/basiccontroller.cpp \
        src/control/compensationoptitrack.cpp \
        src/control/demo.cpp \
        src/control/voluntarycontrol.cpp \
        src/main.cpp \
        src/peripherals/adafruit_ads1115.cpp \
        src/peripherals/buzzer.cpp \
        src/peripherals/helpers/osmerelbow.cpp \
        src/peripherals/mcp4728.cpp \
        src/peripherals/XIMU.cpp \
        src/peripherals/helpers/pronosupination.cpp \
        src/peripherals/myoband/myoLinux/gattclient.cpp \
        src/peripherals/myoband/myoLinux/myoclient.cpp \
        src/peripherals/myoband/myoLinux/serial.cpp \
        src/peripherals/myoband/myoband.cpp \
        src/peripherals/roboclaw/client.cpp \
        src/peripherals/roboclaw/factory.cpp \
        src/peripherals/roboclaw/message.cpp \
        src/peripherals/roboclaw/server.cpp \
        src/peripherals/touch_bionics/touch_bionics_hand.cpp \
        src/ui/consoleinput.cpp \
        src/ui/consolemenu.cpp \
        src/utils/optilistener.cpp

INCLUDEPATH += src/

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

LIBS += -lwiringPi -li2c -lboost_thread -lboost_system
