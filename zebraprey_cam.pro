# Created by and for Qt Creator. This file was created for editing the project sources only.
# You may attempt to use it for building too, by modifying this file here.

#TARGET = zebraprey_cam

HEADERS = \
   $$PWD/include/util.h \
    include/aux.h \
    include/circular_buffer_ts.h \
    include/ellipse_detect.h \
    include/template_detect.h \
    src/arduino-serial-lib.h


SOURCES = \
   $$PWD/src/util.cpp \
   $$PWD/src/Rec_onDisk.cpp \
    src/arduino-serial-lib.c \
    src/aux.cpp \
    src/ellipse_detect.cpp \
    src/template_detect.cpp



INCLUDEPATH = \
    $$PWD/include \
    /usr/include/flycapture


LIBS = `pkg-config --libs opencv`
LIBS += -L/usr/local/lib -lflycapture -lpthread -lboost_thread -lboost_system
#DEFINES = 

DISTFILES += \
    img/fishbody_tmp1.pgm \
    img/fishbody_tmp2.pgm \
    img/fishbody_tmp3.pgm \
    img/fishbody_tmp4.pgm \
    img/fishbody_tmp5.pgm \
    img/fishbody_tmp6.pgm \
    img/fishbody_tmp7.pgm \
    img/fishbody_tmp8.pgm \
    img/fishbody_tmp9.pgm

RESOURCES += \
    qml.qrc

