# Created by and for Qt Creator. This file was created for editing the project sources only.
# You may attempt to use it for building too, by modifying this file here.

#TARGET = zebraprey_cam

HEADERS = \
   $$PWD/include/util.h \
    include/aux.h \
    include/circular_video_buffer_ts.h \
    src/arduino-serial-lib.h


SOURCES = \
   $$PWD/src/util.cpp \
   $$PWD/src/main.cpp \
    src/arduino-serial-lib.c \
    src/aux.cpp


INCLUDEPATH = \
    $$PWD/include \
    /usr/include/flycapture
INCLUDEPATH += /usr/local//include/opencv4/
INCLUDEPATH += `pkg-config --cflags opencv`


LIBS = -L/usr/local/lib -lflycapture -lpthread -lboost_thread -lboost_system
LIBS += `pkg-config --libs opencv`


#DEFINES = 

DISTFILES += \
    scripts/install_opencv.sh \
    scripts/makevids.sh

