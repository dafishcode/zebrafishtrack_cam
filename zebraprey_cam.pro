# Created by and for Qt Creator. This file was created for editing the project sources only.
# You may attempt to use it for building too, by modifying this file here.

#TARGET = zebraprey_cam

HEADERS = \
   $$PWD/include/util.h


SOURCES = \
   $$PWD/src/util.cpp \
   $$PWD/src/Rec_onDisk.cpp



INCLUDEPATH = \
    $$PWD/include \
    /usr/include/flycapture


LIBS = `pkg-config --libs opencv`
LIBS += -L/usr/local/lib -lflycapture -lpthread
#DEFINES = 

