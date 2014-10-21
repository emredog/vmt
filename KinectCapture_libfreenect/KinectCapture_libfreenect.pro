#-------------------------------------------------
#
# Project created by QtCreator 2014-10-08T10:23:03
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = KinectCapture_libfreenect
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

INCLUDEPATH += /usr/include/libusb-1.0

LIBS += -lopencv_core -lopencv_highgui -lfreenect -lopencv_imgproc


SOURCES += main.cpp
