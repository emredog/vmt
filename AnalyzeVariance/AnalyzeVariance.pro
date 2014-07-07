#-------------------------------------------------
#
# Project created by QtCreator 2014-07-04T11:08:03
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = AnalyzeVariance
CONFIG   += console
CONFIG   += debug
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    ../AnalyzeAnnotations/boundingbox.cpp

HEADERS += \
    helperFunctions.h \
    ../AnalyzeAnnotations/boundingbox.h

INCLUDEPATH += "../AnalyzeAnnotations"


LIBS += -lopencv_core \
-lopencv_imgproc \
-lopencv_highgui
