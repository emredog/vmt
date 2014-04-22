#-------------------------------------------------
#
# Project created by QtCreator 2014-03-18T15:45:18
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = ConvertBBoxCandidatesToTrackFile
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp

LIBS += -lopencv_core -lopencv_highgui
