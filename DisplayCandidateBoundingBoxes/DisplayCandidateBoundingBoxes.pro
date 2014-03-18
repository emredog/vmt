#-------------------------------------------------
#
# Project created by QtCreator 2014-03-18T12:30:26
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = DisplayCandidateBoundingBoxes
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp

LIBS += -lopencv_core -lopencv_highgui
