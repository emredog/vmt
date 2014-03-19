#-------------------------------------------------
#
# Project created by QtCreator 2014-03-19T19:46:15
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = DisplayTrackRectangles
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp

LIBS += -lopencv_core -lopencv_highgui
