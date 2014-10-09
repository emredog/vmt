#-------------------------------------------------
#
# Project created by QtCreator 2014-10-08T11:48:31
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = DepthStreamViewer
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

LIBS += -lopencv_highgui -lopencv_core


SOURCES += main.cpp
