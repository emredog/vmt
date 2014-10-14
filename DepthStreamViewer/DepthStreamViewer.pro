#-------------------------------------------------
#
# Project created by QtCreator 2014-10-14T13:48:14
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = DepthStreamViewer
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

LIBS += -lopencv_core -lopencv_highgui

SOURCES += main.cpp
