#-------------------------------------------------
#
# Project created by QtCreator 2014-04-10T17:10:37
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = ConvertBBoxCandidatesToAnnotationFile
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp


LIBS += -lopencv_core -lopencv_highgui
