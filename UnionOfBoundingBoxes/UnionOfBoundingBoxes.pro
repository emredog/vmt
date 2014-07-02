#-------------------------------------------------
#
# Project created by QtCreator 2014-07-02T11:53:00
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = UnionOfBoundingBoxes
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

INCLUDEPATH += "../AnalyzeAnnotations/"


SOURCES += main.cpp \
    ../AnalyzeAnnotations/boundingbox.cpp \
    ../AnalyzeAnnotations/action.cpp

HEADERS += \
    ../AnalyzeAnnotations/boundingbox.h \
    ../AnalyzeAnnotations/action.h
