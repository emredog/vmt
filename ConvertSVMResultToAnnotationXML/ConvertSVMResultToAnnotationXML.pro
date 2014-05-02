#-------------------------------------------------
#
# Project created by QtCreator 2014-05-02T14:47:11
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = ConvertSVMResultToAnnotationXML
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    ../AnalyzeAnnotations/action.cpp \
    ../AnalyzeAnnotations/boundingbox.cpp \
    ../AnalyzeAnnotations/video.cpp

HEADERS += \
    ../AnalyzeAnnotations/action.h \
    ../AnalyzeAnnotations/boundingbox.h \
    ../AnalyzeAnnotations/video.h
