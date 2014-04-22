#-------------------------------------------------
#
# Project created by QtCreator 2014-04-22T15:08:20
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = ConvertAnnotationToSlidingWindow
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
