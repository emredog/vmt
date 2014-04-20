#-------------------------------------------------
#
# Project created by QtCreator 2014-04-20T18:56:24
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = ConvertAnnotationToTrackFile
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
