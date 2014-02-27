#-------------------------------------------------
#
# Project created by QtCreator 2014-02-27T14:58:41
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = Clustering_OpenCV
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp

LIBS += -lopencv_core \
-lopencv_ml \

