#-------------------------------------------------
#
# Project created by QtCreator 2014-07-14T10:55:28
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = test-CropBox
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp

INCLUDEPATH += /usr/include/pcl-1.7
INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/include/vtk

LIBS += -L/usr/lib64/vtk/ \
-lvtkCommonCore -lvtkRenderingCore -lvtkRenderingAnnotation \
-lvtkRenderingLOD -lvtkCommonDataModel -lvtkCommonMath

LIBS += -lpcl_common -lpcl_visualization -lpcl_filters -lpcl_io -lpcl_search -lpcl_features \
-lboost_system -lboost_thread

HEADERS +=
