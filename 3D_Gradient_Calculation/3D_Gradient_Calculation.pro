#-------------------------------------------------
#
# Project created by QtCreator 2014-08-04T16:13:50
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = 3D_Gradient_Calculation
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    ../VMT_Calculate/intensitygradientcomputation.cpp \
    ../VMT_Calculate/depthtotolerance.cpp \
    ../VMT_Calculate/PointCloudFunctions.cpp

HEADERS += \
    ../VMT_Calculate/intensitygradientcomputation.h \
    ../VMT_Calculate/PointCloudFunctions.h

INCLUDEPATH += "../VMT_Calculate"
INCLUDEPATH += /usr/include/pcl-1.7
INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/include/vtk

LIBS += -lpcl_common -lpcl_visualization -lpcl_filters -lpcl_io -lboost_system

LIBS += -lopencv_core \
-lopencv_imgproc \
-lopencv_highgui

LIBS += -L/usr/lib64/vtk \
-lvtkCommonCore \
-lvtkRenderingCore \
-lvtkRenderingAnnotation \
-lvtkRenderingLOD \
-lvtkCommonDataModel \
-lvtkCommonMath \
-lboost_thread


