#-------------------------------------------------
#
# Project created by QtCreator 2014-08-26T10:44:20
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = CameraMotion
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    ../VMT_Calculate/PointCloudFunctions.cpp \
    ../VMT_Calculate/VmtFunctions.cpp \
    ../AnalyzeAnnotations/boundingbox.cpp \
    ../VMT_Calculate/depthtotolerance.cpp \
    LK_Demo.cpp


INCLUDEPATH += /usr/include/pcl-1.7
INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += ../VMT_Calculate
INCLUDEPATH += ../AnalyzeAnnotations

LIBS += -lpcl_common -lpcl_visualization -lpcl_filters -lpcl_io -lpcl_sample_consensus -lpcl_segmentation\
-lboost_system \
-lopencv_core \
-lopencv_imgproc \
-lopencv_highgui \
-lopencv_video

INCLUDEPATH += /usr/include/vtk

LIBS += -L/usr/lib64/vtk/ \
-lvtkCommonCore -lvtkRenderingCore -lvtkRenderingAnnotation \
-lvtkRenderingLOD -lvtkCommonDataModel -lvtkCommonMath -lboost_system -lboost_thread

HEADERS += \
    ../VMT_Calculate/PointCloudFunctions.h \
    ../VMT_Calculate/VmtFunctions.h \
    ../AnalyzeAnnotations/boundingbox.h \
    ../VMT_Calculate/depthtotolerance.h
