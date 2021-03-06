#-------------------------------------------------
#
# Project created by QtCreator 2014-06-19T09:48:19
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = VMT_Calculate
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    HelperFunctions.cpp \
    PointCloudFunctions.cpp \
    VmtFunctions.cpp \
    ../AnalyzeAnnotations/boundingbox.cpp \
    depthtotolerance.cpp \
    ../3D_Gradient_Calculation/intensitygradientcomputation.cpp

HEADERS += \
    HelperFunctions.h \
    PointCloudFunctions.h \
    VmtFunctions.h \
    ../AnalyzeAnnotations/boundingbox.h \
    depthtotolerance.h \
    ../3D_Gradient_Calculation/intensitygradientcomputation.h

INCLUDEPATH += /usr/include/pcl-1.7
INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += ../AnalyzeAnnotations/
INCLUDEPATH += ../3D_Gradient_Calculation/

LIBS += -lpcl_common -lpcl_visualization -lpcl_filters -lpcl_io \
-lboost_system \
-lopencv_core \
-lopencv_imgproc \
-lopencv_highgui \
#-lopencv_ml \
#-lopencv_video \
#-lopencv_features2d \
#-lopencv_calib3d \
#-lopencv_objdetect \
#-lopencv_contrib \
#-lopencv_legacy \
#-lopencv_flann
