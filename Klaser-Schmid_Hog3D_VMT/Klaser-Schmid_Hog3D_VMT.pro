#-------------------------------------------------
#
# Project created by QtCreator 2014-07-02T09:04:52
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = Klaser-Schmid_Hog3D_VMT
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

#paths for PCL
INCLUDEPATH += /usr/include/pcl-1.7
INCLUDEPATH += /usr/include/eigen3

#Libs for PCL
LIBS += -lpcl_common -lpcl_filters -lpcl_io -lpcl_search -lpcl_features


LIBS += -lboost_regex -lboost_system -lboost_filesystem -lboost_program_options \
-lavformat -lavutil -lavcodec -lswscale \
-lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d \
-lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_legacy \
-lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching \
-lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab

#path for boundingbox.h
INCLUDEPATH += ../AnalyzeAnnotations/

SOURCES += main.cpp \
    opencv/functions.cpp \
    opencv/IplImageWrapper.cpp \
    FastHog3DComputer.cpp \
    opencv/vmt.cpp \
    pclgradientcomputer.cpp \
    vmtcalculator.cpp \
    vmt_calculation/depthtotolerance.cpp \
    vmt_calculation/PointCloudFunctions.cpp \
    vmt_calculation/VmtFunctions.cpp \
    ../AnalyzeAnnotations/boundingbox.cpp

HEADERS += \
    geometry/Box.h \
    geometry/Box.hpp \
    geometry/Point.h \
    geometry/Point.hpp \
    geometry/Size.h \
    geometry/Size.hpp \
    numeric/functions.h \
    numeric/functions.hpp \
    opencv/functions.h \
    opencv/functions.hpp \
    opencv/IplImageWrapper.h \
    opencv/IplImageWrapper.hpp \
    Box3D.h \
    FastHog3DComputer.h \
    opencv/vmt.h \
    pclgradientcomputer.h \
    vmtcalculator.h \
    vmt_calculation/depthtotolerance.h \
    vmt_calculation/PointCloudFunctions.h \
    vmt_calculation/VmtFunctions.h \
    ../AnalyzeAnnotations/boundingbox.h
