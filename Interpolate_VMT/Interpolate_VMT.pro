#-------------------------------------------------
#
# Project created by QtCreator 2014-10-23T11:51:48
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = Interpolate_VMT
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

#paths for PCL
INCLUDEPATH += /usr/include/pcl-1.7
INCLUDEPATH += /usr/include/eigen3

#Libs for PCL
LIBS += -lpcl_common -lpcl_filters -lpcl_io -lpcl_search -lpcl_features

#libs for boost
LIBS += -lboost_regex -lboost_system -lboost_filesystem -lboost_program_options

#Libs for OpenCV
LIBS += -lopencv_core -lopencv_flann

#path for vmt.h
INCLUDEPATH += /home/emredog/git/vmt/Klaser-Schmid_Hog3D_VMT/

#path for PointCloudFunctions
INCLUDEPATH += /home/emredog/git/vmt/Klaser-Schmid_Hog3D_VMT/vmt_calculation/


SOURCES += main.cpp \
    ../Klaser-Schmid_Hog3D_VMT/opencv/vmt.cpp \
    ../Klaser-Schmid_Hog3D_VMT/vmt_calculation/PointCloudFunctions.cpp \
    interpolatevmt.cpp

HEADERS += \
    ../Klaser-Schmid_Hog3D_VMT/opencv/vmt.h \
    ../Klaser-Schmid_Hog3D_VMT/vmt_calculation/PointCloudFunctions.h \
    interpolatevmt.h
