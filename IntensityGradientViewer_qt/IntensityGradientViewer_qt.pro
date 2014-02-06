#-------------------------------------------------
#
# Project created by QtCreator 2014-02-06T18:36:13
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = IntensityGradientViewer_qt
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp

INCLUDEPATH += /usr/include/pcl-1.7 \
/usr/include/eigen3 \
/usr/include/vtk


LIBS += -L/usr/lib64/vtk \
-lpcl_common \
-lpcl_features \
-lpcl_filters \
-lpcl_io \
-lpcl_io_ply \
-lpcl_kdtree \
-lpcl_keypoints \
-lpcl_octree \
-lpcl_outofcore \
-lpcl_people \
-lpcl_recognition \
-lpcl_registration \
-lpcl_sample_consensus \
-lpcl_search \
-lpcl_segmentation \
-lpcl_surface \
-lpcl_tracking \
-lpcl_visualization \
-lboost_system -lboost_thread \
-lopencv_core \
-lopencv_imgproc \
-lopencv_highgui \
-lopencv_features2d \
-lopencv_objdetect \
-lopencv_contrib \
-lopencv_legacy \
-lopencv_flann \
-lvtkCommonCore \
-lvtkRenderingCore \
-lvtkRenderingAnnotation \
-lvtkRenderingLOD \
-lvtkCommonDataModel \
-lvtkCommonMath
