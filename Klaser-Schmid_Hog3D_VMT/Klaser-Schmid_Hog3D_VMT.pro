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

#INCLUDEPATH += /usr/include/ffmpeg
LIBS += -lboost_regex -lboost_system -lboost_filesystem -lboost_program_options \
-lavformat -lavutil -lavcodec -lswscale \
-lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d \
-lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_legacy \
-lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching \
-lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab


SOURCES += main.cpp \
#    ffpp/Codec.cpp \
#    ffpp/CodecContext.cpp \
#    ffpp/FlipContext.cpp \
#    ffpp/FormatContext.cpp \
#    ffpp/Frame.cpp \
#    ffpp/InputFormat.cpp \
#    ffpp/InputFormatContext.cpp \
#    ffpp/OutputFormat.cpp \
#    ffpp/OutputFormatContext.cpp \
#    ffpp/Packet.cpp \
#    ffpp/ScalerContext.cpp \
#    ffpp/Stream.cpp \
#    ffpp/PixelFormat.c \
    opencv/functions.cpp \
    opencv/IplImageWrapper.cpp \
    FastHog3DComputer.cpp \
    FastVideoGradientComputer.cpp \
    opencv/vmt.cpp \
    pclgradientcomputer.cpp

HEADERS += \
#    ffpp/Codec.h \
#    ffpp/CodecContext.h \
#    ffpp/FlipContext.h \
#    ffpp/FormatContext.h \
#    ffpp/Frame.h \
#    ffpp/InputFormat.h \
#    ffpp/InputFormatContext.h \
#    ffpp/OutputFormat.h \
#    ffpp/OutputFormatContext.h \
#    ffpp/Packet.h \
#    ffpp/PixelFormat.h \
#    ffpp/ScalerContext.h \
#    ffpp/Stream.h \
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
    FastVideoGradientComputer.h \
    opencv/vmt.h \
    pclgradientcomputer.h
