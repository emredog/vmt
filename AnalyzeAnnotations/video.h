#ifndef VIDEO_H
#define VIDEO_H

#include <QString>
#include <QMap>

#include "action.h"

class Video
{
public:
    Video(QString fullPath);
    Video();

    QString name;

    QMap<int, Action> actions;

    int getNumberOfActions();
    void printSummary();

    void writeWithAnnotationFormat(QString fullPath);

};

#endif // VIDEO_H
