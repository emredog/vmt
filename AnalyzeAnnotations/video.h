#ifndef VIDEO_H
#define VIDEO_H

#include <QString>
#include <QMap>

#include "action.h"

class Video
{
public:
    Video(QString fullPath);

    QString name;

    QMap<int, Action> actions;

    int getNumberOfActions();
    void printSummary();

};

#endif // VIDEO_H
