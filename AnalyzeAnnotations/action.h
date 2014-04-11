#ifndef ACTION_H
#define ACTION_H

#include "boundingbox.h"

#include <QMap>

class Action
{
public:
    Action();

    int number;
    int activityClass;

    QMap<int, BoundingBox> boundingBoxes;

   // Video* parentVideo;

    int getDuration();
};

#endif // ACTION_H
