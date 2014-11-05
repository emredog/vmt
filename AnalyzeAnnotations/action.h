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

    bool operator==(const Action& other) const;
    bool operator> (const Action& other) const;
    bool operator>= (const Action& other) const;
    bool operator< (const Action& other) const;
    bool operator<= (const Action& other) const;

    bool fillBoxesFromTrackFile(QString pathToTrack);

   // Video* parentVideo;

    int getDuration() const;
};

#endif // ACTION_H
