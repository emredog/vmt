#include "action.h"
#include "boundingbox.h"

Action::Action()
{
    this->activityClass = -1;
    this->number = -1;
    //this->parentVideo = 0;
}

int Action::getDuration() const
{
    return this->boundingBoxes.size();
}

bool Action::operator==(const Action& other) const
{
    int minDuration = qMin(this->getDuration(), other.getDuration());

    if (qAbs(this->getDuration()-other.getDuration()) > minDuration/4) //difference bw durations is too high, can't be equal
        return false;

    int overlapCounter = 0;
    QMap<int,BoundingBox> lessBoundingBoxes = (minDuration == this->getDuration())? this->boundingBoxes : other.boundingBoxes;
    QList<int> keys = lessBoundingBoxes.keys();
    qSort(keys);
    for(int i=keys.first(); i<=keys.last(); i++)
    {
        BoundingBox thisBbox  = this->boundingBoxes.value(i);
        BoundingBox otherBbox = other.boundingBoxes.value(i);
        if (thisBbox.frameNr == -1 || otherBbox.frameNr == -1) //one of the bboxes are missing for this frame
            continue;

        if (thisBbox.overlaps(otherBbox)) //if theres an overlap
        {
            overlapCounter++;
            if (overlapCounter > minDuration/4) //if 1/4th of the frames overlap, consider it similar
                return true;
        }
    }

    return false;
}
