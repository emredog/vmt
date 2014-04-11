#include "action.h"
#include "boundingbox.h"

Action::Action()
{
    this->activityClass = -1;
    this->number = -1;
    //this->parentVideo = 0;
}

int Action::getDuration()
{
    return this->boundingBoxes.size();
}
