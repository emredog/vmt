#include "boundingbox.h"
#include "action.h"

BoundingBox::BoundingBox()
{
    this->x = -1;
    this->y = -1;
    this->width = -1;
    this->height = -1;
    this->frameNr = -1;
    //this->parentAction = 0;
}

BoundingBox::~BoundingBox()
{
    //this->parentAction = 0;
}
