#include "boundingbox.h"
#include "action.h"
#include <QRect>

BoundingBox::BoundingBox()
{
    this->x = -1;
    this->y = -1;
    this->width = -1;
    this->height = -1;
    this->frameNr = -1;
    this->score = -1.0;
    //this->parentAction = 0;
}

BoundingBox::~BoundingBox()
{
    //this->parentAction = 0;
}

bool BoundingBox::overlaps(BoundingBox &bbox)
{
    QRect thisRect(this->x, this->y, this->width, this->height);
    QRect otherRect(bbox.x, bbox.y, bbox.width, bbox.height);

    return thisRect.intersects(otherRect);
}
