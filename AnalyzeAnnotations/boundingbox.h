#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

class Action;

class BoundingBox
{
public:
    BoundingBox();
    ~BoundingBox();

    int x, y, width, height, frameNr;
    float score;

    inline bool operator< (const BoundingBox& that){return this->frameNr < that.frameNr; }
    inline bool operator> (const BoundingBox& that){return this->frameNr > that.frameNr; }
    inline bool operator<=(const BoundingBox& that){return this->frameNr <= that.frameNr; }
    inline bool operator>=(const BoundingBox& that){return this->frameNr >= that.frameNr; }

    bool overlaps(BoundingBox &bbox);

    //Action* parentAction;
};

#endif // BOUNDINGBOX_H
