#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

class Action;

class BoundingBox
{
public:
    BoundingBox();
    ~BoundingBox();

    int x, y, width, height, frameNr;

    //Action* parentAction;
};

#endif // BOUNDINGBOX_H
