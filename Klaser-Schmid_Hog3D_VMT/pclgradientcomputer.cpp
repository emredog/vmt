#include "pclgradientcomputer.h"

#include <opencv/vmt.h>
#include <opencv/cv.h>
#include <opencv/functions.h>
#include <cmath>


PclGradientComputer::~PclGradientComputer()
{
    //TODO
}

PclGradientComputer::VectorType PclGradientComputer::getGradientVector(const Box3D &box) const
{
    VectorType vec(3);
    //TODO

    return vec;
}

bool PclGradientComputer::isInBuffer(const Box3D &box) const
{
    //TODO
    return true;
}

int PclGradientComputer::getWidth() const
{    
    return _vmt->getWidth();
}

int PclGradientComputer::getHeight() const
{
    //TODO
    return _vmt->getHeight();
}

int PclGradientComputer::getDepth() const
{
    //TODO
    return _vmt->getDepth();
}

void PclGradientComputer::init()
{
    if (!_vmt)
        return;

    //TODO
}
