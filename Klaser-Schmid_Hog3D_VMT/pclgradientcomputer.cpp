#include "pclgradientcomputer.h"

#include <opencv/vmt.h>


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
    //TODO
    return 0;
}

int PclGradientComputer::getHeight() const
{
    //TODO
    return 0;
}

std::size_t PclGradientComputer::getVideoLength() const
{
    //TODO
    return static_cast<std::size_t>(0);
}

void PclGradientComputer::init()
{
    if (!_vmt)
        return;

    //TODO
}
