#ifndef PCLGRADIENTCOMPUTER_H
#define PCLGRADIENTCOMPUTER_H

#include <vector>
#include <cmath>
#include <map>

#include <boost/numeric/ublas/vector.hpp>
#include "Box3D.h"
#include <geometry/Box.hpp>

#include <opencv/IplImageWrapper.h>

class Vmt;

class PclGradientComputer
{
public:
    //constructor
    PclGradientComputer(Vmt* vmt)
    : _vmt(vmt)
    {
        init();
    }

    typedef boost::numeric::ublas::vector<double> VectorType;

    virtual ~PclGradientComputer();

    //main (essential) function of this class
    VectorType getGradientVector(const Box3D& box) const;


    //maybe we can get rid of this
    bool isInBuffer(const Box3D& box) const;

    bool isInVideo(const Box3D& box) const
    {
        if (box.z < 0 || (box.z + box.depth) >= getDepth())
            return false;
        Box<double> box2DVideo(0, 0, getWidth(), getHeight());
        Box<double> box2D(box.x, box.y, box.width, box.height);
        return box2DVideo.contains(box2D);
    }

    int getWidth() const;
    int getHeight() const;
    int getDepth() const;

protected:
    Vmt* _vmt;
    void init();
};

#endif // PCLGRADIENTCOMPUTER_H
