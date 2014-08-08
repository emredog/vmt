#ifndef OCVGRADIENTCOMPUTER_H
#define OCVGRADIENTCOMPUTER_H

#include "intensitygradientcomputation.h"

#include <boost/numeric/ublas/vector.hpp>
#include "Box3D.h"
#include <geometry/Box.hpp>
#include <opencv/IplImageWrapper.h>
#include <opencv2/core/core.hpp>


class Vmt;

class OcvGradientComputer
{
public:
    OcvGradientComputer(Vmt* vmt);

    typedef boost::numeric::ublas::vector<double> VectorType;

    virtual ~OcvGradientComputer();

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
    cv::Mat _vmtMat;

    cv::Mat cropVmt(const Box3D& box) const;
};

#endif // OCVGRADIENTCOMPUTER_H
