#ifndef INTERPOLATEVMT_H
#define INTERPOLATEVMT_H

#include <QMap>
#include <QPair>

#include "opencv/vmt.h"

class InterpolateVmt
{
protected:

    QMap<float, cv::Point3i> findEnclosingPts(cv::Point3i pt, const cv::Mat& mat, int wid, int hei, int dep, int sLimit=50); //search Limit
    uchar calculateInterpolationVal(QMap<float, cv::Point3i>, cv::Point3i curPoint);
    enum Octant { ppp=0, mpp, mmp, pmp, ppm, mpm, mmm, pmm};

    Vmt Interpolate_Trilinear(const Vmt& vmt);

    QMap<int, uchar> InterpolateArray(const QPair<int, uchar>& prevPt, const QPair<int, uchar>& nextPt);
public:
    InterpolateVmt();
    Vmt Interpolate(const Vmt& vmt);

};

#endif // INTERPOLATEVMT_H
