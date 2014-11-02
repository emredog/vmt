#ifndef INTERPOLATEVMT_H
#define INTERPOLATEVMT_H

#include <QMap>
#include <QPair>

#include "opencv/vmt.h"

class InterpolateVmt
{
protected:

    QMap<float, cv::Point3i> findEnclosingPts(cv::Point3i pt, const cv::Mat& mat, int wid, int hei, int dep, int sLimit=50); //search Limit

    enum Octant { ppp=0, mpp, mmp, pmp, ppm, mpm, mmm, pmm};

    Vmt Interpolate_Trilinear(const Vmt& vmt);

    QMap<int, uchar> InterpolateArray(const QPair<int, uchar>& prevPt, const QPair<int, uchar>& nextPt);

    static inline void copyElem(const uchar* from, uchar* to, size_t elemSize)
    {
        size_t i;
        for( i = 0; i + sizeof(int) <= elemSize; i += sizeof(int) )
            *(int*)(to + i) = *(const int*)(from + i);
        for( ; i < elemSize; i++ )
            to[i] = from[i];
    }


public:
    InterpolateVmt();
    Vmt Interpolate(const Vmt& vmt, int maxSegmentLength);
    //smoothing with moving least squares
    pcl::PointCloud<pcl::PointXYZI> smootingMLS(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud);


};

#endif // INTERPOLATEVMT_H
