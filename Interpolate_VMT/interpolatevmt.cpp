#include "interpolatevmt.h"


#include <QtCore/qmath.h>

using namespace std;
using namespace cv;

//struct QPairFirstComparer
//{
//    template<typename T1, typename T2>
//    bool operator()(const QPair<T1,T2> & a, const QPair<T1,T2> & b) const
//    {
//        return a.first < b.first;
//    }
//};

InterpolateVmt::InterpolateVmt()
{
}

Vmt InterpolateVmt::Interpolate(const Vmt &vmt)
{
    Mat matVmt;
    vmt.getSparseMat().copyTo(matVmt);

    //Size of vmt
    int width = matVmt.size[0];
    int heigth = matVmt.size[1];
    int depth = matVmt.size[2];

    CV_Assert(matVmt.type() == CV_8UC1); //chech if type is well CV_8UC1

    Mat interpolatedMat(matVmt.dims, matVmt.size, matVmt.type());
    interpolatedMat = matVmt;

    //some common variables:
    Range ranges[3];

    //PART I: Interpolate in X direction
    int counter = 0;
    ranges[0] = Range::all(); //take all on X
    for(int y=0; y<heigth; y++)
        for (int z=0; z<depth; z++)
        {
            ranges[1] = Range(y, y+1);
            ranges[2] = Range(z, z+1);

            Mat currentSegment; //(width, 1, CV_8UC1);
            currentSegment = matVmt(ranges);

            SparseMat sparse_mat(currentSegment);
            if (sparse_mat.nzcount() >= 2)
            {
                QList<QPair<int, uchar> > ptsInSegment;

                SparseMatConstIterator_<uchar> it = sparse_mat.begin<uchar>(), it_end = sparse_mat.end<uchar>();
                for(; it != it_end; ++it)
                {
                    // take every non-zero points
                    const SparseMat::Node* n = it.node();
                    ptsInSegment.append(qMakePair(n->idx[0], it.value<uchar>()));
                }

                //sort points in segment
                qSort(ptsInSegment);
                for (int i=0; i<ptsInSegment.length()-1; i++)
                {
                    QMap<int, uchar> interPolatedPoints = InterpolateArray(ptsInSegment[i], ptsInSegment[i+1]);
                    QMapIterator<int, uchar> mapIt(interPolatedPoints);
                    //insert points:
                    while(mapIt.hasNext())
                    {
                        mapIt.next();
                        interpolatedMat.at<uchar>(mapIt.key(), y, z) = mapIt.value();
                        counter++;
                    }
                }
            }
        }

    cerr << "# " << counter << " points added for interpolation in X direction.\n";

    //PART II: Interpolate in Y direction

    counter = 0;
    ranges[1] = Range::all(); //take all on Y
    for(int x=0; x<width; x++)
        for (int z=0; z<depth; z++)
        {
            ranges[0] = Range(x, x+1);
            ranges[2] = Range(z, z+1);

            Mat currentSegment; //(width, 1, CV_8UC1);            
            currentSegment = matVmt(ranges);

            SparseMat sparse_mat(currentSegment);
            if (sparse_mat.nzcount() >= 2)
            {
                QList<QPair<int, uchar> > ptsInSegment;

                SparseMatConstIterator_<uchar> it = sparse_mat.begin<uchar>(), it_end = sparse_mat.end<uchar>();
                for(; it != it_end; ++it)
                {
                    // take every non-zero points
                    const SparseMat::Node* n = it.node();
                    ptsInSegment.append(qMakePair(n->idx[1], it.value<uchar>()));
                }

                //sort points in segment
                qSort(ptsInSegment);
                for (int i=0; i<ptsInSegment.length()-1; i++)
                {
                    QMap<int, uchar> interPolatedPoints = InterpolateArray(ptsInSegment[i], ptsInSegment[i+1]);
                    QMapIterator<int, uchar> mapIt(interPolatedPoints);
                    //insert points:
                    while(mapIt.hasNext())
                    {
                        mapIt.next();                        
                        interpolatedMat.at<uchar>(x, mapIt.key(), z) = mapIt.value();
                        counter++;
                    }
                }
            }
        }

    cerr << "# " << counter << " points added for interpolation in Y direction.\n";

    //PART III: Interpolate in Z direction

    counter = 0;
    ranges[2] = Range::all(); //take all on Z
    for(int x=0; x<width; x++)
        for (int y=0; y<heigth; y++)
        {
            ranges[0] = Range(x, x+1);
            ranges[1] = Range(y, y+1);

            Mat currentSegment; //(width, 1, CV_8UC1);
//            currentSegment = inpolatedInXY(ranges);,
            currentSegment = matVmt(ranges);

            SparseMat sparse_mat(currentSegment);
            if (sparse_mat.nzcount() >= 2)
            {
                QList<QPair<int, uchar> > ptsInSegment;

                SparseMatConstIterator_<uchar> it = sparse_mat.begin<uchar>(), it_end = sparse_mat.end<uchar>();
                for(; it != it_end; ++it)
                {
                    // take every non-zero points
                    const SparseMat::Node* n = it.node();
                    ptsInSegment.append(qMakePair(n->idx[2], it.value<uchar>()));
                }

                //sort points in segment
                qSort(ptsInSegment);
                for (int i=0; i<ptsInSegment.length()-1; i++)
                {
                    QMap<int, uchar> interPolatedPoints = InterpolateArray(ptsInSegment[i], ptsInSegment[i+1]);
                    QMapIterator<int, uchar> mapIt(interPolatedPoints);
                    //insert points:
                    while(mapIt.hasNext())
                    {
                        mapIt.next();
                        //inpolatedInXYZ.at<uchar>(x, y, mapIt.key()) = mapIt.value();
                        interpolatedMat.at<uchar>(x, y, mapIt.key()) = mapIt.value();
                        counter++;
                    }
                }
            }
        }

    cerr << "# " << counter << " points added for interpolation in Z direction.\n";

    SparseMat spaaa(interpolatedMat);
    interpolatedMat.release();

    return Vmt(spaaa);
}

QMap<int, uchar> InterpolateVmt::InterpolateArray(const QPair<int, uchar> &prevPt, const QPair<int, uchar> &nextPt)
{
    if(nextPt.first - prevPt.first < 2) //should be 2 different points, with at least 1 point inbetween
        return QMap<int, uchar>(); //return an empty map

    QMap<int, uchar> interpolatedPoints;

    uchar interpolatedValue = 0;
    for (int xi=prevPt.first+1; xi<nextPt.first; xi++)
    {
        interpolatedValue = (uchar)((int)prevPt.second + ((int)nextPt.second-(int)prevPt.second)*(xi-prevPt.first)/(prevPt.first-nextPt.first));
        if (interpolatedValue > 0)
            interpolatedPoints.insert(xi, interpolatedValue);
    }

    return interpolatedPoints;
}

QMap<float, cv::Point3i> InterpolateVmt::findEnclosingPts(cv::Point3i pt, const cv::Mat &mat, int wid, int hei, int dep, int sLimit)
{
    //http://upload.wikimedia.org/wikipedia/commons/thumb/7/7f/Enclosing_points2.svg/220px-Enclosing_points2.svg.png

    int startX  = pt.x >= sLimit ? pt.x - sLimit : 0;
    int startY  = pt.y >= sLimit ? pt.y - sLimit : 0;
    int startZ  = pt.z >= sLimit ? pt.z - sLimit : 0;
    int endX    = pt.x + sLimit < wid ? pt.x + sLimit : wid-1;
    int endY    = pt.y + sLimit < hei ? pt.y + sLimit : hei-1;
    int endZ    = pt.z + sLimit < dep ? pt.z + sLimit : dep-1;

    QMap<float, cv::Point3i> distancesToPoints;

    QMap<Octant, QMap<int, cv::Point3i> > distancesToAllPointsByOctant;
    distancesToAllPointsByOctant.insert(ppp, QMap<int, cv::Point3i>());
    distancesToAllPointsByOctant.insert(mpp, QMap<int, cv::Point3i>());
    distancesToAllPointsByOctant.insert(mmp, QMap<int, cv::Point3i>());
    distancesToAllPointsByOctant.insert(pmp, QMap<int, cv::Point3i>());
    distancesToAllPointsByOctant.insert(ppm, QMap<int, cv::Point3i>());
    distancesToAllPointsByOctant.insert(mpm, QMap<int, cv::Point3i>());
    distancesToAllPointsByOctant.insert(mmm, QMap<int, cv::Point3i>());
    distancesToAllPointsByOctant.insert(pmm, QMap<int, cv::Point3i>());

    for (int offX=startX; offX < endX; offX++)
    {
        for (int offY=startY; offY < endY; offY++)
        {
            for (int offZ=-startZ; offZ < endZ; offZ++)
            {
                if (offX == 0 && offY == 0 && offZ == 0)
                    continue;

                Octant currentOctant;

                //FIXME maybe use byte operations to get rid off 8 lines of "if-else"? 001 010 etc..

                if (offX>=0 && offY>=0 && offZ>=0) currentOctant = ppp;
                else if (offX<=0 && offY>=0 && offZ>=0) currentOctant = mpp;
                else if (offX<=0 && offY<=0 && offZ>=0) currentOctant = mmp;
                else if (offX>=0 && offY<=0 && offZ>=0) currentOctant = pmp;
                else if (offX>=0 && offY>=0 && offZ<=0) currentOctant = ppm;
                else if (offX<=0 && offY>=0 && offZ<=0) currentOctant = mpm;
                else if (offX<=0 && offY<=0 && offZ<=0) currentOctant = mmm;
                else /*(offX>=0 && offY<=0 && offZ<=0)*/ currentOctant = pmm;

                uchar val = mat.at<uchar>(pt.x + offX, pt.y + offY, pt.z + offZ);
                if (val > 0)
                {
                    cv::Point3i neighbor(pt.x + offX, pt.y + offY, pt.z + offZ);
                    QMap<Octant, QMap<int, cv::Point3i> >::iterator it = distancesToAllPointsByOctant.find(currentOctant);
                    it.value().insert((offX*offX + offY*offY + offZ*offZ), neighbor);
                    //                    distancesToAllPointsByOctant.value(currentOctant).insert();
                }
            }
        }
    }

    QMap<Octant, QMap<int, cv::Point3i> >::const_iterator it = distancesToAllPointsByOctant.constBegin();
    for (; it!=distancesToAllPointsByOctant.constEnd(); it++)
    {
        if (it.value().empty()) //no values in this octant
            return QMap<float, cv::Point3i>(); //return an empty map

        QMap<int, cv::Point3i>::const_iterator it2 = it.value().begin(); //first element (with smallest distance)

        float distance = qSqrt((double)it2.key());
        distancesToPoints.insert(distance, it2.value());
    }

    return distancesToPoints;

}

uchar InterpolateVmt::calculateInterpolationVal(QMap<float, cv::Point3i>, cv::Point3i curPoint)
{
    //points are in this order: ppp=0, mpp, mmp, pmp, ppm, mpm, mmm, pmm
    uchar newVal = 0;

    return newVal;
}

Vmt InterpolateVmt::Interpolate_Trilinear(const Vmt &vmt)
{
    cv::Mat matVmt;
    vmt.getSparseMat().copyTo(matVmt);
    //convert sparse mat to regular mat

    cv::Mat interpolatedMat(matVmt.dims, vmt.getSparseMat().size(), matVmt.type());
    interpolatedMat = cv::Scalar(0);

    int width = vmt.getWidth();
    int heigth = vmt.getHeight();
    int depth = vmt.getDepth();

    int counter = 0;
    int counterZero = 0;
    int counterPositive = 0;

    for(int x=0; x<width; x++)
        for (int y=0; y<heigth; y++)
            for (int z=0; z<depth; z++)
            {
                uchar val = matVmt.at<uchar>(x,y,z);
                if (val > 0) //no need to interpolate
                {
                    counterPositive++;
                    interpolatedMat.at<uchar>(x,y,z) = val;
                    continue;
                }
                else
                {
                    counterZero++;
                    cv::Point3i testPt(x,y,z);
                    QMap<float, cv::Point3i> distanceToPoints = findEnclosingPts(testPt, matVmt, width, heigth, depth, 20);
                    if (distanceToPoints.empty())
                        continue; //do not interpolate for this point

                    counter++;
                    //interpolatedMat.at<uchar>(x,y,z) = calculateInterpolationVal(distanceToPoints, testPt);
                }
            }

    std::cout << counterPositive << " positive points \n";
    std::cout << counterZero << " zero points \n";
    std::cout << counter << " more points after interpolation.\n\n";


    cv::SparseMat interpolatedSparseMat(interpolatedMat);

    Vmt resultingVmt(interpolatedSparseMat);

    return resultingVmt;


}




