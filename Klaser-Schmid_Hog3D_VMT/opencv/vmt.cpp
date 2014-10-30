#include "vmt.h"

// std
#include <stdexcept>

// my stuff
#include <opencv/IplImageWrapper.h>
//#include <opencv/functions.h>
//#include <numeric/functions.hpp>
#include <vmt_calculation/PointCloudFunctions.h>

const bool DEBUGOUT = false;

Vmt::Vmt(cv::SparseMat sparseMat)
{
    _width = 0;
    _height = 0;
    _depth = 0;

    _sparseMat = sparseMat;

    if (_sparseMat.nzcount() > 0)
    {
        //FIXME: check the indices..
        _width  = _sparseMat.size()[0];
        _height = _sparseMat.size()[1];
        _depth  = _sparseMat.size()[2];

        _pointCloud = PointCloudFunctions::convertToPointCloud(_sparseMat);
    }
}

Vmt::Vmt(PointCloud<pcl::PointXYZI>::Ptr cloud, int width, int height, int depth)
{
    _width = width;
    _height = height;
    _depth = depth;

    _pointCloud = cloud;

    if (!cloud->empty())
    {
        int sizes[] = {width, height, depth};
        _sparseMat = PointCloudFunctions::convertToSparseMat(cloud, 3, sizes);
    }
}

Vmt::~Vmt()
{
    //clean up
    _width = 0;
    _height = 0;
    _depth = 0;

    _sparseMat.release();
    if (_pointCloud)
    {
        _pointCloud->clear();
        _pointCloud.reset();
    }
}

Vmt::Vmt(const Vmt &other) : _width(other._width), _height(other._height), _depth(other._height)
{
    _sparseMat = other._sparseMat;
    //take a deep copy of the point cloud:
    _pointCloud = other._pointCloud->makeShared();
}

Vmt &Vmt::operator=(const Vmt &other)
{
    _width  = other._width;
    _height = other._height;
    _depth  = other._depth;

    _sparseMat = other._sparseMat;
    //take a deep copy of the point cloud:
    _pointCloud = other._pointCloud->makeShared();
}

pcl::PointCloud<pcl::PointXYZI>::ConstPtr Vmt::getPointCloud_Const() const
{
    return _pointCloud;
}

PointCloud<pcl::PointXYZI>::Ptr Vmt::getPointCloud() const
{
    return _pointCloud;
}
