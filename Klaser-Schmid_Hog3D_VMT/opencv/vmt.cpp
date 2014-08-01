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

Vmt::~Vmt()
{
    //clean up
    _width = 0;
    _height = 0;
    _depth = 0;

    _sparseMat.release();
    _pointCloud->clear();
    _pointCloud.reset();
}

pcl::PointCloud<pcl::PointXYZI>::ConstPtr Vmt::getPointCloud() const
{
    return _pointCloud;
}
