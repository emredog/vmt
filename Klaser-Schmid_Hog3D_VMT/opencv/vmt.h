#ifndef VMT_H
#define VMT_H

#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

class Vmt
{
public:
    typedef /*long*/ long FrameIndex;
protected:

    int _width;
    int _height;
    int _depth;

    cv::SparseMat _sparseMat;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _pointCloud;


public:
    //constructor
    Vmt(cv::SparseMat sparseMat);
    Vmt(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int width, int height, int depth);

    //desctructor
    ~Vmt();

    int getWidth() const { return this->_width; }
    int getHeight() const { return this->_height; }
    int getDepth() const { return this->_depth; }

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr getPointCloud() const;
    cv::SparseMat getSparseMat() const {return _sparseMat;}
};

#endif // VMT_H
