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
    Vmt(){_width = -1; _height = -1; _depth = -1;}
    Vmt(cv::SparseMat sparseMat);
//    Vmt(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int width, int height, int depth);
    Vmt(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    //desctructor
    ~Vmt();

    //copy constructor
    Vmt(const Vmt& other);

    //assignment operator
    Vmt& operator=(const Vmt& other);

    int getWidth() const { return this->_width; }
    int getHeight() const { return this->_height; }
    int getDepth() const { return this->_depth; }

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr getPointCloud_Const() const;
    pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud() const;

    cv::SparseMat getSparseMat() const {return _sparseMat;}
};

#endif // VMT_H
