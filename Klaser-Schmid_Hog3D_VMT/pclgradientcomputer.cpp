#include "pclgradientcomputer.h"

#include <opencv/vmt.h>
#include <opencv/cv.h>
#include <opencv/functions.h>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/intensity_gradient.h>


PclGradientComputer::~PclGradientComputer()
{
    //TODO
}

PclGradientComputer::VectorType PclGradientComputer::getGradientVector(const Box3D &box) const
{
    //initialize a zero vector
    VectorType vec(3); vec[0] = 0.0; vec[1] = 0.0; vec[2] = 0.0;

    double radiusToSearch = 10*std::max(std::max(box.width, box.height), box.depth); //FIXME: radius to search?

    //crop the VMT in a new point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr croppedCloud(new pcl::PointCloud<pcl::PointXYZI>);
    {
        pcl::PointXYZ minPt(box.x, box.y, box.z);
        pcl::PointXYZ maxPt(box.x + box.width, box.y + box.height, box.z + box.depth);

        pcl::CropBox<pcl::PointXYZI> cropBoxFilter;
        cropBoxFilter.setInputCloud(_vmt->getPointCloud());
        cropBoxFilter.setMin(minPt.getArray4fMap());
        cropBoxFilter.setMax(maxPt.getArray4fMap());

        cropBoxFilter.filter(*croppedCloud);
    }

    //case of 1 or less points (we need at least 2 points to form a intensity gradient vector)
    if (croppedCloud->points.size() < 2)
    {
        return vec;
    }

    //estimate surface normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
    {
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> norm_est;
        norm_est.setInputCloud(croppedCloud);
        norm_est.setRadiusSearch(radiusToSearch);
        norm_est.compute(*normals);
    }

    //estimate all intensity gradients
    pcl::PointCloud<pcl::IntensityGradient>::Ptr gradients (new pcl::PointCloud<pcl::IntensityGradient> ());
    {
        pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient> grad_est;
        grad_est.setInputCloud(croppedCloud);
        grad_est.setInputNormals(normals);
        grad_est.setRadiusSearch(radiusToSearch);
        grad_est.compute(*gradients);
    }

    //clean-up:
    croppedCloud.reset();
    normals.reset();

    //calculate mean gradients of the cropped box
    double sumDx = 0.0, sumDy = 0.0, sumDz = 0.0;

    for (size_t i=0; i<gradients->points.size(); i++)
    {
        double dx = gradients->points[i].gradient_x;
        double dy = gradients->points[i].gradient_y;
        double dz = gradients->points[i].gradient_z;

        if (isnan(dx) || isnan(dy) || isnan(dz))
            continue;

        sumDx += dx;
        sumDy += dy;
        sumDz += dz;

    }

    vec[0] = sumDx / (double)gradients->points.size();
    vec[1] = sumDy / (double)gradients->points.size();
    vec[2] = sumDz / (double)gradients->points.size();

    //FIXME: do I need to check for the origin point of the gradient? (i guess not..)

    return vec;
}

bool PclGradientComputer::isInBuffer(const Box3D &box) const
{
    //TODO
    return true;
}

int PclGradientComputer::getWidth() const
{    
    return _vmt->getWidth();
}

int PclGradientComputer::getHeight() const
{
    //TODO
    return _vmt->getHeight();
}

int PclGradientComputer::getDepth() const
{
    //TODO
    return _vmt->getDepth();
}

