#include <QCoreApplication>

#include "VmtFunctions.h"
#include "PointCloudFunctions.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

using namespace cv;
using namespace pcl;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    VmtFunctions vmtCore;


    //return a.exec();

    //Read image:
    Mat image = imread("/home/emredog/LIRIS-data/training-validation/vid0004/depth-000001.jp2", CV_LOAD_IMAGE_UNCHANGED);


    //Check for invalid input
    if(! image.data )
    {
        cout <<  "Could not open or find the image" << endl ;
        return -1;
    }

    SparseMat sparseVolObj = vmtCore.generateSparseVolumeObject(image, 2);

    PointCloud<PointXYZI>::Ptr cloud = PointCloudFunctions::convertToPointCloud(sparseVolObj);


    // Create the segmentation object
    SACSegmentation<pcl::PointXYZI> seg;
    ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    PointIndices::Ptr inliers (new pcl::PointIndices);

    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);


    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

//TODO: display plane (coefs are ax + by + cz + d = 0)
}
