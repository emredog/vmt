//#include <QCoreApplication>
//#include <QStringList>
//#include <QDir>

//#include "VmtFunctions.h"
//#include "PointCloudFunctions.h"

//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>

//using namespace cv;
//using namespace pcl;
//using namespace pcl::visualization;

//boost::shared_ptr<PCLVisualizer> viewCloud (PointCloud<PointXYZI>::ConstPtr cloud, ModelCoefficients::Ptr coefs);

//int main(int argc, char *argv[])
//{
//    QCoreApplication a(argc, argv);
//    VmtFunctions vmtCore;


//    //return a.exec();

//    QString videoFolderPath = "/home/emredog/LIRIS-data/training-validation/vid0004/";
//    //get Depth image paths in the folder
//    QStringList filters;
//    filters << "*.jp2";
//    QDir videoDir(videoFolderPath);
//    QStringList depthImgFileNames = videoDir.entryList(filters, QDir::Files | QDir::NoDotAndDotDot, QDir::Name);

//    for (int i=0; i<depthImgFileNames.length(); i++)
//    {
//        //Read image
//        cv::Mat image = imread(videoDir.absoluteFilePath(depthImgFileNames[i]).toStdString(), CV_LOAD_IMAGE_UNCHANGED);



//        //Check for invalid input
//        if(! image.data )
//        {
//            cout <<  "Could not open or find the image" << endl ;
//            return -1;
//        }

//        SparseMat sparseVolObj = vmtCore.generateSparseVolumeObject(image, 2);

//        PointCloud<PointXYZI>::Ptr cloud = PointCloudFunctions::convertToPointCloud(sparseVolObj);


//        // Create the segmentation object
//        SACSegmentation<pcl::PointXYZI> seg;
//        ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//        PointIndices::Ptr inliers (new pcl::PointIndices);

//        // Optional
//        seg.setOptimizeCoefficients (true);
//        // Mandatory
//        seg.setModelType (pcl::SACMODEL_PLANE);
//        seg.setMethodType (pcl::SAC_RANSAC);
//        seg.setDistanceThreshold (0.01);

//        seg.setInputCloud (cloud);
//        seg.segment (*inliers, *coefficients);


//        if (inliers->indices.size () == 0)
//        {
//            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
//            continue;
//        }

//        std::cerr << "Model coefficients: " << coefficients->values[0] << ", "
//                  << coefficients->values[1] << ", "
//                  << coefficients->values[2] << ", "
//                  << coefficients->values[3] << std::endl;
//    }

//    //TODO: display plane (coefs are ax + by + cz + d = 0)

//    //    boost::shared_ptr<PCLVisualizer> viewer = viewCloud(cloud, coefficients);

//    //    //--------------------
//    //    // -----Main loop-----
//    //    //--------------------
//    //    while (!viewer->wasStopped ())
//    //    {
//    //        viewer->spinOnce (100);
//    //        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    //    }

//    return 0;
//}

//boost::shared_ptr<PCLVisualizer> viewCloud (PointCloud<PointXYZI>::ConstPtr cloud, ModelCoefficients::Ptr coefs)
//{
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new PCLVisualizer ("3D Viewer"));

//    viewer->setBackgroundColor (0, 0, 0);

//    PointCloudColorHandlerGenericField<PointXYZI> intensity(cloud, "intensity");

//    viewer->addCoordinateSystem (10.0);

//    viewer->addPointCloud<PointXYZI> (cloud, intensity, "vmt");

//    viewer->addPlane (*coefs, "plane");

//    pcl::ModelCoefficients coeffs;
//    coeffs.values.push_back(0.0);
//    coeffs.values.push_back(0.0);
//    coeffs.values.push_back(1.0);
//    coeffs.values.push_back(0.0);
//    viewer->addPlane (coeffs, "plane2");

//    viewer->initCameraParameters();
//    return (viewer);
//}
