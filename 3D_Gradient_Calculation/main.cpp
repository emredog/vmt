#include "intensitygradientcomputation.h"
#include "PointCloudFunctions.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

using namespace pcl;
using namespace pcl::visualization;

using namespace cv;

boost::shared_ptr<PCLVisualizer> viewMeanGradient (PointCloud<PointXYZI>::ConstPtr cloud, cv::Vec3f meanGrad);

int main()
{
    //#	Size after trimming: 193x311x526, for vid0008_1_typing_716-756Union_STFiltered.pcd
    //    SparseMat spVmt = PointCloudFunctions::loadVmtFromPCD("/home/emredog/Documents/output/vid0008_1_typing_716-756Union_STFiltered.pcd",
    //                                                          193, 311, 526);


    SparseMat spVmt = PointCloudFunctions::loadVmtFromPCD("/home/emredog/Documents/output/test_cubes/testCube_X_5.pcd", 40, 40, 40);

    Mat vmt;
    spVmt.convertTo(vmt, CV_8UC1);


    //    cv::Range ranges[3];
    //    //some cube that I know there are points in it
    //    ranges[0] = cv::Range(40, 80);
    //    ranges[1] = cv::Range(120, 156);
    //    ranges[2] = cv::Range(452, 484);

    //    Mat cube = IntensityGradientComputation::crop3dMat(vmt, ranges);

    //    {
    //        SparseMat spCube(cube);

    //        nzCount = spCube.nzcount();

    //        spCube.release();
    //    }

    //create test cube
//    int sizes[] = {40, 40, 40};
//    Mat testCube(3, sizes, CV_8UC1);
//    testCube = Scalar(0);


//    int step = 2;



//    unsigned int val = 0;
//    for (int x=0; x<sizes[0]; x+=step)
//    {
//        val += 255 / 40;
//        for (int y=0; y<sizes[1]; y += step)
//            for (int z=0; z<sizes[2]; z+=step)

//                if ( x + z + y == 0)
//                    val = 0;
//                else
//                    val = 255 * (x + z + y);
//        testCube.at<uchar>(x, y, z) = static_cast<uchar>(val);

//    }
//    SparseMat testCubeSparse(testCube);
//    PointCloudFunctions::saveVmtAsCloud(testCubeSparse, "/home/emredog/Documents/output/testCube_X_2.pcd");


        IntensityGradientComputation::computeMeanIntensityGradientSobel3D(vmt);
        cv::Vec3f vec = IntensityGradientComputation::computeMeanIntensityGradientDifferentiation(vmt, IntensityGradientComputation::Central_Difference);

        std::cout << vec << std::endl;

        //display the test cube and the calculated vector:

        PointCloud<PointXYZI>::Ptr cloud = PointCloudFunctions::convertToPointCloud(spVmt);
        boost::shared_ptr<PCLVisualizer> viewer = viewMeanGradient(cloud, vec);
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }

}

boost::shared_ptr<PCLVisualizer> viewMeanGradient (PointCloud<PointXYZI>::ConstPtr cloud, cv::Vec3f meanGrad)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem(100.0);
    PointCloudColorHandlerGenericField<PointXYZI> intensity(cloud, "intensity");
    viewer->addPointCloud<PointXYZI> (cloud, intensity, "vmt");
    viewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "vmt");

    PointXYZ stPt; stPt.x = 0.0; stPt.y = 0.0; stPt.z = 0.0;
    PointXYZ endPt; endPt.x = meanGrad[0]; endPt.y = meanGrad[1]; endPt.z = meanGrad[2];

    viewer->addArrow(endPt, stPt, 1.0, 1.0, 1.0, false, "mean gradient");

    viewer->initCameraParameters ();
    return (viewer);
}
