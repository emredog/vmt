#include <QCoreApplication>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/intensity_gradient.h>
#include <boost/thread/thread.hpp>

using namespace pcl;
using namespace pcl::visualization;

#define GRAD_SCALE 1.0
#define NORMAL_SEARCH_RADIUS 8.0
#define INT_GRAD_SEARCH_RADIUS 8.0
#define GRAD_THRESHOLD 0.0
#define NORM_THRESHOLD 0.7

boost::shared_ptr<PCLVisualizer> normalsVis (PointCloud<PointXYZI>::ConstPtr cloud, PointCloud<Normal>::ConstPtr normals);
boost::shared_ptr<PCLVisualizer> gradsVis (PointCloud<PointXYZI>::ConstPtr cloud, PointCloud<IntensityGradient>::ConstPtr gradient, float gradThreshold, float normalThreshold);
boost::shared_ptr<PCLVisualizer> meanGradVis (PointCloud<PointXYZI>::ConstPtr cloud, PointCloud<IntensityGradient>::ConstPtr gradient, float gradThreshold, float normalThreshold);
boost::shared_ptr<PCLVisualizer> gradsVis17 (PointCloud<PointXYZI>::ConstPtr cloud, PointCloud<IntensityGradient>::ConstPtr gradient, float gradThreshold, float normalThreshold);
boost::shared_ptr<PCLVisualizer> viewCloud (PointCloud<PointXYZI>::ConstPtr cloud1);
boost::shared_ptr<PCLVisualizer> viewPointNormals (PointCloud<PointNormal>::ConstPtr cloud);

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    float normalSearchRadius = NORMAL_SEARCH_RADIUS;
    float intensityGradSearchRadius = INT_GRAD_SEARCH_RADIUS;
    float gradThreshold = GRAD_THRESHOLD;
    float normalThreshold = NORM_THRESHOLD;

    PointCloud<PointXYZI>::Ptr cloud (new PointCloud<PointXYZI>);

    if (io::loadPCDFile<PointXYZI> ("/home/emredog/Documents/test.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file /home/emredog/Documents/test.pcd \n");
        return (-1);
    }

    PointXYZI minInPt;
    PointXYZI maxInPt;

    getMinMax3D(*cloud, minInPt, maxInPt);
    float l = minInPt.getVector4fMap()[0];
    l = minInPt.getVector4fMap()[1];
    l = minInPt.getVector4fMap()[2];
    l = maxInPt.getVector4fMap()[0];
    l = maxInPt.getVector4fMap()[1];
    l = maxInPt.getVector4fMap()[2];


    PointXYZ minPt(269.0, 269.0, 269.0);
    PointXYZ maxPt(273.0, 273.0, 273.0);

    //    cloud->

    CropBox<PointXYZI> cropBoxFilter;
    cropBoxFilter.setInputCloud(cloud);
    cropBoxFilter.setMin(minPt.getArray4fMap());
    cropBoxFilter.setMax(maxPt.getArray4fMap());

    PointCloud<PointXYZI>::Ptr cloudOut(new PointCloud<PointXYZI>);
    //    cloudOut->resize(1000);

    cropBoxFilter.filter(*cloudOut);

    getMinMax3D(*cloudOut, minInPt, maxInPt);
    l = minInPt.getVector4fMap()[0];
    l = minInPt.getVector4fMap()[1];
    l = minInPt.getVector4fMap()[2];
    l = maxInPt.getVector4fMap()[0];
    l = maxInPt.getVector4fMap()[1];
    l = maxInPt.getVector4fMap()[2];

    io::savePCDFileASCII("test_output.pcd", *cloudOut);

    //// ESTIMATE SURFACE NORMALS--------------------------------------------------------------------------------------
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
    NormalEstimation<PointXYZI, Normal> norm_est;
    norm_est.setInputCloud (cloudOut);
    //norm_est.setSearchMethod(tree);
    //norm_est.setSearchMethod (tree1); //FIXME!!!
    //norm_est.setKSearch(NEAREST_NEIGHBORS);  //search for the 10 nearest neighbors
    norm_est.setRadiusSearch (normalSearchRadius);
    norm_est.compute (*normals);
    std::cout << "Surface normals are computed with Search Radius: " << normalSearchRadius << std::endl;
    //--------------------------------------------------------------------------------------------------------------------

    //// ESTIMATE INTENSITY GRADIENT---------------------------------------------------------------------------------------
    //The intensity gradient at a given point will be a vector orthogonal to the surface normal
    //and pointing in the direction of the greatest increase in local intensity;
    //the vector's magnitude indicates the rate of intensity change.

    PointCloud<IntensityGradient>::Ptr gradient (new PointCloud<IntensityGradient> ());
    IntensityGradientEstimation<PointXYZI, Normal, IntensityGradient> grad_est;
    grad_est.setInputCloud (cloudOut);
    grad_est.setInputNormals (normals);
    //grad_est.setSearchMethod (boost::make_shared<KdTreeFLANN<PointXYZI> > (false)); //FIXME!!!
    grad_est.setRadiusSearch (intensityGradSearchRadius);
    //grad_est.setKSearch(NEAREST_NEIGHBORS);  //search for the K nearest neighbors
    grad_est.compute (*gradient);
    std::cout << "Intensity Gradients are computed with Search Radius: "<< intensityGradSearchRadius << std::endl;
    //--------------------------------------------------------------------------------------------------------------------

    //// VIEW THE RESULTS---------------------------------------
    boost::shared_ptr<PCLVisualizer> viewer;
    //    viewer = normalsVis(cloudOut, normals);
//    viewer = gradsVis(cloudOut, gradient, gradThreshold, normalThreshold);
//    viewer = gradsVis17(cloudOut, gradient, gradThreshold, normalThreshold);
    viewer = meanGradVis(cloudOut, gradient, gradThreshold, normalThreshold);
    //viewer = viewCloud(lastVmt_BF);
    //viewer = viewCloud(lastVMT_downsampled);
    //viewer = viewPointNormals(mls_points);

    //----------------------------------------------------------

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////
// VIEWER METHODS
////////////////////////////////////////////////////////////////////////////////////////////////
boost::shared_ptr<PCLVisualizer> normalsVis (
        PointCloud<PointXYZI>::ConstPtr cloud, PointCloud<Normal>::ConstPtr normals)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    PointCloudColorHandlerGenericField<PointXYZI> intensity(cloud, "intensity");
    viewer->addPointCloud<PointXYZI> (cloud, intensity, "vmt");
    viewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "vmt");

    viewer->addPointCloudNormals<PointXYZI, Normal> (cloud, normals, 10, 20.0, "normals");
    //viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "normals");

    viewer->initCameraParameters ();
    return (viewer);
}

boost::shared_ptr<PCLVisualizer> gradsVis17 (
        PointCloud<PointXYZI>::ConstPtr cloud, PointCloud<IntensityGradient>::ConstPtr gradient, float gradThreshold, float normalThreshold)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    PointCloudColorHandlerGenericField<PointXYZI> intensity(cloud, "intensity");
    viewer->addPointCloud<PointXYZI> (cloud, intensity, "vmt");
    viewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "vmt");

//    viewer->addPointCloudIntensityGradients(cloud, gradient);

    viewer->initCameraParameters ();
    return (viewer);
}

boost::shared_ptr<PCLVisualizer> meanGradVis (
        PointCloud<PointXYZI>::ConstPtr cloud, PointCloud<IntensityGradient>::ConstPtr gradient, float gradThreshold, float normalThreshold)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    PointCloudColorHandlerGenericField<PointXYZI> intensity(cloud, "intensity");
    viewer->addPointCloud<PointXYZI> (cloud, intensity, "vmt");
    viewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "vmt");

    //FIXME: 1.6'da addPointCloudIntensityGradient yok, o yuzden amelelik yapiyorum:
    cout << "Adding vectors with gradThreshold: " << gradThreshold << ", normalThreshold: " << normalThreshold << endl;
    int counter = 0;
    float sumDx = 0.0, sumDy = 0.0, sumDz = 0.0;

    for (size_t i = 0; i<cloud->points.size(); ++i)
    {
        if (i % 250 == 0)
            cout << "\tindex: " << i << " / counter: " << counter << endl;

        float dx = gradient->points[i].gradient_x;
        float dy = gradient->points[i].gradient_y;
        float dz = gradient->points[i].gradient_z;

//        const PointXYZI &curPoint = cloud->points[i];
//        PointXYZI gradPoint;


//        float normSquare = dx*dx + dy*dy + dz*dz; //comparison is made by squares (for performance)


        //myfile << curPoint.x << ";" << curPoint.y << ";" << curPoint.z << ";"  << curPoint.intensity << ";"
        //	<< dx << ";" << dy << ";" << dz << ";" << sqrt(normSquare) << endl;

        sumDx += dx;
        sumDy += dy;
        sumDz += dz;


//        if (normSquare < normalThreshold*normalThreshold)
//        {
//            gradPoint.x = curPoint.x + (dx)*GRAD_SCALE;
//            gradPoint.y = curPoint.y + (dy)*GRAD_SCALE;
//            gradPoint.z = curPoint.z + (dz)*GRAD_SCALE;
//            //gradPoint.intensity = normSquare //TODO: color the arrow for norm visualization
//            viewer->addArrow(gradPoint, curPoint, 1.0, 1.0, 1.0, false, QString::number(i).toStdString());
//            counter++;
//        }
        counter++;
    }
    //myfile.close();

    Eigen::Vector4f cent;
    compute3DCentroid(*cloud, cent);

    float meanDx = sumDx / (float)counter;
    float meanDy = sumDy / (float)counter;
    float meanDz = sumDz / (float)counter;

    PointXYZI stPoint;
    stPoint.x = cent[0];
    stPoint.y = cent[1];
    stPoint.z = cent[2];

    PointXYZI gradPoint;
    gradPoint.x = stPoint.x + meanDx*GRAD_SCALE;
    gradPoint.y = stPoint.y + meanDy*GRAD_SCALE;
    gradPoint.z = stPoint.z + meanDz*GRAD_SCALE;

    viewer->addArrow(gradPoint, stPoint, 1.0, 1.0, 1.0, false, "mean");

    viewer->initCameraParameters ();
    return (viewer);
}

boost::shared_ptr<PCLVisualizer> gradsVis (
        PointCloud<PointXYZI>::ConstPtr cloud, PointCloud<IntensityGradient>::ConstPtr gradient, float gradThreshold, float normalThreshold)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    PointCloudColorHandlerGenericField<PointXYZI> intensity(cloud, "intensity");
    viewer->addPointCloud<PointXYZI> (cloud, intensity, "vmt");
    viewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "vmt");

    //FIXME: 1.6'da addPointCloudIntensityGradient yok, o yuzden amelelik yapiyorum:
    cout << "Adding vectors with gradThreshold: " << gradThreshold << ", normalThreshold: " << normalThreshold << endl;
    int counter = 0;
    //ofstream myfile;
    //myfile.open ("C:\\Users\\emredog\\Desktop\\Info.txt");
    //myfile << "x;y;z;intensity;dx;dy;dz;norm" << endl;
    for (size_t i = 0; i<cloud->points.size(); ++i)
    {
        if (i % 250 == 0)
            cout << "\tindex: " << i << " / counter: " << counter << endl;

        float dx = gradient->points[i].gradient_x;
        float dy = gradient->points[i].gradient_y;
        float dz = gradient->points[i].gradient_z;

        //if ((abs(dx) < gradThreshold) &&
        //	(abs(dy) < gradThreshold) &&
        //	(abs(dz) < gradThreshold) )
        //{
        //	//cout << "CONTINUE..." << abs(gradient->points[i].gradient_x) << ","
        //	//	 << abs(gradient->points[i].gradient_y) << ","
        //	//	 << abs(gradient->points[i].gradient_z) << endl;
        //	continue; //if none of the components is greater than threshold, proceed to next point
        //}


        const PointXYZI &curPoint = cloud->points[i];
        PointXYZI gradPoint;


        float normSquare = dx*dx + dy*dy + dz*dz; //comparison is made by squares (for performance)


        //myfile << curPoint.x << ";" << curPoint.y << ";" << curPoint.z << ";"  << curPoint.intensity << ";"
        //	<< dx << ";" << dy << ";" << dz << ";" << sqrt(normSquare) << endl;


        if (normSquare < normalThreshold*normalThreshold)
        {
            gradPoint.x = curPoint.x + (dx)*GRAD_SCALE;
            gradPoint.y = curPoint.y + (dy)*GRAD_SCALE;
            gradPoint.z = curPoint.z + (dz)*GRAD_SCALE;
            //gradPoint.intensity = normSquare //TODO: color the arrow for norm visualization
            viewer->addArrow(gradPoint, curPoint, 1.0, 1.0, 1.0, false, QString::number(i).toStdString());
            counter++;
        }
    }
    //myfile.close();

    viewer->initCameraParameters ();
    return (viewer);
}

boost::shared_ptr<PCLVisualizer> viewCloud (
        PointCloud<PointXYZI>::ConstPtr cloud1)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor (0, 0, 0);

    PointCloudColorHandlerGenericField<PointXYZI> intensity(cloud1, "intensity");

    viewer->addPointCloud<PointXYZI> (cloud1, intensity, "vmt");

    viewer->initCameraParameters();
    return (viewer);
}

boost::shared_ptr<PCLVisualizer> viewPointNormals (
        PointCloud<PointNormal>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor (0, 0, 0);

    PointCloudColorHandlerGenericField<PointNormal> intensity(cloud, "intensity");

    viewer->addPointCloud<PointNormal> (cloud, intensity, "vmt");

    viewer->initCameraParameters();
    return (viewer);
}
