#include <QCoreApplication>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/intensity_gradient.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/moment_invariants.h>
#include <pcl/features/boundary.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/intensity_spin.h>
#include <pcl/features/rift.h>
#include <pcl/filters/bilateral.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>

#include <QDir>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;
using namespace std;

#define GRAD_THRESHOLD 0.0
#define NORM_THRESHOLD 0.7
#define SCALE 20.0
//#define SIGMA_S 40 //Increasing the spatial parameter SIGMA_S smooths larger features.
//#define SIGMA_R 200 //As SIGMA_R increases, the bilateral filter becomes closer to Gaussian blur because the range Gaussian is flatter i.e., almost a constant over the intensity interval covered by the image.
#define NORMAL_SEARCH_RADIUS 100.0
#define INT_GRAD_SEARCH_RADIUS 120.0


boost::shared_ptr<PCLVisualizer> normalsVis (PointCloud<PointXYZI>::ConstPtr cloud, PointCloud<Normal>::ConstPtr normals);
boost::shared_ptr<PCLVisualizer> gradsVis (PointCloud<PointXYZI>::ConstPtr cloud, PointCloud<IntensityGradient>::ConstPtr gradient, float gradThreshold, float normalThreshold);
boost::shared_ptr<PCLVisualizer> viewCloud (PointCloud<PointXYZI>::ConstPtr cloud1);
boost::shared_ptr<PCLVisualizer> viewPointNormals (PointCloud<PointNormal>::ConstPtr cloud);

PointCloud<PointXYZI>::Ptr downSampleCloud(PointCloud<PointXYZI>::Ptr inputCloud, float leafSize, bool save = false, string fileNameToSave = "");
PointCloud<PointXYZI>::Ptr applyBilateralFilter(PointCloud<PointXYZI>::Ptr inputCloud, float sigmaS, float sigmaR, bool save = false, string fileNameToSave = "");

void getDir(string& d, vector<string> & f);

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    //float sigma_s = SIGMA_S, sigma_r = SIGMA_R;

    float normalSearchRadius = NORMAL_SEARCH_RADIUS;
    float intensityGradSearchRadius = INT_GRAD_SEARCH_RADIUS;
    float gradThreshold = GRAD_THRESHOLD;
    float normalThreshold = NORM_THRESHOLD;

    int fileIndex = 40;

    if (argc == 3)
    {
        intensityGradSearchRadius = (float)atoi(argv[1]);
        fileIndex = atoi(argv[2]);
    }

    std::vector<string> files;
    string folder = "/home/emredog/Documents/output/";
    getDir(folder, files);


    string fileName = files.at(fileIndex);

    PointCloud<PointXYZI>::Ptr vmt (new PointCloud<PointXYZI>);

    clock_t start = clock();
    if (loadPCDFile<PointXYZI> (folder + fileName, *vmt) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file:");
        PCL_ERROR(fileName.c_str());
        return (-1);
    }
    clock_t stop = clock();

    std::cout << "Loaded "
              << vmt->width * vmt->height
              << " data points from " << fileName << " " << (float)(stop-start) / (float)CLOCKS_PER_SEC << " seconds."
              << std::endl;

    PointCloud<PointXYZI>::Ptr temp = downSampleCloud(vmt, 10.f);
    vmt->clear();

    vmt = temp;


#pragma region ESTIMATE SURFACE NORMALS
    ////KdTreeFLANN<PointT>::Ptr tree (new KdTreeFLANN<PointXYZI> );
    ////tree->setInputCloud(lastVMT);
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
    NormalEstimation<PointXYZI, Normal> norm_est;
    norm_est.setInputCloud (vmt);
    //norm_est.setSearchMethod(tree);
    //norm_est.setSearchMethod (tree1); //FIXME!!!
    //norm_est.setKSearch(NEAREST_NEIGHBORS);  //search for the 10 nearest neighbors
    norm_est.setRadiusSearch (normalSearchRadius);
    start = clock();
    norm_est.compute (*normals);
    stop = clock();
    std::cout << "Surface normals are computed with Search Radius: " << normalSearchRadius << " in " << (float)(stop-start) / (float)CLOCKS_PER_SEC << " seconds."
              << std::endl;
#pragma endregion

#pragma region ESTIMATE INTENSITY GRADIENT
    //The intensity gradient at a given point will be a vector orthogonal to the surface normal
    //and pointing in the direction of the greatest increase in local intensity;
    //the vector's magnitude indicates the rate of intensity change.

    PointCloud<IntensityGradient>::Ptr gradient (new PointCloud<IntensityGradient> ());
    IntensityGradientEstimation<PointXYZI, Normal, IntensityGradient> grad_est;
    grad_est.setInputCloud (vmt);
    grad_est.setInputNormals (normals);
    //grad_est.setSearchMethod (boost::make_shared<KdTreeFLANN<PointXYZI> > (false)); //FIXME!!!
    grad_est.setRadiusSearch (intensityGradSearchRadius);
    //grad_est.setKSearch(NEAREST_NEIGHBORS);  //search for the K nearest neighbors
    start = clock();
    grad_est.compute (*gradient);
    stop = clock();
    std::cout << "Intensity Gradients are computed with Search Radius: "<< intensityGradSearchRadius << "in " << (float)(stop-start) / (float)CLOCKS_PER_SEC << " seconds."
              << std::endl;
#pragma endregion



    //// VIEW THE RESULTS---------------------------------------
    boost::shared_ptr<PCLVisualizer> viewer;
    //viewer = normalsVis(lastVMT, normals);
    viewer = gradsVis(vmt, gradient, gradThreshold, normalThreshold);
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


    //// CLEAN-UP----------------------------
    vmt->clear();
    normals->clear();
    //gradient->clear();
    //lastVmt_BF->clear();

    return a.exec();
}

#pragma region FUNCTIONS
PointCloud<PointXYZI>::Ptr downSampleCloud(PointCloud<PointXYZI>::Ptr inputCloud, float leafSize, bool save, string fileNameToSave)
{
    PointCloud<PointXYZI>::Ptr downsampled(new PointCloud<PointXYZI> ());
    VoxelGrid<PointXYZI> sor;
    sor.setInputCloud (inputCloud);
    sor.setFilterLimits(0, 2000);
    sor.setLeafSize (leafSize, leafSize, leafSize);
    clock_t start = clock();
    sor.filter (*downsampled);
    clock_t stop = clock();
    std::cout << "Downsampled in " << (float)(stop-start) / (float)CLOCKS_PER_SEC << " seconds." << std::endl;
    std::cout << "\tPointCloud after downsampling: " << downsampled->width * downsampled->height << " data points (" << pcl::getFieldsList (*downsampled) << ")." << endl;

    if (save)
    {
        savePCDFileASCII (fileNameToSave, *downsampled);
    }

    return downsampled;
}

PointCloud<PointXYZI>::Ptr applyBilateralFilter(PointCloud<PointXYZI>::Ptr inputCloud, float sigmaS, float sigmaR, bool save, string fileNameToSave)
{
    PointCloud<PointXYZI>::Ptr lastVmt_BF (new PointCloud<PointXYZI>);
    BilateralFilter<PointXYZI> bf;
    bf.setInputCloud(inputCloud);
    bf.setHalfSize(sigmaS);
    bf.setStdDev(sigmaR);
    clock_t start = clock();
    bf.filter(*lastVmt_BF);
    clock_t stop = clock();
    std::cout << "Bilateral filter applied (sigma_s: " << sigmaS << ", sigma_r: " << sigmaR <<" in " << (float)(stop-start) / (float)CLOCKS_PER_SEC << " seconds."
              << std::endl;

    if (save)
    {
        savePCDFileASCII (fileNameToSave, *lastVmt_BF);
    }

    return lastVmt_BF;
}
#pragma endregion

#pragma region VIEWER METHODS
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

    viewer->addPointCloudNormals<PointXYZI, Normal> (cloud, normals, 10, SCALE, "normals");
    //viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "normals");

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
    for (size_t i = 0; i<cloud->points.size(); i+=20)
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


        //float normSquare = dx*dx + dy*dy + dz*dz; //comparison is made by squares (for performance)


        //myfile << curPoint.x << ";" << curPoint.y << ";" << curPoint.z << ";"  << curPoint.intensity << ";"
        //	<< dx << ";" << dy << ";" << dz << ";" << sqrt(normSquare) << endl;


        //if (normSquare < normalThreshold*normalThreshold)
        {
            gradPoint.x = curPoint.x + (dx)*SCALE;
            gradPoint.y = curPoint.y + (dy)*SCALE;
            gradPoint.z = curPoint.z + (dz)*SCALE;
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

void getDir(string& d, vector<string> & f)
{
    QDir dir(QString::fromStdString(d));
    QStringList entryList = dir.entryList(QDir::Files);
    foreach(QString s, entryList) f.push_back(s.toStdString());
}
#pragma endregion

