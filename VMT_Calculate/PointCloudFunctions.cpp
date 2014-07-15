#include "PointCloudFunctions.h"


PointCloudFunctions::PointCloudFunctions(void)
{
}


PointCloudFunctions::~PointCloudFunctions(void)
{
}

cv::SparseMat PointCloudFunctions::loadVmtFromPCD(string fileName)
{
    PointCloud<PointXYZI>::Ptr cloud (new PointCloud<PointXYZI>);

    if (pcl::io::loadPCDFile<PointXYZI> (fileName, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return cv::SparseMat();
    }

    int dim = DIM;
    int sizes[] = {SIZE_X, SIZE_Y, SIZE_Z}; //FIXME: too much or too less of Z

    return convertToSparseMat(cloud, dim, sizes);
}

bool PointCloudFunctions::saveVmtAsCloud(const cv::SparseMat &vmt, std::string fileName)
{
    //TRANSFORM VMT INTO Point Clout
    PointCloud<PointXYZI> cloud;

    cloud.resize(vmt.nzcount());
    int i = 0;

    int rows = vmt.size()[1];
    int maxZ = vmt.size()[2];

    for (cv::SparseMatConstIterator it = vmt.begin(); it != vmt.end(); ++it)
    {
        const cv::SparseMat::Node* n = it.node();
        uchar val = it.value<uchar>();

        cloud.points[i].x = n->idx[0];
        cloud.points[i].y = rows - n->idx[1];
        cloud.points[i].z = maxZ - n->idx[2];
        cloud.points[i].intensity = static_cast<float>(val);

        i++;
    }

    if (pcl::io::savePCDFileASCII (fileName, cloud) >= 0) //FIXME: what is the return value? it's not mentioned in http://docs.pointclouds.org/1.6.0/group__io.html#ga5e406a5854fa8ad026cad85158fef266
        return true;
    else
        return false;
}

PointCloud<PointXYZI>::Ptr PointCloudFunctions::convertToPointCloud(const cv::SparseMat &vmt)
{
    PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI> ());
    cloud->resize(vmt.nzcount());
    int i = 0;

    int rows = vmt.size()[1];
    int maxZ = vmt.size()[2];

    for (cv::SparseMatConstIterator it=vmt.begin(); it != vmt.end(); ++it)
    {
        const cv::SparseMat::Node* n = it.node();
        uchar val = it.value<uchar>();

        cloud->points[i].x = n->idx[0];
        cloud->points[i].y = rows - n->idx[1];
        cloud->points[i].z = maxZ - n->idx[2];
        cloud->points[i].intensity = static_cast<float>(val);

        i++;
    }

    return cloud;
}

//PointCloud<PointXYZI>::Ptr PointCloudFunctions::convertToOrganizedPointCloud(const cv::SparseMat &vmt)
//{
//    int rows = vmt.size()[1];
//    int cols = vmt.size()[0];
//    int maxZ = vmt.size()[2];
//    PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI> (cols, rows));

//    for (cv::SparseMatConstIterator it=vmt.begin(); it != vmt.end(); ++it)
//    {
//        const cv::SparseMat::Node* n = it.node();
//        uchar val = it.value<uchar>();

//        cloud->at(n->idx[0], rows - n->idx[1], maxZ - n->idx[2])  ;
//        cloud->points[i].y = ;
//        cloud->points[i].z = ;
//        cloud->points[i].intensity = static_cast<float>(val);

//        i++;
//    }

//    return cloud;
//}

cv::SparseMat PointCloudFunctions::convertToSparseMat(PointCloud<PointXYZI>::Ptr ptCloud, int dim, const int sizes[])
{
    PointCloud<PointXYZI>::const_iterator it;
    cv::SparseMat sparseMat(dim, sizes, CV_16UC1);

    int rows = sizes[1];
    int maxZ = sizes[2];

    for (it = ptCloud->begin(); it < ptCloud->end(); it++)
    {
        uchar val = static_cast<uchar>(it->intensity);
        if (val > 0)
            sparseMat.ref<uchar>(it->x, rows - it->y, maxZ - it->z) = val;
    }

    return sparseMat;
}

cv::SparseMat PointCloudFunctions::statisticalOutlierRemoval(const cv::SparseMat &vmt, int meanK, double stdDevMulThreshold)
{
    PointCloud<PointXYZI>::Ptr cloud = convertToPointCloud(vmt);
    PointCloud<PointXYZI>::Ptr cloud_filtered (new PointCloud<PointXYZI>);

    // Create the filtering object
    StatisticalOutlierRemoval<PointXYZI> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (stdDevMulThreshold);
    sor.filter (*cloud_filtered);

    cloud->clear();
    cloud.reset();

    return convertToSparseMat(cloud_filtered, vmt.dims(), vmt.size());
}

bool PointCloudFunctions::statisticalOutlierRemovalAndSave(const cv::SparseMat &vmt, string fileName, int meanK, double stdDevMulThreshold)
{
    //    "Our sparse outlier removal is based on the computation of the distribution of point to neighbors distances in the input dataset.
    //    For each point, we compute the mean distance from it to all its neighbors. By assuming that the resulted distribution is
    //    Gaussian with a mean and a standard deviation, all points whose mean distances are outside an interval defined by the global
    //    distances mean and standard deviation can be considered as outliers and trimmed from the dataset."

    PointCloud<PointXYZI>::Ptr cloud = convertToPointCloud(vmt);
    PointCloud<PointXYZI>::Ptr cloud_filtered (new PointCloud<PointXYZI>);

    // Create the filtering object
    StatisticalOutlierRemoval<PointXYZI> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (stdDevMulThreshold);
    sor.filter (*cloud_filtered);

    cloud->clear();
    cloud.reset();

    bool resultRet = (pcl::io::savePCDFileASCII (fileName, *cloud_filtered) >= 0); //FIXME: what is the return value? it's not mentioned in http://docs.pointclouds.org/1.6.0/group__io.html#ga5e406a5854fa8ad026cad85158fef266

    cloud_filtered->clear();
    cloud_filtered.reset();

    return resultRet;
}

bool PointCloudFunctions::radiusOutlierRemovalAndSave(const cv::SparseMat &vmt, string fileName, double radius, int minNeighbors)
{
    //    "The user specifies a number of neighbors which every indice must have within a specified radius to remain in the PointCloud."

    PointCloud<PointXYZI>::Ptr cloud = convertToPointCloud(vmt);
    PointCloud<PointXYZI>::Ptr cloud_filtered (new PointCloud<PointXYZI>);

    // Create the filtering object
    RadiusOutlierRemoval <PointXYZI> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch (radius);
    ror.setMinNeighborsInRadius (minNeighbors);
    ror.setNegative (true);
    ror.filter (*cloud_filtered);

    cloud->clear();
    cloud.reset();

    bool resultRet = (pcl::io::savePCDFileASCII (fileName, *cloud_filtered) >= 0); //FIXME: what is the return value? it's not mentioned in http://docs.pointclouds.org/1.6.0/group__io.html#ga5e406a5854fa8ad026cad85158fef266

    cloud_filtered->clear();
    cloud_filtered.reset();

    return resultRet;
}

PointCloud<PointXYZI>::Ptr PointCloudFunctions::downSampleCloud(pcl::PointCloud<PointXYZI>::Ptr inputCloud, float leafSize, bool save, string fileNameToSave)
{
    PointCloud<PointXYZI>::Ptr downsampled(new PointCloud<PointXYZI> ());
    VoxelGrid<PointXYZI> sor;
    sor.setInputCloud (inputCloud);
    sor.setFilterLimits(0, 2000);
    sor.setLeafSize (leafSize, leafSize, leafSize);
    sor.filter (*downsampled);

    if (save)
    {
        savePCDFileASCII (fileNameToSave, *downsampled);
    }

    return downsampled;
}

bool PointCloudFunctions::saveCloud(const PointCloud<PointXYZI>::Ptr cloud, const string &fileName)
{
    if (savePCDFileASCII (fileName, *cloud) >= 0) //FIXME: what is the return value? it's not mentioned in http://docs.pointclouds.org/1.6.0/group__io.html#ga5e406a5854fa8ad026cad85158fef266
        return true;
    else
        return false;
}
