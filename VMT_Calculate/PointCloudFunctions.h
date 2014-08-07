#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>


#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)

#include <string>

#define DIM 3
#define SIZE_X 640
#define SIZE_Y 480
#define SIZE_Z 2000



using namespace pcl;
using namespace pcl::io;

using namespace std;

class PointCloudFunctions
{
private:
	PointCloudFunctions(void);
	~PointCloudFunctions(void);
public:
    static cv::SparseMat loadVmtFromPCD(std::string fileName, int sizeX = SIZE_X, int sizeY = SIZE_Y, int sizeZ = SIZE_Z);
	static bool saveVmtAsCloud(const cv::SparseMat &vmt, std::string fileName);
	static PointCloud<PointXYZI>::Ptr downSampleCloud(pcl::PointCloud<PointXYZI>::Ptr inputCloud, 
		float leafSize, bool save, string fileNameToSave);

    static bool saveCloud(const PointCloud<PointXYZI>::Ptr cloud, const string &fileName);

	static PointCloud<PointXYZI>::Ptr convertToPointCloud(const cv::SparseMat &vmt);
    static PointCloud<PointXYZI>::Ptr convertToOrganizedPointCloud(const cv::SparseMat &vmt);
    static cv::SparseMat convertToSparseMat(PointCloud<PointXYZI>::Ptr ptCloud, int dim, const int sizes[]);

    static cv::SparseMat statisticalOutlierRemoval(const cv::SparseMat &vmt, int meanK = 50, double stdDevMulThreshold = 1.0);
    static bool statisticalOutlierRemovalAndSave(const cv::SparseMat &vmt, std::string fileName, int meanK = 50, double stdDevMulThreshold = 1.0);
    static bool radiusOutlierRemovalAndSave(const cv::SparseMat &vmt, std::string fileName, double radius = 0.1, int minNeighbors = 5);
};

