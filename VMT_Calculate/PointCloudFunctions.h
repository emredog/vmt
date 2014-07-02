#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/common.h>

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)

#include <string>

using namespace pcl;
using namespace pcl::io;

using namespace std;

class PointCloudFunctions
{
private:
	PointCloudFunctions(void);
	~PointCloudFunctions(void);
public:
	static bool saveVmtAsCloud(const cv::SparseMat &vmt, std::string fileName);
	static PointCloud<PointXYZI>::Ptr downSampleCloud(pcl::PointCloud<PointXYZI>::Ptr inputCloud, 
		float leafSize, bool save, string fileNameToSave);
	static PointCloud<PointXYZI>::Ptr convertToPointCloud(const cv::SparseMat &vmt);

    static bool statisticalOutlierRemovalAndSave(const cv::SparseMat &vmt, std::string fileName, int meanK = 50, double stdDevMulThreshold = 1.0);
    static bool radiusOutlierRemovalAndSave(const cv::SparseMat &vmt, std::string fileName, double radius = 0.1, int minNeighbors = 5);
};

