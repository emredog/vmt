#include "PointCloudFunctions.h"


PointCloudFunctions::PointCloudFunctions(void)
{
}


PointCloudFunctions::~PointCloudFunctions(void)
{
}

bool PointCloudFunctions::saveVmtAsCloud(const cv::SparseMat &vmt, std::string fileName)
{
	//TRANSFORM LAST VMT INTO Point Clout
	PointCloud<PointXYZI> cloud;
	
	cloud.resize(vmt.nzcount());
	int i = 0;

    int rows = vmt.size()[1];

    for (cv::SparseMatConstIterator it = vmt.begin(); it != vmt.end(); ++it)
	{
		const cv::SparseMat::Node* n = it.node();
		uchar val = it.value<uchar>();

        int maxZ = vmt.size()[2];

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
	for (cv::SparseMatConstIterator it=vmt.begin(); it != vmt.end(); ++it)
	{
		const cv::SparseMat::Node* n = it.node();
		uchar val = it.value<uchar>();

        cloud->points[i].x = n->idx[0];
        cloud->points[i].y = n->idx[1]; //FIXME 0? 1?
		cloud->points[i].z = n->idx[2];
		cloud->points[i].intensity = static_cast<float>(val);

		i++;	
	}

	return cloud;
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
