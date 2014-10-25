#include <QString>

#include "interpolatevmt.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


int main(int argc, char *argv[])
{
    QString vmtFile = "/home/emredog/VMT_vid0179_1_enter-leave_78-87Union_181-464-548.pcd";

    int widthOfVmt  = 181;
    int heightOfVmt = 464;
    int depthOfVmt  = 548;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (vmtFile.toStdString(), *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file\n");
        return -1;
    }

    Vmt myVmt(cloud, widthOfVmt, heightOfVmt, depthOfVmt);

    InterpolateVmt vmtInterpolation;

    Vmt newVmt = vmtInterpolation.Interpolate(myVmt);

    pcl::io::savePCDFile("/home/emredog/VMT_vid0179_1_enter-leave_78-87Union_181-464-548-INTERPOL-08.pcd",
                             *(newVmt.getPointCloud_Const()));

}
