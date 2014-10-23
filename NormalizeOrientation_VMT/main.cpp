#include <QCoreApplication>

#include "opencv/vmt.h"
#include "orientationnormalizer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

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

    OrientationNormalizer orNor;

    QList<float> angles;
    angles << 0.0; //around X
    angles << 2.0; //around Y
    angles << 0.0; //around Z
    Vmt rotatedVmt = orNor.rotateVmt(angles, myVmt);

    pcl::io::savePCDFile("/home/emredog/VMT_vid0179_1_enter-leave_78-87Union_181-464-548-ROTATED-Y.pcd",
                         *(rotatedVmt.getPointCloud_Const()));

}
