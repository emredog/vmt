#include <QString>
#include <QStringList>
#include <QFileInfo>

#include "interpolatevmt.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


int main(int argc, char *argv[])
{
    if (argc != 2 && argc != 3)
    {
        std::cerr << "Usage: ./Interpolate_VMT <PATH/TO/PCD_FILE_TO_INTERPOLATE.pcd> [maxSegmentLength]\n\n";
        return -1;
    }

    QFileInfo vmtInfo(argv[1]);

    bool ok = false;

    int maxSegmentLength = 20; //pixels
    if (argc == 3)
    {
        maxSegmentLength = QString(argv[2]).toInt(&ok);
        if (!ok || maxSegmentLength <= 0)
        {
           std::cerr << "maxSegmentLength should be a positive integer.\n\n";
           return -1;
        }
        ok = false;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (vmtInfo.absoluteFilePath().toStdString(), *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file\n");
        return -1;
    }

    Vmt myVmt(cloud);

    InterpolateVmt vmtInterpolation;

    Vmt newVmt = vmtInterpolation.Interpolate(myVmt, maxSegmentLength);

    pcl::io::savePCDFile(QString("%1/%2-Intpolated-MaxL%3.pcd").arg(vmtInfo.absolutePath())
                                                               .arg(vmtInfo.baseName())
                                                               .arg(maxSegmentLength).toStdString() ,
                             *(newVmt.getPointCloud_Const()));

}
