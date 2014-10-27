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

    QStringList dims = vmtInfo.baseName().split("_").last().split("-");

    if (dims.length() != 3)
    {
        std::cerr << "File name does not contain dimension info. Good example is: /home/emredog/VMT_vid0179_1_enter-leave_78-87Union_181-464-548.pcd\n\n";
        return -1;
    }

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

    int widthOfVmt  = dims[0].toInt(&ok);
    if (!ok){std::cerr << dims[0].toStdString() << " is not a valid number!\n\n"; return -1;}
    int heightOfVmt = dims[1].toInt(&ok);
    if (!ok){std::cerr << dims[1].toStdString() << " is not a valid number!\n\n"; return -1;}
    int depthOfVmt  = dims[2].toInt(&ok);
    if (!ok){std::cerr << dims[2].toStdString() << " is not a valid number!\n\n"; return -1;}

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (vmtInfo.absoluteFilePath().toStdString(), *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file\n");
        return -1;
    }

    Vmt myVmt(cloud, widthOfVmt, heightOfVmt, depthOfVmt);

    InterpolateVmt vmtInterpolation;

    Vmt newVmt = vmtInterpolation.Interpolate(myVmt, maxSegmentLength);

    pcl::io::savePCDFile(QString("%1/%2-Intpolated-MaxL%3.pcd").arg(vmtInfo.absolutePath())
                                                               .arg(vmtInfo.baseName())
                                                               .arg(maxSegmentLength).toStdString() ,
                             *(newVmt.getPointCloud_Const()));

}
