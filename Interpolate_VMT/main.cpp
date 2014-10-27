#include <QString>
#include <QStringList>
#include <QFileInfo>

#include "interpolatevmt.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: ./Interpolate_VMT <PATH/TO/PCD_FILE_TO_INTERPOLATE.pcd>\n\n";
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

    Vmt newVmt = vmtInterpolation.Interpolate(myVmt);

    pcl::io::savePCDFile(QString("%1/%2-Intpolated.pcd").arg(vmtInfo.absolutePath()).arg(vmtInfo.baseName()).toStdString() ,
                             *(newVmt.getPointCloud_Const()));

}
