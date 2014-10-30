#include <QFileInfo>
#include <QString>
#include <QStringList>

#include "opencv/vmt.h"
#include "orientationnormalizer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char *argv[])
{    
    if (argc != 5)
    {
        std::cerr << "Usage: ./NormalizeOrientation_VMT <PATH/TO/PCD_FILE_TO_INTERPOLATE.pcd> <radiansAroundXAxis> <radiansAroundYAxis> <radiansAroundZAxis>\n\n";
        return -1;
    }

    //read VMT file and dimensions
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

    //read rotation angles:
    float alpha = 0.0, beta = 0.0, theta = 0.0;
    alpha = QString(argv[2]).toFloat(&ok);
    if (!ok){std::cerr << argv[2] << " is not a valid number!\n\n"; return -1;}
    beta = QString(argv[3]).toFloat(&ok);
    if (!ok){std::cerr << argv[3] << " is not a valid number!\n\n"; return -1;}
    theta = QString(argv[4]).toFloat(&ok);
    if (!ok){std::cerr << argv[4] << " is not a valid number!\n\n"; return -1;}



    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (vmtInfo.absoluteFilePath().toStdString(), *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file\n");
        return -1;
    }

    Vmt myVmt(cloud, widthOfVmt, heightOfVmt, depthOfVmt);

    QList<float> angles;
    angles << alpha; //around X
    angles << beta; //around Y
    angles << theta; //around Z
    Vmt rotatedVmt = OrientationNormalizer::rotateVmt(angles, myVmt);

    pcl::io::savePCDFile(QString("%1/%2-Rotated-A%3B%4T%5.pcd").arg(vmtInfo.absolutePath())
                         .arg(vmtInfo.baseName())
                         .arg(alpha)
                         .arg(beta)
                         .arg(theta).toStdString(),
                         *(rotatedVmt.getPointCloud_Const()));

}
