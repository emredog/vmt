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
    QStringList nameParts = vmtInfo.baseName().split("_");

//    QStringList dims = nameParts.last().split("-");

//    if (dims.length() != 3)
//    {
//        std::cerr << "File name does not contain dimension info. Good example is: /home/emredog/VMT_vid0179_1_enter-leave_78-87Union_181-464-548.pcd\n\n";
//        return -1;
//    }

    bool ok = false;

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

    Vmt myVmt(cloud);

    QList<float> angles;
    angles << alpha; //around X
    angles << beta; //around Y
    angles << theta; //around Z
    Vmt rotatedVmt = OrientationNormalizer::rotateVmt(angles, myVmt);

    //get min & max points
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr rotatedCloud = rotatedVmt.getPointCloud_Const();
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D(*rotatedCloud, minPt, maxPt);
    //set the offsets and shift the rotated cloud in the positive quadrant
    QList<float> offsets;
    offsets << -minPt.x;
    offsets << -minPt.y;
    offsets << -minPt.z;
    Vmt translatedVmt = OrientationNormalizer::translateVmt(offsets, rotatedVmt);

    nameParts.removeLast();


    pcl::io::savePCDFile(QString("%1/%2_Rotated.pcd").arg(vmtInfo.absolutePath())
                         .arg(nameParts.join("_"))
                         .toStdString(),
                         *(translatedVmt.getPointCloud_Const()));

}
