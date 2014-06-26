#include "VmtFunctions.h"
#include "PointCloudFunctions.h"
#include <stdio.h>
#include <QTime>
#include <QStringList>


int main(int argc, char *argv[])
{

    if (argc < 2 || argc > 5)
    {
        std::cout << "\n\nUsage: ./VMT_Calculate <videoFolder> <trackFile> [XY-Tolerance=0] [Z-Tolerance=0] \nYou entered " << argc << " values.\n\n";
        return -1;
    }

    QString videoPath = argv[1]; //"/home/emredog/LIRIS-data/training-validation/vid0025";
    QString trackFile = argv[2]; //"/home/emredog/LIRIS-data/training-validation_annotations-with-NO-ACTION-SLIDING_WINDOWS/vid0025_1_unlock-enter_(150-177).track";
    QString outputFolder = "/home/emredog/Documents/output/";

    int downsamplingRate = 1; // --> no downsampling
    int xyTolerance = 0;
    int zTolerance = 0; // --> no depth tolerance

    if (argc == 5)
    {
        bool ok1 = false, ok2  = false;
        xyTolerance = QString(argv[3]).toInt(&ok1);
        zTolerance = QString(argv[4]).toInt(&ok2);
        if (!ok1 || !ok2 || zTolerance < 0 || xyTolerance < 0)
        {
            std::cout << "Z-Tolerance and/or XY-Tolerance should be a positive integer or zero.\n\n";
            return -1;
        }
    }


    VmtFunctions* vmtCore = new VmtFunctions(640, 480, xyTolerance, xyTolerance, zTolerance);

    QTime myTimer;
    myTimer.start();
    cv::SparseMat vmt = vmtCore->ConstructSparseVMT(videoPath, trackFile, downsamplingRate);
    int mSecs = myTimer.elapsed();
    std::cout << "VMT is generated in " << (double)mSecs / 1000.0 << "\n";

    delete vmtCore;
    vmtCore = 0;


    QStringList nameParts = trackFile.split("/");
    QString fileName = nameParts.last();
    fileName.chop(6); //remove the extension
    QString toleranceInfo = QString("X%1Y%2Z%3").arg(xyTolerance).arg(xyTolerance).arg(QString::number(zTolerance).rightJustified(2, '0'));
    if (PointCloudFunctions::saveVmtAsCloud(vmt, outputFolder.append(fileName).append(toleranceInfo).append(".pcd").toStdString()))
        std::cout << "Successfully saved as a point cloud with " << vmt.nzcount() << " points.\n";
    else
        std::cout << "Saving as point cloud have failed.\n";

//    vmtCore->Save3DSparseMatrix(vmt, outputFolder.append(fileName).append(".dat"));
//        std::cout << "Successfully saved as a sparse matrix with " << vmt.size()[0]*vmt.size()[1]*vmt.size()[2] << " lines.\n";

}
