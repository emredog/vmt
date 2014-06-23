#include "VmtFunctions.h"
#include "PointCloudFunctions.h"
#include <stdio.h>
#include <QTime>
#include <QStringList>


int main(int argc, char *argv[])
{

    if (argc != 3 && argc != 4)
    {
        std::cout << "\n\nUsage: ./VMT_Calculate <videoFolder> <trackFile> [downsamplingRate=0]\nYou entered " << argc << " values.\n\n";
        return -1;
    }

    QString videoPath = argv[1]; //"/home/emredog/LIRIS-data/training-validation/vid0025";
    QString trackFile = argv[2]; //"/home/emredog/LIRIS-data/training-validation_annotations-with-NO-ACTION-SLIDING_WINDOWS/vid0025_1_unlock-enter_(150-177).track";
    QString outputFolder = "/home/emredog/Documents/output/";

    int downsamplingRate = 1; // --> no downsampling

    if (argc == 4)
    {
        bool ok = false;
        downsamplingRate = QString(argv[3]).toInt(&ok);
        if (!ok || downsamplingRate <= 0)
            std::cout << "Downsampling rate should be a positive integer.\n\n";
    }


    VmtFunctions* vmtCore = new VmtFunctions();

    QTime myTimer;
    myTimer.start();
    cv::SparseMat vmt = vmtCore->GenerateSparseVMT(videoPath, trackFile, downsamplingRate);
    int mSecs = myTimer.elapsed();
    std::cout << "VMT is generated in " << (double)mSecs / 1000.0 << "\n";

    delete vmtCore;
    vmtCore = 0;


    QStringList nameParts = trackFile.split("/");
    QString fileName = nameParts.last();
    fileName.chop(6); //remove the extension
    if (PointCloudFunctions::saveVmtAsCloud(vmt, outputFolder.append(fileName).append(".pcd").toStdString()))
        std::cout << "Successfully saved as a point cloud with " << vmt.nzcount() << " points.\n";
    else
        std::cout << "Saving as point cloud have failed.\n";
}
