#include "VmtFunctions.h"
#include "PointCloudFunctions.h"

#include <stdio.h>
#include <QTime>
#include <QStringList>
#include <QCoreApplication>
#include <QCommandLineParser>


int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName("VMT Calculator");
    QCoreApplication::setApplicationVersion("0.1");

    QCommandLineParser parser;
    parser.setApplicationDescription("TODO: add application description here.");
    parser.addHelpOption();
    parser.addVersionOption();
    parser.addPositionalArgument("videoFolder", QCoreApplication::translate("main", "Video folder to get depth images."));
    parser.addPositionalArgument("trackFile", QCoreApplication::translate("main", "Track file to read bounding boxes."));

    //Verbose option
    QCommandLineOption optionVerbose(QStringList() << "v" << "verbose", "Show debugging output.");
    parser.addOption(optionVerbose);

    //Save volume object option
    QCommandLineOption optionSaveVolumeObject(QStringList() << "sVolObj" << "saveVolumeObject", "Save intermediate volume objects to output folder.");
    parser.addOption(optionSaveVolumeObject);

    //Save delta option
    QCommandLineOption optionSaveDelta(QStringList() << "sDelta" << "saveDelta", "Save intermediate volume objects differences to output folder.");
    parser.addOption(optionSaveDelta);

    //Save VMT option
    QCommandLineOption optionSaveVMT(QStringList() << "sVMT" << "saveVMT", "Save intermediate VMTs to output folder.");
    parser.addOption(optionSaveVMT);

//    //Tolerance in XY
//    QCommandLineOption optionToleranceXY(QStringList() << "tolXY" << "toleranceXY", "Subtraction tolerance in X-Y axis", "toleranceXZ", "0");
//    parser.addOption(optionToleranceXY);

//    //Tolerance in Z
//    QCommandLineOption optionToleranceZ(QStringList() << "tolZ" << "toleranceZ", "Subtraction tolerance in Z axis", "toleranceZ", "0");
//    parser.addOption(optionToleranceZ);

    //Track point X
    QCommandLineOption optionTrackX("trackX", "X coordinate of tracked point", "trackX", "-1");
    parser.addOption(optionTrackX);

    //Track point, Y
    QCommandLineOption optionTrackY("trackY", "Y coordinate of tracked point", "trackY", "-1");
    parser.addOption(optionTrackY);

    //downsampling rate
    QCommandLineOption optionDownsample(QStringList() << "downsample" << "downsample", "Downsample resulting VMT (a rate of 2 results in a downsampling of 1/4)", "downsample", "1");
    parser.addOption(optionDownsample);


    //... TODO: add more options
    // save vmt, volobj, vmt,
    // output folder...

    parser.process(app);

    const QStringList positionalArgs = parser.positionalArguments();

    QString videoPath = positionalArgs[0];
    QString trackFile = positionalArgs[1];
    QString outputFolder = "/home/emredog/Documents/output/";

    bool ok = false;

    // fetch bool options
    bool isVerbose = parser.isSet(optionVerbose); //TODO
    bool saveVolObj = parser.isSet(optionSaveVolumeObject);
    bool saveDelta = parser.isSet(optionSaveDelta);
    bool saveVmt = parser.isSet(optionSaveVMT);

//    // fetch XY tolerance
//    QString xyTolStr = parser.value(optionToleranceXY);
//    int xyTolerance = xyTolStr.toInt(&ok);
//    if (!ok) xyTolerance = 0;

//    // fetch Z tolerance
//    QString zTolStr = parser.value(optionToleranceZ);
//    int zTolerance = zTolStr.toInt(&ok);
//    if (!ok) zTolerance = 0;

    // fetch track point
    QString trackXStr = parser.value(optionTrackX);
    QString trackYStr = parser.value(optionTrackY);

    int trackX = -1, trackY = -1;
    trackX = trackXStr.toInt(&ok);
    if (!ok) trackX = -1;
    trackY = trackYStr.toInt(&ok);
    if (!ok) trackY = -1;

    // fetch downsampling
    QString downsampleStr = parser.value(optionDownsample);
    int downsamplingRate = downsampleStr.toInt(&ok);
    if (!ok) downsamplingRate = 1;

    //----------------------------------------------------------------------------------------------
    // Real deal starts here...
    //----------------------------------------------------------------------------------------------

    VmtFunctions* vmtCore = new VmtFunctions(640, 480);
    vmtCore->setSavedObjects(saveVolObj, saveDelta, saveVmt);
    vmtCore->setDownsampleRate(downsamplingRate);
    if (trackX >= 0 && trackY >= 0)
        vmtCore->setTrackPoint(trackX, trackY);




    QTime myTimer;
    myTimer.start();
    cv::SparseMat vmt = vmtCore->constructSparseVMT(videoPath, trackFile);
    int mSecs = myTimer.elapsed();
    std::cout << "VMT is generated in " << (double)mSecs / 1000.0 << "\n";

    delete vmtCore;
    vmtCore = 0;

    if (vmt.nzcount() <= 0)
        return 0;


    QStringList nameParts = trackFile.split("/");
    QString fileName = nameParts.last();
    fileName.chop(6); //remove the extension

    if (PointCloudFunctions::saveVmtAsCloud(vmt, QString("%1%2%3").arg(outputFolder).arg(fileName).arg(".pcd").toStdString()))
        std::cout << "Successfully saved as a point cloud with " << vmt.nzcount() << " points.\n";
    else
        std::cout << "Saving as point cloud have failed.\n";

    if (PointCloudFunctions::statisticalOutlierRemovalAndSave(vmt, QString("%1%2%3").arg(outputFolder).arg(fileName).arg("_StatisticalFilter.pcd").toStdString()))
        std::cout << "Successfully filtered (Statistical outlier removal) & saved point cloud\n";
    else
        std::cout << "Saving as point cloud have failed.\n";

    if (PointCloudFunctions::radiusOutlierRemovalAndSave(vmt, QString("%1%2%3").arg(outputFolder).arg(fileName).arg("_RadiusFilter.pcd").toStdString()))
        std::cout << "Successfully filtered (Radius outlier removal) & saved point cloud\n";
    else
        std::cout << "Saving as point cloud have failed.\n";

    //    vmtCore->Save3DSparseMatrix(vmt, outputFolder.append(fileName).append(".dat"));
    //        std::cout << "Successfully saved as a sparse matrix with " << vmt.size()[0]*vmt.size()[1]*vmt.size()[2] << " lines.\n";

}
