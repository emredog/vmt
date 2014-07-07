#include <QCoreApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QDebug>
#include <QHash>
#include <QPoint>
#include <QList>

#include "helperFunctions.h"
#include "../AnalyzeAnnotations/boundingbox.h"

#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    QCoreApplication::setApplicationName("Analyze Variance");
    QCoreApplication::setApplicationVersion("0.1");

    QCommandLineParser parser;
    parser.setApplicationDescription("TODO: add application description here.");
    parser.addHelpOption();
    parser.addVersionOption();
    parser.addPositionalArgument("videoFolder", QCoreApplication::translate("main", "Video folder to get depth images."));
    parser.addPositionalArgument("trackFile", QCoreApplication::translate("main", "Track file to read bounding boxes."));

    parser.process(a);

    const QStringList positionalArgs = parser.positionalArguments();

    QString videoPath = positionalArgs[0];
    QString trackFilePath = positionalArgs[1];
    QString outputFolder = "/home/emredog/Documents/output/";

    //read track file
    QFile trackFile(trackFilePath);
    if (!trackFile.open(QFile::ReadOnly))
        return -1;

    QList<BoundingBox> bboxSequence;
    //Parse track file:
    QTextStream reader(&trackFile);
    while (!reader.atEnd())
    {
        QString line = reader.readLine();
        QStringList parts = line.split(" ");

        BoundingBox bbox;
        bbox.frameNr    = parts[0].toInt();
        bbox.x          = parts[1].toInt();
        bbox.y          = parts[2].toInt();
        bbox.width      = parts[3].toInt();
        bbox.height     = parts[4].toInt();

        bboxSequence.append(bbox);
    }

    //get Depth image paths in the folder
    QStringList filters;
    filters << "*.jp2";
    QDir videoDir(videoPath);
    QStringList depthImgFileNames = videoDir.entryList(filters, QDir::Files | QDir::NoDotAndDotDot, QDir::Name);

    QList<cv::Mat> depthImages;

    foreach(BoundingBox bb, bboxSequence)
    {
        QString fileName = depthImgFileNames[bb.frameNr-1];
        if (bb.frameNr >= depthImgFileNames.length() ||
                !fileName.contains(QString::number(bb.frameNr)))
            return -1;

        //Read the file
        cv::Mat depthImg = cv::imread(videoDir.absoluteFilePath(fileName).toStdString(), CV_LOAD_IMAGE_UNCHANGED);

        //Check for invalid input
        if(! depthImg.data )
        {
            qDebug() <<  "Could not open or find the image: " << videoDir.absoluteFilePath(fileName);
            return -1;
        }

        depthImages.append(depthImg);

    }

    QHash<QPoint, QList<unsigned int> > pointToDepthList = parseDepthImages(depthImages);
    int l = pointToDepthList.keys().length();

    qDebug() << "X\t\tY\t\t#ofValues\t\tMean";

    QHashIterator<QPoint, QList<unsigned int> > it(pointToDepthList);
    while (it.hasNext()) {
        it.next();

        QList<unsigned int> depthVals = it.value();
        int sum = 0;
        foreach(int a, depthVals) sum += a;
        double mean = (double)sum / (double)depthVals.length();



        qDebug() << it.key().x() << "\t" << it.key().y() <<
                 "\t" << depthVals.length() << "\t" << mean;
    }



    return 0;
}
