#include <QCoreApplication>
#include <QDir>
#include <QTextStream>

#include <iostream>

#include "action.h"
#include "boundingbox.h"

void unionBoundingBoxes(QString trackFilePath);

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    const QString tracksFolder = "/home/emredog/LIRIS-data/test_tracklets_20140424/track";

    QStringList filters;
    filters << "*.track";

    QStringList trackFiles = QDir(tracksFolder).entryList(filters, QDir::Files | QDir::NoDotAndDotDot);

    foreach(QString trackFile, trackFiles)
    {
        unionBoundingBoxes(QDir(tracksFolder).absoluteFilePath(trackFile));
        std::cout << ".";
    }


}

void unionBoundingBoxes(QString trackFilePath)
{
    Action action;
    action.fillBoxesFromTrackFile(trackFilePath);

    int minX = INT_MAX, maxX = INT_MIN, minY = INT_MAX, maxY = INT_MIN;

    for (int i = action.boundingBoxes.keys().first(); i < action.boundingBoxes.keys().last(); i++)
    {
        BoundingBox bbox = action.boundingBoxes.value(i);

        if (minX > bbox.x) minX = bbox.x;
        if (minY > bbox.y) minY = bbox.y;
        if (maxX < bbox.x + bbox.width) maxX = bbox.x + bbox.width;
        if (maxY < bbox.y + bbox.height) maxY = bbox.y + bbox.height;
    }

    QString bboxString = QString(" %1 %2 %3 %4").arg(minX)          //topleft X
                                                .arg(minY)          //topleft Y
                                                .arg(maxX-minX)     //width
                                                .arg(maxY-minY);    //heigt

    trackFilePath = trackFilePath.replace('(', '_');
    trackFilePath = trackFilePath.replace(')', '_');
    trackFilePath.chop(6); //remove extension
    trackFilePath.append("_Union.track");

    QFile outFile(trackFilePath);
    if (!outFile.open(QFile::WriteOnly))
        return;

    QTextStream out(&outFile);
    for (int i = action.boundingBoxes.keys().first(); i <= action.boundingBoxes.keys().last(); i++)
        out << action.boundingBoxes.value(i).frameNr << bboxString << endl;

    out.flush();
    outFile.close();
}
