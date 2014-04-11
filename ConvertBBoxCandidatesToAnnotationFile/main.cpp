#include <QCoreApplication>
#include <QFile>
#include <QXmlStreamWriter>
#include <QPair>
#include <QDir>
#include <QTextStream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

typedef QPair<Rect, float> RectWithScore;

QList<RectWithScore> getAllBoundingBoxes(QString bboxFilePath);

int main(/*int argc, char *argv[]*/)
{
    //QCoreApplication a(argc, argv);

    const QDir testFolder("/home/emredog/LIRIS-data/test/");
    const QDir bboxMainFolder("/home/emredog/Documents/ADSC_NUS_Harl_result_code_v2/HumanDetectionResults/");

    QStringList filters;

    //get video folders:
    //QStringList vidFolders = testFolder.entryList(filters, QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    QStringList bboxFolders = bboxMainFolder.entryList(filters, QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);

    filters.clear();
    filters << "*.txt";
    //for each video
    for (int i=0; i<bboxFolders.length(); i++)
    {
        QFile annotationFile(bboxFolders[i].left(7).append("_withScores").append(".xml"));
        if (!annotationFile.open(QIODevice::Truncate | QIODevice::WriteOnly))
                continue;

        //get bounding box files
        QDir bboxFolder(bboxMainFolder.absoluteFilePath(bboxFolders[i]));
        QStringList bboxFilesForAVideo = bboxFolder.entryList(filters, QDir::Files, QDir::Name);

        QXmlStreamWriter xmlWriter(&annotationFile);

        xmlWriter.setAutoFormatting(true);
        xmlWriter.writeStartDocument();
        xmlWriter.writeStartElement("tagset");
        xmlWriter.writeStartElement("video");
        xmlWriter.writeStartElement("videoName");
        xmlWriter.writeCharacters(QString("d1/").append(bboxFolders[i].left(7)));
        xmlWriter.writeEndElement(); //videoName
        xmlWriter.writeStartElement("action");
        xmlWriter.writeAttribute("nr", "N/A");
        xmlWriter.writeAttribute("class", "N/A");

        //for each bounding box:
        for (int frameNr=0; frameNr<bboxFilesForAVideo.length(); frameNr++)
        {
            QList<RectWithScore> bBoxes = getAllBoundingBoxes(bboxFolder.absoluteFilePath(bboxFilesForAVideo[frameNr]));
            if (bBoxes.isEmpty())
                continue; //no bbox for this frame

            //for each bbox for this frame
            foreach(RectWithScore bbox, bBoxes)
            {
                xmlWriter.writeStartElement("bbox");
                xmlWriter.writeAttribute("x", QString::number(bbox.first.x));
                xmlWriter.writeAttribute("y", QString::number(bbox.first.y));
                xmlWriter.writeAttribute("width", QString::number(bbox.first.width));
                xmlWriter.writeAttribute("height", QString::number(bbox.first.height));
                xmlWriter.writeAttribute("framenr", QString::number(frameNr));
                xmlWriter.writeAttribute("score", QString::number(bbox.second));
                xmlWriter.writeEndElement(); //bbox
            }
        }

        xmlWriter.writeEndElement(); //action
        xmlWriter.writeEndElement(); //video
        xmlWriter.writeEndElement(); //tagset

        xmlWriter.writeEndDocument();
    }

    //return a.exec();
}

QList<RectWithScore> getAllBoundingBoxes(QString bboxFilePath)
{
    //open file
    QFile file(bboxFilePath);
    if (!file.open(QIODevice::ReadOnly))
        return QList<RectWithScore>();

    QTextStream in(&file);
    QList<RectWithScore> boundingBoxes;

    while (!in.atEnd())
    {
        QString line = in.readLine(); // 1:153.0629;116.1167;173.1750;189.0091;7.3548;
        if (line.startsWith('<'))
            continue;

        line = line.remove(0, 2); //remove first 2 characters
        QStringList values = line.split(";");
        Point pt1(values[0].toFloat()*2, values[1].toFloat()*2); //*2 because coordinates are scaled for 320x240 images
        Point pt2(values[2].toFloat()*2, values[3].toFloat()*2);
        Rect rect(pt1, pt2);
        float score = values[4].toFloat();
        RectWithScore rctScore(rect, score);
        boundingBoxes.append(rctScore);

    }

    return boundingBoxes;

}
