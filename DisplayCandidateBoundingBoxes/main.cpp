#include <QCoreApplication>
#include <QDir>
#include <QTextStream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

typedef QPair<Rect, float> RectWithScore;

bool operator>(const RectWithScore &cC1, const RectWithScore &cC2)
{
    return cC1.second > cC2.second;
}

bool operator< (const RectWithScore &cC1, const RectWithScore &cC2)
{
    return cC1.second < cC2.second;
}

void paintBBoxes(Mat& img, QString bboxFilePath, int drawBest);


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    const QDir imgFolder("/home/emredog/LIRIS-data/test/vid0006/");
    const QDir bboxFolder("/home/emredog/Documents/ADSC_NUS_Harl_result_code_v2/HumanDetectionResults/vid0006_result");

    QStringList filters; filters << "*.jpg";
    QStringList imgFiles = imgFolder.entryList(filters);
    filters.clear();
    filters << "*.txt";
    QStringList bboxFiles = bboxFolder.entryList(filters);
    Mat img;

    namedWindow("Display Bounding Boxes", WINDOW_AUTOSIZE);

    for (int i=0; i<imgFiles.count(); i++)
    {
        img = imread(imgFolder.absoluteFilePath(imgFiles[i]).toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
        paintBBoxes(img, bboxFolder.absoluteFilePath(bboxFiles[i]), 3);
        imshow("Display Bounding Boxes", img);
        waitKey(0);
    }



    return a.exec();
}


void paintBBoxes(Mat& img, QString bboxFilePath, int drawBest)
{
    //paint directly all rectangles
    QFile file(bboxFilePath);
    if (!file.open(QIODevice::ReadOnly))
        return;

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

    //sort list by score (ascending)
    qSort(boundingBoxes);

    for (int i=boundingBoxes.count()-1; i>boundingBoxes.count()-1-drawBest; i--)
    {
        rectangle(img, boundingBoxes[i].first, 255);
    }


}
