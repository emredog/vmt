#include <QCoreApplication>
#include <QDir>
#include <QTextStream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define MAX_X 639
#define MAX_Y 479

#define DILATION 0
#define DRAW_BEST 300   //set to a big number to display all candidates
#define T_FACTOR 0.60   //set to 0 to display all candidates

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
Rect dilateRectangle(const Rect& rect, int amount);

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    const QDir imgFolder("/home/emredog/LIRIS-data/test/vid0003/");
    const QDir bboxFolder("/home/emredog/Documents/ADSC_NUS_Harl_result_code_v2/HumanDetectionResults/vid0003_result");

    QStringList filters; filters << "*.jpg";
    QStringList imgFiles = imgFolder.entryList(filters);
    imgFiles.removeFirst();
    filters.clear();
    filters << "*.txt";
    QStringList bboxFiles = bboxFolder.entryList(filters);
    Mat img;

    namedWindow("Display Bounding Boxes", WINDOW_AUTOSIZE);

    for (int i=0; i<imgFiles.count(); i++)
    {
        img = imread(imgFolder.absoluteFilePath(imgFiles[i]).toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
        paintBBoxes(img, bboxFolder.absoluteFilePath(bboxFiles[i]), DRAW_BEST);
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
    float maxScore = boundingBoxes.last().second;
    float threshold = maxScore * T_FACTOR;

    for (int i=boundingBoxes.count()-1; i>boundingBoxes.count()-1-drawBest && i>=0; i--)
    {
        if (boundingBoxes[i].second < threshold)
            continue;

        Rect dilated = dilateRectangle(boundingBoxes[i].first, DILATION);
        rectangle(img, dilated, 255);
    }
}

Rect dilateRectangle(const Rect& rect, int amount)
{
    if (amount <= 0)
        return rect;

    Point newTopLeft, newBottomRight;

    newTopLeft.x = rect.tl().x - amount;
    if (newTopLeft.x < 0) //too much
        newTopLeft.x = 0;

    newTopLeft.y = rect.tl().y - amount;
    if (newTopLeft.y < 0) //too much
        newTopLeft.y = 0;

    newBottomRight.x = rect.br().x + amount;
    if (newBottomRight.x > MAX_X)
        newBottomRight.x = MAX_X;

    newBottomRight.y = rect.br().y + amount;
    if (newBottomRight.y > MAX_X)
        newBottomRight.y = MAX_X;

    return Rect(newTopLeft, newBottomRight);

}
