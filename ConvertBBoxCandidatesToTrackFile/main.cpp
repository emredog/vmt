#include <QCoreApplication>
#include <QDir>
#include <QTextStream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define MAX_X 639
#define MAX_Y 479

#define DILATION 10

#define BEST_N 3




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

QList<Rect> getBestBoundingBox(QString bboxFilePath, int bestN);
Rect dilateRectangle(const Rect& rect, int amount);

int main(/*int argc, char *argv[]*/)
{
    const QString suffix = QString("_b%1_d%2").arg(BEST_N).arg(DILATION);
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
        QFile trackFile(testFolder.absoluteFilePath(bboxFolders[i].left(7).append(suffix).append(".track")));
        if (!trackFile.open(QIODevice::Truncate | QIODevice::WriteOnly))
                continue;

        QTextStream out(&trackFile);
        //write frame number

        //QDir vidFolder(testFolder.absoluteFilePath(vidFolders[i]));
        QDir bboxFolder(bboxMainFolder.absoluteFilePath(bboxFolders[i]));

        QStringList bboxFilesForAVideo = bboxFolder.entryList(filters, QDir::Files, QDir::Name);

        for (int frameNr=0; frameNr<bboxFilesForAVideo.length(); frameNr++)
        {
            QList<Rect> bestBBoxes = getBestBoundingBox(bboxFolder.absoluteFilePath(bboxFilesForAVideo[frameNr]), 3);

            if (bestBBoxes.isEmpty())
            {
                continue;
            }
            else if (bestBBoxes.length() == 1) //only one rectangle, dilate it:
            {
                if (bestBBoxes[0].width > 0) //valid rectangle?
                {
                    Rect dilated = dilateRectangle(bestBBoxes[0], DILATION);
                    out << frameNr+1 << " "
                        << dilated.x << " "
                        << dilated.y << " "
                        << dilated.width << " "
                        << dilated.height << "\n";
                }
            }
            else //more than one rectangles, create a new rectangle that contains all bounding boxes
            {
                int topLeftMinX = INT_MAX, topLeftMinY = INT_MAX, botRightMaxX = INT_MIN, botRightMaxy = INT_MIN;
                //get topmost-leftmost point and bottommost-rightmost point of all available bounding boxes
                foreach(Rect rect, bestBBoxes)
                {
                    if (rect.tl().x < topLeftMinX) topLeftMinX = rect.tl().x;
                    if (rect.tl().y < topLeftMinY) topLeftMinY = rect.tl().y;
                    if (rect.br().x > botRightMaxX) botRightMaxX = rect.br().x;
                    if (rect.br().x > botRightMaxy) botRightMaxy = rect.br().y;
                }

                Rect containerRect(Point(topLeftMinX, topLeftMinY), Point(botRightMaxX, botRightMaxy));
                Rect dilated = dilateRectangle(containerRect, DILATION);
                out << frameNr+1 << " "
                    << containerRect.x << " "
                    << containerRect.y << " "
                    << containerRect.width << " "
                    << containerRect.height << "\n";
            }
        }

        out.flush();
        trackFile.close();
    }
}

QList<Rect> getBestBoundingBox(QString bboxFilePath, int bestN)
{
    //open file
    QFile file(bboxFilePath);
    if (!file.open(QIODevice::ReadOnly))
        return QList<Rect>();

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

    if (boundingBoxes.isEmpty())
        return QList<Rect>();

    QList<Rect> bestRectangles;

    if (bestN <= 1)
    {
        //get maximum score rectangle
        QList<RectWithScore>::iterator it = std::max_element(boundingBoxes.begin(), boundingBoxes.end());

        bestRectangles.append(it->first);
    }
    else
    {
        qSort(boundingBoxes);
        for (int n = boundingBoxes.length()-1; (n > boundingBoxes.length()-1-bestN) && n >= 0; n--)
            bestRectangles.append(boundingBoxes[n].first);
    }

    return bestRectangles;

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
