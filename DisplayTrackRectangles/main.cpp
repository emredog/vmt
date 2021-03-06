#include <QCoreApplication>
#include <QDir>
#include <QTextStream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

void paintTrackFile(Mat& img, QString trackFile, int frameIndex);

int main(int argc, char *argv[])
{

    const QDir imgFolder("/home/emredog/LIRIS-data/training-validation/vid0152/");
    const QString trackFile("/home/emredog/LIRIS-data/training-validation_annotations-with-NO-ACTION-SLIDING_WINDOWS/union_of_bbox-stationary_cam/vid0152_9_no-action_1853-1893Union.track");

    const int startFrom = 1853;

    QCoreApplication a(argc, argv);

    QStringList filters; filters << "*.jpg";
    QStringList imgFiles = imgFolder.entryList(filters);
    filters.clear();
    filters << "*.track";
    Mat img;

    namedWindow("Display Track Rectangle", WINDOW_AUTOSIZE);


    for (int i=startFrom; i<imgFiles.count(); i++)
    {
        img = imread(imgFolder.absoluteFilePath(imgFiles[i]).toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
        paintTrackFile(img, trackFile, i);
        imshow("Display Track Rectangle", img);
        waitKey(0);
    }


    return a.exec();
}

void paintTrackFile(Mat& img, QString trackFile, int frameIndex)
{
    QFile file(trackFile);
    if (!file.open(QIODevice::ReadOnly))
        return;

    QTextStream in(&file);

    while (!in.atEnd())
    {
        QString line = in.readLine();
        QStringList strValues = line.split(" ");
        if (strValues[0].toInt() != frameIndex)
            continue;
        else
        {
            rectangle(img,
                      Rect(strValues[1].toInt(), strValues[2].toInt(), strValues[3].toInt(), strValues[4].toInt()),
                      255);
            break;
        }


    }

}
