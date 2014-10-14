#include <QDir>
#include <QStringList>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        cout << "Usage: ./DepthStreamViewer <dir>\n";
        return -1;
    }

    QString argument =  QString::fromAscii(argv[1]);

    QDir dirContainingFiles(argument);

    QStringList filters;
    filters << "*.png";

    QStringList entries = dirContainingFiles.entryList(filters, QDir::Files | QDir::NoDotAndDotDot, QDir::Name);
    if (entries.length() <= 0)
    {
        cout << "Empty folder!!\n\n";
        return -1;
    }

    namedWindow(argument.toStdString(), CV_WINDOW_AUTOSIZE);

    double scaleFactor = 255.0/2048.0;

    foreach (QString fileName, entries)
    {
        Mat depthMat = imread(dirContainingFiles.absoluteFilePath(fileName).toStdString(), CV_LOAD_IMAGE_UNCHANGED);
        Mat depthf (Size(640,480), CV_8UC1);
        depthMat.convertTo(depthf, CV_8UC1, scaleFactor);
        imshow(argument.toStdString(), depthf);
        waitKey(20);
    }

    destroyWindow(argument.toStdString());

    return 0;
}
