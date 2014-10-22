#include <QDir>
#include <QStringList>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    if (argc != 2 && argc != 3)
    {
        cout << "Usage: ./DepthStreamViewer [-s] <dir>\n\n";
        return -1;
    }

    QString argument =  QString::fromAscii((argc == 2)? argv[1] : argv[2]);
    QDir dirContainingFiles(argument);
    QDir saveDir;
    bool doSave = false;

    if (argc == 3)
    {
        argument = QString::fromAscii(argv[1]);
        if (argument.compare("-s") == 0)
        {
            doSave = true;
            saveDir = QDir(QString("%1/%2").arg(dirContainingFiles.absolutePath()).arg("grayScaled"));
            if (!saveDir.exists()) saveDir.mkpath(saveDir.absolutePath());
        }
        else
        {
            cout << "Usage: ./DepthStreamViewer [-s] <dir>\n\n";
            return -1;
        }
    }



    QStringList filters;
    filters << "*.png";

    string winName = (argc == 2)? argv[1] : argv[2];

    QStringList entries = dirContainingFiles.entryList(filters, QDir::Files | QDir::NoDotAndDotDot, QDir::Name);
    if (entries.length() <= 0)
    {
        cout << "Empty folder!!\n\n";
        return -1;
    }

    namedWindow(winName, CV_WINDOW_AUTOSIZE);

    double scaleFactor = 255.0/2048.0;

    foreach (QString fileName, entries)
    {
        Mat depthMat = imread(dirContainingFiles.absoluteFilePath(fileName).toStdString(), CV_LOAD_IMAGE_UNCHANGED);
        Mat depthf (Size(640,480), CV_8UC1);
        depthMat.convertTo(depthf, CV_8UC1, scaleFactor);
        imshow(winName, depthf);
        waitKey(20);
        if (doSave)
        {
            imwrite(saveDir.absoluteFilePath(fileName).toStdString(), depthf);
        }
    }

    destroyWindow(winName);

    return 0;
}
