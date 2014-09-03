#include <QString>
#include <QStringList>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QList>
#include <QVector>


#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"



#include <iostream>
#include <ctype.h>

#include "boundingbox.h"

using namespace cv;
using namespace std;

static void help()
{
    // print a welcome message, and the OpenCV version
    cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
            "Using OpenCV version " << CV_VERSION << endl;
    cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
    cout << "\nHot keys: \n"
            "\tESC - quit the program\n"
            "\tr - auto-initialize tracking\n"
            "\tc - delete all the points\n"
            "\tn - switch the \"night\" mode on/off\n"
            "To add/remove a feature point click it\n" << endl;
}

Point2f point;
bool addRemovePt = false;

static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{
    if( event == CV_EVENT_LBUTTONDOWN )
    {
        point = Point2f((float)x, (float)y);
        addRemovePt = true;
    }
}

int main()
{
    //help();

    QString videoFolderPath = "/home/emredog/LIRIS-data/training-validation/vid0004";
    QString trackPath = "/home/emredog/LIRIS-data/training-validation_annotations-with-NO-ACTION-SLIDING_WINDOWS/union_of_bbox/vid0004_1_discussion_101-141Union.track";
    //get GRAYSCALE image paths in the folder
    QStringList filters;
    filters << "*.jpg";
    QDir videoDir(videoFolderPath);
    QStringList depthImgFileNames = videoDir.entryList(filters, QDir::Files | QDir::NoDotAndDotDot, QDir::Name);

    //read track file
    QFile trackFile(trackPath);
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

    SIFT sift; // = SIFT::SIFT();

    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    Size subPixWinSize(10,10), winSize(31,31);

    const int MAX_COUNT = 500;
    bool needToInit = false;

    namedWindow( "LK Demo", 1 );
    setMouseCallback( "LK Demo", onMouse, 0 );

    Mat gray, prevGray, image;
    vector<Point2f> points[2];
    vector<KeyPoint> keyPoints;




    for(int frameNo=0; frameNo<bboxSequence.length(); frameNo++)
    {
        BoundingBox bbox = bboxSequence[frameNo];
        //Mat frame;
        gray = imread(videoDir.absoluteFilePath(depthImgFileNames[bbox.frameNr-1]).toStdString(), CV_LOAD_IMAGE_GRAYSCALE);

        if( gray.empty() )
        {
            cerr << "Frame is empty!!!" << endl;
            continue;
        }

        gray.copyTo(image);
        //cvtColor(image, gray, CV_BGR2GRAY);



        //mask the image with the bounding box
        cv::Rect roi(bbox.x-10, bbox.y-10, bbox.width+10, bbox.height+10); //FIXME: arbitrary offset for the bbox
        cv::Mat mask = cv::Mat(gray.rows, gray.cols, CV_8UC1, cv::Scalar(0));
        mask(roi) = cv::Scalar(1);
        cv::Mat maskedGrayImg = cv::Mat(gray.rows, gray.cols, gray.type(), cv::Scalar(0));
        gray.copyTo(maskedGrayImg, mask);

        sift.detect(maskedGrayImg, keyPoints);
        //draw obtained keypoints
        drawKeypoints(image, keyPoints, image, Scalar(255,0,0));
        //draw the bounding box
        rectangle(image, Point2i(bbox.x, bbox.y), Point2i(bbox.x + bbox.width, bbox.y + bbox.height),
                                  Scalar(255,255,255));

        Mat descriptors;
        sift.compute(maskedGrayImg, keyPoints, descriptors);

        //        // automatic initialization
        //        goodFeaturesToTrack(maskedGrayImg, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
        //        points[0].resize(points[1].size());

        //        cornerSubPix(maskedGrayImg, points[1], subPixWinSize, Size(-1,-1), termcrit);
        //        addRemovePt = false;

        //cout << "M = "<< endl << " "  << descriptors << endl << endl;






        needToInit = false;
        imshow("LK Demo", image);

        char c = (char)waitKey(0);
        if( c == 27 )
            break;

    }

    return 0;
}

