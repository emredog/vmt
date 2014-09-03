//#include <QString>
//#include <QStringList>
//#include <QDir>
//#include <QFile>
//#include <QTextStream>
//#include <QList>
//#include <QVector>


//#include "opencv2/video/tracking.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"

//#include <iostream>
//#include <ctype.h>

//#include "boundingbox.h"

//using namespace cv;
//using namespace std;

//static void help()
//{
//    // print a welcome message, and the OpenCV version
//    cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
//            "Using OpenCV version " << CV_VERSION << endl;
//    cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
//    cout << "\nHot keys: \n"
//            "\tESC - quit the program\n"
//            "\tr - auto-initialize tracking\n"
//            "\tc - delete all the points\n"
//            "\tn - switch the \"night\" mode on/off\n"
//            "To add/remove a feature point click it\n" << endl;
//}

//Point2f point;
//bool addRemovePt = false;

//static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
//{
//    if( event == CV_EVENT_LBUTTONDOWN )
//    {
//        point = Point2f((float)x, (float)y);
//        addRemovePt = true;
//    }
//}

//int main()
//{
//    //help();

//    QString videoFolderPath = "/home/emredog/LIRIS-data/training-validation/vid0004";
//    QString trackPath = "/home/emredog/LIRIS-data/training-validation_annotations-with-NO-ACTION-SLIDING_WINDOWS/union_of_bbox/vid0004_1_discussion_101-141Union.track";
//    //get GRAYSCALE image paths in the folder
//    QStringList filters;
//    filters << "*.jpg";
//    QDir videoDir(videoFolderPath);
//    QStringList depthImgFileNames = videoDir.entryList(filters, QDir::Files | QDir::NoDotAndDotDot, QDir::Name);

//    //read track file
//    QFile trackFile(trackPath);
//    if (!trackFile.open(QFile::ReadOnly))
//        return -1;

//    QList<BoundingBox> bboxSequence;

//    //Parse track file:
//    QTextStream reader(&trackFile);
//    while (!reader.atEnd())
//    {
//        QString line = reader.readLine();
//        QStringList parts = line.split(" ");

//        BoundingBox bbox;
//        bbox.frameNr    = parts[0].toInt();
//        bbox.x          = parts[1].toInt();
//        bbox.y          = parts[2].toInt();
//        bbox.width      = parts[3].toInt();
//        bbox.height     = parts[4].toInt();

//        bboxSequence.append(bbox);
//    }



//    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
//    Size subPixWinSize(10,10), winSize(31,31);

//    const int MAX_COUNT = 500;
//    bool needToInit = false;
//    bool nightMode = false;

//    namedWindow( "LK Demo", 1 );
//    setMouseCallback( "LK Demo", onMouse, 0 );

//    Mat gray, prevGray, image;
//    vector<Point2f> points[2];


//    for(int frameNo=0; frameNo<bboxSequence.length(); frameNo++)
//    {
//        BoundingBox bbox = bboxSequence[frameNo];
//        //Mat frame;
//        gray = imread(videoDir.absoluteFilePath(depthImgFileNames[bbox.frameNr-1]).toStdString(), CV_LOAD_IMAGE_GRAYSCALE);

//        if( gray.empty() )
//        {
//            cerr << "Frame is empty!!!" << endl;
//            continue;
//        }

//        gray.copyTo(image);
//        //cvtColor(image, gray, CV_BGR2GRAY);

//        if( nightMode )
//            image = Scalar::all(0);

//        if( frameNo == 0)
//        {
//            //mask the image with the bounding box
//            cv::Rect roi(bbox.x-10, bbox.y-10, bbox.width+10, bbox.height+10); //FIXME: arbitrary offset for the bbox
//            cv::Mat mask = cv::Mat(gray.rows, gray.cols, CV_8UC1, cv::Scalar(0));
//            mask(roi) = cv::Scalar(1);
//            cv::Mat maskedGrayImg = cv::Mat(gray.rows, gray.cols, gray.type(), cv::Scalar(0));
//            gray.copyTo(maskedGrayImg, mask);

//            // automatic initialization
//            goodFeaturesToTrack(maskedGrayImg, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
//            points[0].resize(points[1].size());

//            cornerSubPix(maskedGrayImg, points[1], subPixWinSize, Size(-1,-1), termcrit);
//            addRemovePt = false;


//        }
//        else if( !points[0].empty() )
//        {
//            vector<uchar> status;
//            vector<float> err;
//            if(prevGray.empty())
//                gray.copyTo(prevGray);
//            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
//                    3, termcrit, 0, 0.001);
//            size_t i, k;

//            for( i = k = 0; i < points[1].size(); i++ )
//            {
//                if( addRemovePt )
//                {
//                    if( norm(point - points[1][i]) <= 5 )
//                    {
//                        addRemovePt = false;
//                        continue;
//                    }
//                }

//                if( !status[i] )
//                    continue;

//                points[1][k++] = points[1][i];
//                circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
//                rectangle(image, Point2i(bbox.x, bbox.y), Point2i(bbox.x + bbox.width, bbox.y + bbox.height),
//                          Scalar(255,255,255));

//            }
//            points[1].resize(k);

//            //delete me later:
//            std::cout << "\\n";
//            for (int i=0; i<points[0].size() && i<points[1].size()
//                 && i<status.size() && i<err.size(); i++)
//            {
////                std::cout << "Movement of point " << i << endl;
////                std::cout << "\tIn X: " << points[1].at(i).x - points[0].at(i).x << "\tIn Y: "
////                << points[1].at(i).y - points[0].at(i).y<< std::endl;
//                std::cout << (unsigned int)status.at(i) << "\t" << err.at(i)<< std::endl;

//            };

//        }

//        if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
//        {
//            vector<Point2f> tmp;
//            tmp.push_back(point);
//            cornerSubPix( gray, tmp, winSize, cvSize(-1,-1), termcrit);
//            points[1].push_back(tmp[0]);
//            addRemovePt = false;
//        }

//        needToInit = false;
//        imshow("LK Demo", image);

//        char c = (char)waitKey(0);
//        if( c == 27 )
//            break;
//        switch( c )
//        {
//        case 'r':
//            needToInit = true;
//            break;
//        case 'c':
//            points[0].clear();
//            points[1].clear();
//            break;
//        case 'n':
//            nightMode = !nightMode;
//            break;
//        default:
//            break;
//        }



//        std::swap(points[1], points[0]);
//        cv::swap(prevGray, gray);
//    }

//    return 0;
//}
