#include <libfreenect/libfreenect.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include <QString>
#include <QTime>
#include <QDir>
#include <QProcess>


#define OUTPUT_ROOT "/home/emredog/gsu-data"


using namespace cv;
using namespace std;


class myMutex {
    public:
        myMutex() {
            pthread_mutex_init( &m_mutex, NULL );
        }
        void lock() {
            pthread_mutex_lock( &m_mutex );
        }
        void unlock() {
            pthread_mutex_unlock( &m_mutex );
        }
    private:
        pthread_mutex_t m_mutex;
};


class MyFreenectDevice : public Freenect::FreenectDevice {
    public:
        MyFreenectDevice(freenect_context *_ctx, int _index)
            : Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
            m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false),
            m_new_depth_frame(false),
            depthMat(Size(640,480),CV_16UC1),
            rgbMat(Size(640,480), CV_8UC3, Scalar(0)),
            ownMat(Size(640,480),CV_8UC3,Scalar(0)) {

            for( unsigned int i = 0 ; i < 2048 ; i++) {
                float v = i/2048.0;
                v = std::pow(v, 3)* 6;
                m_gamma[i] = v*6*256;
            }
        }

        // Do not call directly even in child
        void VideoCallback(void* _rgb, uint32_t timestamp) {
            std::cout << "";
            m_rgb_mutex.lock();
            uint8_t* rgb = static_cast<uint8_t*>(_rgb);
            rgbMat.data = rgb;
            m_new_rgb_frame = true;
            m_rgb_mutex.unlock();
        }

        // Do not call directly even in child
        void DepthCallback(void* _depth, uint32_t timestamp) {
            std::cout << ".";
            m_depth_mutex.lock();
            uint16_t* depth = static_cast<uint16_t*>(_depth);
            depthMat.data = (uchar*) depth;
            m_new_depth_frame = true;
            m_depth_mutex.unlock();
        }

        bool getVideo(Mat& output) {
            m_rgb_mutex.lock();
            if(m_new_rgb_frame) {
                cv::cvtColor(rgbMat, output, CV_RGB2BGR);
                m_new_rgb_frame = false;
                m_rgb_mutex.unlock();
                return true;
            } else {
                m_rgb_mutex.unlock();
                return false;
            }
        }

        bool getDepth(Mat& output) {
                m_depth_mutex.lock();
                if(m_new_depth_frame) {
                    depthMat.copyTo(output);
                    m_new_depth_frame = false;
                    m_depth_mutex.unlock();
                    return true;
                } else {
                    m_depth_mutex.unlock();
                    return false;
                }
            }
    private:
        std::vector<uint8_t> m_buffer_depth;
        std::vector<uint8_t> m_buffer_rgb;
        std::vector<uint16_t> m_gamma;
        Mat depthMat;
        Mat rgbMat;
        Mat ownMat;
        myMutex m_rgb_mutex;
        myMutex m_depth_mutex;
        bool m_new_rgb_frame;
        bool m_new_depth_frame;
};


int main(int argc, char **argv) {
    bool die(false);
    bool write = true;
    bool recordBoth = true;
    int delaySec = 0;

    if (argc != 2 && argc != 3 && argc != 4)
    {
        cout << "Usage: ./KinectCapture_libfreenect <seconds_to_record> [delay_before_recording] [-d or --depthonly]\n\n";
        return -1;
    }

    QString secStr = QString::fromAscii(argv[1]);
    bool ok = false;
    int stopSec = secStr.toInt(&ok);

    if (!ok || stopSec < 0)
    {
        cout << "Usage: ./KinectCapture_libfreenect <seconds_to_record> [delay_before_recording]\n\nseconds_to_record must be a positive integer.\n\n";
        return -1;
    }

    if (argc >= 3)
    {
        secStr = QString::fromAscii(argv[2]);
        delaySec = secStr.toInt(&ok);
        if (!ok || delaySec < 0)
        {
            cout << "Usage: ./KinectCapture_libfreenect <seconds_to_record> [delay_before_recording]\n\ndelay_before_recording must be a positive integer.\n\n";
            return -1;
        }
    }

    if (argc == 4)
    {
        secStr = QString::fromAscii(argv[3]);
        if (secStr.compare("-d") != 0 || secStr.compare("--depthonly") != 0)
        {
            cout << "Usage: ./KinectCapture_libfreenect <seconds_to_record> [delay_before_recording] [-d or --depthonly]\n\n";
            return -1;
        }
        recordBoth = false;
    }

    QDir outputDir;
    {
        QString outputDirStr = QString("%1/KinectCapture_%2/").arg(OUTPUT_ROOT).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss"));
        outputDir = QDir(outputDirStr);
        outputDir.mkpath(outputDirStr);
    }

    int stopMilisec = stopSec * 1000;

    //string filename("snapshot");
    //string suffix(".png");
    int i_snap(0);

    Mat depthMat(Size(640,480),CV_16UC1);
    //Mat depthf (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat grayMat(Size(640,480),CV_8UC1,Scalar(0));

    // The next two lines must be changed as Freenect::Freenect
    // isn't a template but the method createDevice:
    // Freenect::Freenect<MyFreenectDevice> freenect;
    // MyFreenectDevice& device = freenect.createDevice(0);
    // by these two lines:

    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);

    //namedWindow("rgb",CV_WINDOW_AUTOSIZE);
    //namedWindow("depth",CV_WINDOW_AUTOSIZE);    


    vector<int> params;
    params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    params.push_back(2);



    if (delaySec > 0)
    {
        namedWindow("dummywin");
        cout << "Recording is not startet yet. Awaiting any key press, or timeout (" << delaySec << "seconds)\n";
        waitKey(delaySec * 1000);
        destroyWindow("dummywin");
    }




    cout << '\a';
    cout << "Starting recording...\n";
    QTime myTimer;
    myTimer.start();
    device.startDepth();
    if (recordBoth)
    {
        device.startVideo();
    }

    while (!die) {
        if (recordBoth) device.getVideo(rgbMat);
        device.getDepth(depthMat);
        //cv::imshow("rgb", rgbMat);

        //depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);


        //cv::imshow("depth",depthf);
        char k = cvWaitKey(5);
//        if( k == 27 ){
//            //cvDestroyWindow("rgb");
//            cvDestroyWindow("depth");
//            break;
//        }
        if(write)
        {
            QString index = QTime::currentTime().toString("HH-mm-ss-zzz");
            QString fileName = QString("depth_%1.png").arg(index);
            cv::imwrite(outputDir.absoluteFilePath(fileName).toStdString(), depthMat, params);
            if (recordBoth)
            {
                cvtColor(rgbMat, grayMat, CV_BGR2GRAY);
                fileName = QString("gray_%1.jpg").arg(index);
                cv::imwrite(outputDir.absoluteFilePath(fileName).toStdString(), grayMat);
            }
            i_snap++;
        }

        if (myTimer.elapsed() > stopMilisec)
            break;
        //if(iter >= 500) break;
        //iter++;
    }
    int milliseconds = myTimer.elapsed();
    std::cout << i_snap << " frames are captured in " << (double)milliseconds / 1000.0 << " seconds\n";
    cout << '\a';

    if (recordBoth) device.stopVideo();
    device.stopDepth();
    return 0;
}
