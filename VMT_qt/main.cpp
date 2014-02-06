#include <QCoreApplication>
// Video Image PSNR and SSIM
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <queue>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O

#include "VmtFunctions.h"
#include "PointCloudFunctions.h"
#include "HelperFunctions.h"

using namespace std;

#define WINDOW_SIZE 32
#define DEPTH_TOLERANCE 30
#define X_TOLERANCE 0
#define Y_TOLERANCE 0
#define LEAF_SIZE 10.0f

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    VmtFunctions* vmtCore = new VmtFunctions();
    string winName = "Display Window";
    clock_t start_time, end_time;
    //cv::namedWindow(winName, CV_WINDOW_AUTOSIZE );// Create a window for display.

    vector<cv::SparseMat> volumeObjects;
    vector<cv::SparseMat> volObjDifferences;
    cv::SparseMat lastVmt;

    string folder = "/home/emredog/LIRIS-data/training-validation/vid0031/";
    string outputFolder = "/home/emredog/Documents/output/";

    string prefix = "depth";
    vector<string> depthImages;
    HelperFunctions::getDir(folder, depthImages);

    //For the sequence, load every image of action
    //---------------------------------------------------------------------
    int counter = 1;
    for(vector<string>::const_iterator it=depthImages.begin(); it != depthImages.end(); it++)
    {
        //if the file doesn't contain 'prefix', skip it
        size_t found = it->find(prefix);
        if (found==string::npos)
            continue;

        cout << "Begining iteration " << counter << "-------------------------" << endl;

        //Generate Volume Object ( Ot(x,y,z) in the paper)
        //--------------------------------------------------------------------
        start_time = clock();
        cv::SparseMat volumeObject = vmtCore->GenerateSparseVolumeObject(folder + (*it), winName);

        end_time = clock();
        if (volumeObject.nzcount() > 0)
        {
            cout << "\tVolume object generated in " << float(end_time - start_time) / CLOCKS_PER_SEC << " seconds." << endl;
            volumeObjects.push_back(volumeObject); //FIXME!!
        }

        if (volumeObjects.size() == 2) //enough elements to subtract
        {
            //Calculate differences
            cv::SparseMat& t = volumeObjects.back();
            cv::SparseMat& tMinusOne = volumeObjects.front();
            start_time = clock();
            cv::SparseMat difference = vmtCore->SubtractSparseMat(t, tMinusOne, DEPTH_TOLERANCE, X_TOLERANCE, Y_TOLERANCE);
            end_time = clock();
            volObjDifferences.push_back(difference);
            cout << "\tDifference calculated in " << float(end_time - start_time) / CLOCKS_PER_SEC << " seconds." << endl;
            //if (difference.nzcount() > 0)
            //	PointCloudFunctions::SaveVmtAsCloud(difference, (*it) + ".pcd");

            //remove t-1st volume object, we dont need it anymore
            volumeObjects.erase(volumeObjects.begin());
        }

        if (volObjDifferences.size() == WINDOW_SIZE) //we have enough elements to calculate a VMT
        {
            //calculate VMT
            //TODO: THIS CODE IS HIGHLY OPTIMIZABLE!!! (VMT CALCULATIONS ARE MADE REDUNDANTLY!!!!)
            //TODO: create a VMT class with properties (no statics!) and manage VMTs internally
            start_time = clock();
            lastVmt = vmtCore->ConstructVMT(volObjDifferences);
            end_time = clock();
            cout << "\tVMT calculated in " << float(end_time - start_time) / CLOCKS_PER_SEC << " seconds." << endl;
            if (lastVmt.nzcount() > 0)
            {
                start_time = clock();
                PointCloud<PointXYZI>::Ptr cloud = PointCloudFunctions::convertToPointCloud(lastVmt);
                PointCloud<PointXYZI>::Ptr downsampled = PointCloudFunctions::downSampleCloud(cloud, LEAF_SIZE, true, outputFolder + (*it) + "DS_VMT.pcd");
                end_time = clock();
                cout << "\tPoint cloud created, downsampled and written to disk in " << float(end_time - start_time) / CLOCKS_PER_SEC << " seconds." << endl;
                //clean-up memory
                cloud.reset();
                downsampled.reset();
            }
            //remove the oldest element
            volObjDifferences.erase(volObjDifferences.begin());
        }

        counter++;
    }

    //clean-up
    delete vmtCore;

    cout << endl << "Done." << endl;

    return a.exec();
}
