#include "intensitygradientcomputation.h"
#include "PointCloudFunctions.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

int main(int argc, char *argv[])
{
    //#	Size after trimming: 193x311x526, for vid0008_1_typing_716-756Union_STFiltered.pcd
    SparseMat spVmt = PointCloudFunctions::loadVmtFromPCD("/home/emredog/Documents/output/vid0008_1_typing_716-756Union_STFiltered.pcd",
                                        193, 311, 526);

    Mat vmt;
    spVmt.convertTo(vmt, CV_8UC1);


    cv::Range ranges[3];
    //some cube that I know there are points in it
    ranges[0] = cv::Range(0, 52);
    ranges[1] = cv::Range(0, 104);
    ranges[2] = cv::Range(0, 516);

    Mat cube =

    IntensityGradientComputation::computeMeanIntensityGradient(vmt);

}
