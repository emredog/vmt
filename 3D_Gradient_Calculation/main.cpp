#include "intensitygradientcomputation.h"
#include "PointCloudFunctions.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

int main()
{
    //#	Size after trimming: 193x311x526, for vid0008_1_typing_716-756Union_STFiltered.pcd
    SparseMat spVmt = PointCloudFunctions::loadVmtFromPCD("/home/emredog/Documents/output/vid0008_1_typing_716-756Union_STFiltered.pcd",
                                        193, 311, 526);

    size_t nzCount = spVmt.nzcount();

    Mat vmt;
    spVmt.convertTo(vmt, CV_8UC1);


    cv::Range ranges[3];
    //some cube that I know there are points in it
    ranges[0] = cv::Range(40, 80);
    ranges[1] = cv::Range(120, 160);
    ranges[2] = cv::Range(452, 484);

    Mat cube = IntensityGradientComputation::crop3dMat(vmt, ranges);

    SparseMat spCube(cube);

    nzCount = spCube.nzcount();

    IntensityGradientComputation::computeMeanIntensityGradient(cube);

}
