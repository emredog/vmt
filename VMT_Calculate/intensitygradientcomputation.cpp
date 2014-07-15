#include "intensitygradientcomputation.h"
#include "PointCloudFunctions.h"

#include "opencv2/imgproc/imgproc.hpp"

cv::Vec3f IntensityGradientComputation::computeMeanIntensityGradient(const cv::SparseMat &sparseCube)
{
    //Transform sparsemat into a regular mat:
    cv::Mat cube;
    sparseCube.convertTo(cube, CV_8UC1);

    IntensityGradientComputation::computeMeanIntensityGradient(cube);
}

cv::Vec3f IntensityGradientComputation::computeMeanIntensityGradient(const cv::Mat &cube)
{
    if (cube.type() != CV_8UC1)
        return cv::Vec3f(-1.0, -1.0, -1.0);

    //Size of cube
    int sizeX = cube.size[0];
    int sizeY = cube.size[1];
    int sizeZ = cube.size[2];

    //gradients: cube/dx , cube/dy , cube/dz
    cv::Mat gradX(cube.dims, cube.size, CV_16SC1);
    cv::Mat gradY(cube.dims, cube.size, CV_16SC1);
    cv::Mat gradZ(cube.dims, cube.size, CV_16SC1);

    //some common variables:
    cv::Range ranges[3];
//    cv::Mat prevSlice;


    //calculate gradX: ----------------------------------------------------------------------------
    ranges[1] = cv::Range::all();
    ranges[2] = cv::Range::all();
    for (int x = 0; x < sizeX; x++)
    {

        cv::Mat currentSlice(sizeY, sizeZ, CV_8UC1);
        ranges[0] = cv::Range(x, x+1);

        currentSlice = cube(ranges).clone();
        // create a temporarily 2d image and copy its size
        // to make our image a real 2d image
        cv::Mat out;
        const int sizes[2] = {sizeY, sizeZ};
        out.create(2, sizes, CV_8UC1);
        currentSlice.copySize(out);

        cv::Sobel(currentSlice, out, CV_8UC1, 1, 0); //sobel operator on X: dx = 1, dy = 0
        gradX(ranges) = out;

//        if (x != 0) //skip these for the first slice
//        {
//            cv::Mat diff = currentSlice - prevSlice;
//            gradX(ranges) = diff;
//        }

        out.release();
//        prevSlice = currentSlice;
        currentSlice.release();
    }

    //FIXME: delete these:
    cv::SparseMat gradXSparse(gradX);
    PointCloudFunctions::saveVmtAsCloud(gradXSparse, "/home/emredog/Documents/output/gradX_sobel.pcd");

    //calculate gradY: ----------------------------------------------------------------------------
    ranges[0] = cv::Range::all();
    ranges[2] = cv::Range::all();
    for (int y = 0; y < sizeY; y++)
    {

        cv::Mat currentSlice(sizeX, sizeZ, CV_8UC1);
        ranges[1] = cv::Range(y, y+1);

        currentSlice = cube(ranges).clone();
        // create a temporarily 2d image and copy its size
        // to make our image a real 2d image
        cv::Mat out;
        const int sizes[2] = {sizeX, sizeZ};
        out.create(2, sizes, CV_8UC1);
        currentSlice.copySize(out);

        cv::Sobel(currentSlice, out, CV_8UC1, 0, 1); //sobel operator on X: dx = 1, dy = 0
        gradY(ranges) = out;

        out.release();

//        if (y != 0) //skip these for the first slice
//        {
//            cv::Mat diff = currentSlice - prevSlice;
//            gradY(ranges) = diff;
//        }

//        prevSlice = currentSlice;
        currentSlice.release();
    }

    //FIXME: delete these:
    cv::SparseMat gradYSparse(gradY);
    PointCloudFunctions::saveVmtAsCloud(gradYSparse, "/home/emredog/Documents/output/gradY_sobel.pcd");

}

cv::Mat IntensityGradientComputation::crop3dMat(const cv::Mat &inputMat, cv::Range ranges[])
{
    return inputMat(ranges).clone();
}

IntensityGradientComputation::IntensityGradientComputation()
{
}
