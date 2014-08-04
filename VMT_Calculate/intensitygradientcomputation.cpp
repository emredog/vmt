#include "intensitygradientcomputation.h"
#include "PointCloudFunctions.h"

#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

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
    int sizeX = cube.size[0]; //cols
    int sizeY = cube.size[1]; //rows
    int sizeZ = cube.size[2];

    cv::MatConstIterator_<uchar> _it = cube.begin<uchar>();
    for(;_it!=cube.end<uchar>(); _it++)
    {
        uchar val = *_it;
        unsigned int vall = static_cast<unsigned int>(val);
        //std::cout << _it-> << std::endl;
    }

    //gradients: cube/dx , cube/dy , cube/dz
    cv::Mat gradXY(cube.dims, cube.size, CV_16SC1);
    cv::Mat gradXYZ(cube.dims, cube.size, CV_16SC1);

    //some common variables:
    cv::Range ranges[3];
//    cv::Mat prevSlice;


    //calculate gradX and gradY: ----------------------------------------------------------------------------
    ranges[0] = cv::Range::all();
    ranges[1] = cv::Range::all();
    for (int z = 0; z < sizeZ; z++)
    {

        cv::Mat currentSlice(sizeY, sizeX, CV_8UC1);
        ranges[2] = cv::Range(z, z+1); //take a slice orthogonal to Z axis

        currentSlice = cube(ranges).clone();
        // create a temporarily 2d image and copy its size
        // to make our image a real 2d image
        cv::Mat out;
        const int sizes[2] = {sizeY, sizeX};
        out.create(2, sizes, CV_8UC1);
        currentSlice.copySize(out);
        std::cout << currentSlice << std::endl;

        cv::Sobel(currentSlice, out, CV_8UC1, 1, 1); //sobel operator on X and Y: dx = 1, dy = 1
        gradXY(ranges) = out;

        out.release();
//        prevSlice = currentSlice;
        currentSlice.release();
    }

    //FIXME: delete these:
    cv::SparseMat gradXYSparse(gradXY);
    PointCloudFunctions::saveVmtAsCloud(gradXYSparse, "/home/emredog/Documents/output/gradXY_sobel.pcd");



}

cv::Mat IntensityGradientComputation::crop3dMat(const cv::Mat &inputMat, cv::Range ranges[])
{
    return inputMat(ranges).clone();
}

IntensityGradientComputation::IntensityGradientComputation()
{
}
