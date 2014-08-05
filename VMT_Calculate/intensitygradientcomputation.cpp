#include "intensitygradientcomputation.h"
#include "PointCloudFunctions.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#include <QString>

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

    //FIXME: delete these:
//    cv::SparseMat cubeSparse(cube);
//    PointCloudFunctions::saveVmtAsCloud(cubeSparse, "/home/emredog/Documents/output/cube.pcd");
//    cubeSparse.release();

    //Size of cube
    int sizeRows = cube.size[0];
    int sizeCols = cube.size[1];
    int sizeDepth = cube.size[2];

    //gradients: cube/dx , cube/dy , cube/dz
    cv::Mat gradXY(cube.dims, cube.size, CV_8UC1);
    gradXY = cv::Scalar(0);
    //cv::Mat gradXYZ(cube.dims, cube.size, CV_8UC1/*CV_16SC1*/);

    //some common variables:
    cv::Range ranges[3];

    //calculate gradX and gradY: ----------------------------------------------------------------------------
    ranges[0] = cv::Range::all();
    ranges[1] = cv::Range::all();
    for (int z = 0; z < sizeDepth; z++)
    {

        cv::Mat currentSlice(sizeRows, sizeCols, CV_8UC1);
        ranges[2] = cv::Range(z, z+1); //take a slice orthogonal to Z axis

        currentSlice = cube(ranges).clone();

        // create a temporarily 2d image and copy its size
        // to make our image a real 2d image
        cv::Mat out;
        const int sizes[2] = {sizeRows, sizeCols};
        out.create(2, sizes, CV_8UC1);
        currentSlice.copySize(out);

        //QString mmm = QString("/home/emredog/Documents/output/slice/%1.jpg").arg(z);
        //cv::imwrite(mmm.toStdString(), currentSlice);

        //sobel operator on X and Y: dx = 1, dy = 1
        cv::Sobel(currentSlice, out, CV_8UC1, 1, 1, 3, 1, 0, cv::BORDER_REFLECT_101);

        //QString ppp = QString("/home/emredog/Documents/output/sobeled/%1.jpg").arg(z);
        //cv::imwrite(ppp.toStdString(), out);

        //write sobel'ed values into 3D matrix:
        for (int x = 0; x<out.cols; x++)
            for (int y = 0; y<out.rows; y++)
            {
                uchar val = out.at<uchar>(x, y);
                if (val > 0)
                    gradXY.at<uchar>(x, y, z) =  val;
            }

        out.release();
        currentSlice.release();
    }
//    cv::SparseMat gradXYSparse(gradXY);
//    PointCloudFunctions::saveVmtAsCloud(gradXYSparse, "/home/emredog/Documents/output/cube_gradXY_sobel.pcd");


    //calculate gradZ ---------------------------------------------------------------------------------------
    cv::Mat gradXYZ(cube.dims, cube.size, CV_8UC1);
    gradXYZ = cv::Scalar(0);

    //think of it as if we rotated the cube around Y axis:
    ranges[1] = cv::Range::all();
    ranges[2] = cv::Range::all();
    for (int x = 0; x < sizeCols; x++) //think of x as "depth"
    {
        cv::Mat currentSlice(sizeRows, sizeDepth, CV_8UC1); //and think of Z as "width"
        ranges[0] = cv::Range(x, x+1); //take a slice orthogonal to X axis

        currentSlice = gradXY(ranges).clone();

        // create a temporarily 2d image and copy its size
        // to make our image a real 2d image
        cv::Mat out;
        const int sizes[2] = {sizeRows, sizeDepth};
        out.create(2, sizes, CV_8UC1);
        currentSlice.copySize(out);

        QString mmm = QString("/home/emredog/Documents/output/slice/XYZ_%1.jpg").arg(x);
        cv::imwrite(mmm.toStdString(), currentSlice);

        //sobel operator on X and Y: dx = 1, dy = 0 (it's actually dz = 1, since we "kinda" rotated the cube)
        cv::Sobel(currentSlice, out, CV_8UC1, 1, 0, 3, 1, 0, cv::BORDER_REFLECT_101);

        QString ppp = QString("/home/emredog/Documents/output/sobeled/XYZ_%1.jpg").arg(x);
        cv::imwrite(ppp.toStdString(), out);

        //write sobel'ed values into 3D matrix:
        for (int z = 0; z<out.cols; z++) //note that: now out's cols actually goes up to sizeZ
            for (int y = 0; y<out.rows; y++)
            {
                uchar val = out.at<uchar>(z, y);
                if (val > 0)
                    gradXYZ.at<uchar>(x, y, z) =  val;
            }

        out.release();
        currentSlice.release();
    }

//    cv::SparseMat gradXYZSparse(gradXYZ);
//    PointCloudFunctions::saveVmtAsCloud(gradXYZSparse, "/home/emredog/Documents/output/cube_gradXYZ_sobel.pcd");





}

cv::Mat IntensityGradientComputation::crop3dMat(const cv::Mat &inputMat, cv::Range ranges[])
{
    return inputMat(ranges).clone();
}

IntensityGradientComputation::IntensityGradientComputation()
{
}
