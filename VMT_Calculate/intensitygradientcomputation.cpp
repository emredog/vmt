#include "intensitygradientcomputation.h"
#include "PointCloudFunctions.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#include <QString>

cv::Vec3f IntensityGradientComputation::computeMeanIntensityGradientSobel3D(const cv::SparseMat &sparseCube)
{
    //Transform sparsemat into a regular mat:
    cv::Mat cube;
    sparseCube.convertTo(cube, CV_8UC1);

    IntensityGradientComputation::computeMeanIntensityGradientSobel3D(cube);
}

cv::Vec3f IntensityGradientComputation::computeMeanIntensityGradientSobel3D(const cv::Mat &cube)
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
        out.create(2, sizes, CV_64FC1); //not to miss edges with negative slope: http://docs.opencv.org/trunk/doc/py_tutorials/py_imgproc/py_gradients/py_gradients.html#one-important-matter
        currentSlice.copySize(out);
        cv::Mat currentSlice64f(sizeRows, sizeCols, CV_64FC1);
        currentSlice.convertTo(currentSlice64f, CV_64FC1);

        //QString mmm = QString("/home/emredog/Documents/output/slice/%1.jpg").arg(z);
        //cv::imwrite(mmm.toStdString(), currentSlice);

        //sobel operator on X and Y: dx = 1, dy = 1
        cv::Sobel(currentSlice64f, out, CV_64FC1, 1, 1, 3, 1, 0, cv::BORDER_REFLECT_101);

        //take absolutue value of the out (http://docs.opencv.org/trunk/doc/py_tutorials/py_imgproc/py_gradients/py_gradients.html#one-important-matter)
        cv::Mat outAbsolute = cv::abs(out);
        cv::Mat outAbsolute8u(sizeRows, sizeCols, CV_8UC1);
        outAbsolute.convertTo(outAbsolute8u, CV_8UC1);


        //QString ppp = QString("/home/emredog/Documents/output/sobeled/%1.jpg").arg(z);
        //cv::imwrite(ppp.toStdString(), out);

        //write sobel'ed values into 3D matrix:
        for (int x = 0; x<outAbsolute8u.cols; x++)
            for (int y = 0; y<outAbsolute8u.rows; y++)
            {
                uchar val = outAbsolute8u.at<uchar>(x, y);
                if (val > 0)
                    gradXY.at<uchar>(x, y, z) =  val;
            }

        //clean-up
        out.release();
        currentSlice.release();
        currentSlice64f.release();
        outAbsolute.release();
        outAbsolute8u.release();
    }
    cv::SparseMat gradXYSparse(gradXY);
    PointCloudFunctions::saveVmtAsCloud(gradXYSparse, "/home/emredog/Documents/output/cube_gradXY_sobel.pcd");


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
        out.create(2, sizes, CV_64FC1); //not to miss edges with negative slope: http://docs.opencv.org/trunk/doc/py_tutorials/py_imgproc/py_gradients/py_gradients.html#one-important-matter
        currentSlice.copySize(out);
        cv::Mat currentSlice64f(sizeRows, sizeCols, CV_64FC1);
        currentSlice.convertTo(currentSlice64f, CV_64FC1);

        //        QString mmm = QString("/home/emredog/Documents/output/slice/XYZ_%1.jpg").arg(x);
        //        cv::imwrite(mmm.toStdString(), currentSlice);

        //sobel operator on X and Y: dx = 1, dy = 0 (it's actually dz = 1, since we "kinda" rotated the cube)
        cv::Sobel(currentSlice64f, out, CV_64FC1, 1, 0, 3, 1, 0, cv::BORDER_REFLECT_101);

        //        QString ppp = QString("/home/emredog/Documents/output/sobeled/XYZ_%1.jpg").arg(x);
        //        cv::imwrite(ppp.toStdString(), out);

        //take absolutue value of the out (http://docs.opencv.org/trunk/doc/py_tutorials/py_imgproc/py_gradients/py_gradients.html#one-important-matter)
        cv::Mat outAbsolute = cv::abs(out);
        cv::Mat outAbsolute8u(sizeRows, sizeCols, CV_8UC1);
        outAbsolute.convertTo(outAbsolute8u, CV_8UC1);

        //write sobel'ed values into 3D matrix:
        for (int z = 0; z<outAbsolute8u.cols; z++) //note that: now out's cols actually goes up to sizeZ
            for (int y = 0; y<outAbsolute8u.rows; y++)
            {
                uchar val = outAbsolute8u.at<uchar>(z, y);
                if (val > 0)
                    gradXYZ.at<uchar>(x, y, z) =  val;
            }

        //clean-up
        out.release();
        currentSlice.release();
        currentSlice64f.release();
        outAbsolute.release();
        outAbsolute8u.release();
    }

    cv::SparseMat gradXYZSparse(gradXYZ);
    PointCloudFunctions::saveVmtAsCloud(gradXYZSparse, "/home/emredog/Documents/output/cube_gradXYZ_sobel.pcd");





}

cv::Vec3f IntensityGradientComputation::computeMeanIntensityGradientDifferentiation(const cv::SparseMat &sparseCube, Difference diffType)
{
    //Transform sparsemat into a regular mat:
    cv::Mat cube;
    sparseCube.convertTo(cube, CV_8UC1);

    IntensityGradientComputation::computeMeanIntensityGradientDifferentiation(cube, diffType);
}

cv::Vec3f IntensityGradientComputation::computeMeanIntensityGradientDifferentiation(const cv::Mat &cube, Difference diffType)
{
    if (cube.type() != CV_8UC1)
        return cv::Vec3f(-1.0, -1.0, -1.0);

    //Size of cube
    int sizeRows = cube.size[0];
    int sizeCols = cube.size[1];
    int sizeDepth = cube.size[2];

    int sizes[] = {sizeRows, sizeCols, sizeDepth};

    //calculate dX, dY and dZ for the whole cube-------------------------------------------------------------
    cv::Mat cubeDx(3, sizes, CV_16UC1);
    cv::Mat cubeDy(3, sizes, CV_16UC1);
    cv::Mat cubeDz(3, sizes, CV_16UC1);
    cubeDx = cv::Scalar(0);
    cubeDy = cv::Scalar(0);
    cubeDz = cv::Scalar(0);

    //sums of gradient magnituded in separate directions
    int sumX = 0, sumY = 0, sumZ = 0;
    //number of points taken into account for a certain direction
    int counterX = 0, counterY = 0, counterZ = 0;
    int valX, valY, valZ;

    //correction for Central_Difference case (because we can't go to x+1, it's outside of boundaries)
    if (diffType == Central_Difference)
    {
        sizeDepth--; sizeRows--; sizeCols--;
    }



    int x1;
    int x2;

    for (int z = 1; z<sizeDepth; z++)
        for (int y = 1; y<sizeRows; y++)
            for (int x = 1; x < sizeCols; x++) //FIXME: border conditions!!
            {
                if (diffType == Central_Difference)
                {
                    //calculate values in any direction with [-1 0 1] kernel --> NO SMOOTHING

                    valX = -static_cast<int>(cube.at<uchar>(x-1, y, z)) + static_cast<int>(cube.at<uchar>(x+1, y, z));
                    valY = -static_cast<int>(cube.at<uchar>(x, y-1, z)) + static_cast<int>(cube.at<uchar>(x, y+1, z));
                    valZ = -static_cast<int>(cube.at<uchar>(x, y, z-1)) + static_cast<int>(cube.at<uchar>(x, y, z+1));

                    cubeDx.at<int>(x, y, z) = valX;
                    cubeDy.at<int>(x, y, z) = valY;
                    cubeDz.at<int>(x, y, z) = valZ;
                }
                else //Intermediate difference:
                {
                    //calculate values in any direction with [-1 1] kernel --> NO SMOOTHING
                    valX = -static_cast<int>(cube.at<uchar>(x-1, y, z)) + static_cast<int>(cube.at<uchar>(x, y, z));
                    valY = -static_cast<int>(cube.at<uchar>(x, y-1, z)) + static_cast<int>(cube.at<uchar>(x, y, z));
                    valZ = -static_cast<int>(cube.at<uchar>(x, y, z-1)) + static_cast<int>(cube.at<uchar>(x, y, z));
                }

                //cubeDx.at<int>(x, y, z) = valX;
                //cubeDy.at<int>(x, y, z) = valY;
                //cubeDz.at<int>(x, y, z) = valZ;

                if (valX != 0){sumX += valX; counterX++;}
                if (valY != 0){sumY += valY; counterY++;}
                if (valZ != 0){sumZ += valZ; counterZ++;}
            }

    cv::Vec3f vec;

    vec[0] = counterX == 0 ? 0 : (float)sumX / (float)counterX;
    vec[1] = counterY == 0 ? 0 : (float)sumY / (float)counterY;
    vec[2] = counterZ == 0 ? 0 : (float)sumZ / (float)counterZ;

    // write gradient cubes
        cv::SparseMat cubeDxSparse(cubeDx);
        PointCloudFunctions::saveVmtAsCloud(cubeDxSparse, "/home/emredog/Documents/output/cube_gradX_CentralDiff.pcd");
        cv::SparseMat cubeDySparse(cubeDy);
        PointCloudFunctions::saveVmtAsCloud(cubeDySparse, "/home/emredog/Documents/output/cube_gradY_CentralDiff.pcd");
        cv::SparseMat cubeDzSparse(cubeDz);
        PointCloudFunctions::saveVmtAsCloud(cubeDzSparse, "/home/emredog/Documents/output/cube_gradZ_CentralDiff.pcd");

    return vec;
}

cv::Mat IntensityGradientComputation::crop3dMat(const cv::Mat &inputMat, cv::Range ranges[])
{
    return inputMat(ranges).clone();
}

IntensityGradientComputation::IntensityGradientComputation()
{
}
