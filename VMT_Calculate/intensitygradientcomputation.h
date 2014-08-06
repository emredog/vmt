#ifndef INTENSITYGRADIENTCOMPUTATION_H
#define INTENSITYGRADIENTCOMPUTATION_H

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)

class IntensityGradientComputation
{
public:
    enum Difference
    {
        Intermediate_Difference = 0,
        Central_Difference
    };

    static cv::Vec3f computeMeanIntensityGradientSobel3D(const cv::SparseMat& sparseCube);
    static cv::Vec3f computeMeanIntensityGradientSobel3D(const cv::Mat& cube);

    static cv::Vec3f computeMeanIntensityGradientDifferentiation(const cv::SparseMat& sparseCube, Difference diffType);
    static cv::Vec3f computeMeanIntensityGradientDifferentiation(const cv::Mat& cube, Difference diffType);

    static cv::Mat crop3dMat(const cv::Mat &inputMat, cv::Range ranges[]);




protected:
    IntensityGradientComputation();
};

#endif // INTENSITYGRADIENTCOMPUTATION_H
