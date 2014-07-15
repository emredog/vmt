#ifndef INTENSITYGRADIENTCOMPUTATION_H
#define INTENSITYGRADIENTCOMPUTATION_H

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)

class IntensityGradientComputation
{
public:
    static cv::Vec3f computeMeanIntensityGradient(const cv::SparseMat& sparseCube);
    static cv::Vec3f computeMeanIntensityGradient(const cv::Mat& cube);
    static cv::Mat crop3dMat(const cv::Mat &inputMat, cv::Range ranges[]);
protected:
    IntensityGradientComputation();
};

#endif // INTENSITYGRADIENTCOMPUTATION_H
