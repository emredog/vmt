#pragma once

#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <fstream>
#include <iostream> // for standard I/O
#include <queue>

#define I_MAX 255
#define PREDEFINED_THRESHOLD 150 //156
#define X_SIZE 640
#define Y_SIZE 480
#define Z_SIZE 2047
#define MIN_Z 0
#define MAX_Z Z_SIZE



using namespace std;

class VmtFunctions
{
private:
	static const int dims = 3;
	int *matrixSize;
    unsigned int permittedMinZ;
    unsigned int permittedMaxZ;

	enum DimIndex
	{
		Y = 0,
		X = 1,
		Z = 2
	};

	inline float raw_depth_to_meters(int raw_depth)
	{
		if (raw_depth < 2047)
			return 1.0 / (raw_depth * -0.0030711016 + 3.3309495161);
		return 0;
	}

public:
	//constructor for 3D VMTs
	VmtFunctions(int xSize = X_SIZE, int ySize = Y_SIZE);
	~VmtFunctions(void);

	cv::SparseMat GenerateSparseVolumeObject(string imagePath, string windowName, int downsamplingRate = 2);
	cv::SparseMat GenerateSparseVolumeObject(string imagePath, string textPath, string windowName);

    cv::SparseMat SubtractSparseMat(const cv::SparseMat& operand1, const cv::SparseMat& operand2,
		int depthTolerance, int xTolerance, int yTolerance);
	vector<cv::SparseMat> CalculateVolumeObjectDifferencesSparse(const vector<cv::SparseMat>& volumeObjects);
	vector<cv::SparseMat> CalculateVolumeObjectDifferencesSparse(const vector<cv::SparseMat>& volumeObjects, int depthTolerance);

	double AttenuationConstantForAnAction(const vector<cv::SparseMat>& volumeObjects);
	int MagnitudeOfMotion(const cv::SparseMat& sparseMat); 

    cv::SparseMat ConstructVMT(const vector<cv::SparseMat>& volumeObjectDifferences);
	vector<cv::SparseMat> ConstructVMTs(const vector<cv::SparseMat>& volumeObjectDifferences);

	cv::SparseMat CalculateD_Old(cv::SparseMat lastVolumeObject, cv::SparseMat firstVolumeObject);
	cv::SparseMat CalculateD_New(cv::SparseMat lastVolumeObject, cv::SparseMat firstVolumeObject);
	cv::Vec3i CalculateMomentVector(cv::SparseMat volumeObject);

	double CalculateAlpha(cv::Vec3i motionVector);
	double CalculateBeta(cv::Vec3i motionVector);
	double CalculateTheta(cv::Vec3i motionVector);

	cv::Matx33d CalculateRotationX_alpha(double alpha);
	cv::Matx33d CalculateRotationY_beta(double beta);
	cv::Matx33d CalculateRotationZ_theta(double theta);

	cv::SparseMat RotateVMT(const cv::SparseMat& vmt, const cv::Matx33d& rotationMatrix);
	cv::Mat ProjectVMTOntoXY(const cv::SparseMat& vmt);

	void Print3x3Matrix(const cv::Matx33d& mat);
	void Print3DSparseMatrix(cv::SparseMat sparse_mat);
	void Save3DSparseMatrix(cv::SparseMat sparse_mat, string filePath);
	void Save3DMatrix(cv::Mat sparse_mat, string filePath);
};

