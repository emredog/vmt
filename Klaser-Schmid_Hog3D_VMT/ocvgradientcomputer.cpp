#include "ocvgradientcomputer.h"
#include "opencv/vmt.h"
#include "intensitygradientcomputation.h"


OcvGradientComputer::OcvGradientComputer(Vmt *vmt)
    : _vmt(vmt)
{

    cv::SparseMat spMat = this->_vmt->getSparseMat(); //.clone();

    spMat.convertTo(_vmtMat, CV_8UC1);
}

OcvGradientComputer::~OcvGradientComputer()
{
    //TODO
}

OcvGradientComputer::VectorType OcvGradientComputer::getGradientVector(const Box3D &box) const
{
    //initialize a zero vector
    VectorType vec(3); vec[0] = 0.0; vec[1] = 0.0; vec[2] = 0.0;

    //crop VMT with given box
    cv::Mat cube = this->cropVmt(box);

    //return zero vector if cube is not available (not in boundaries of the cell)
    // OR the cube doesnt contain any point
    cv::SparseMat spCube(cube);
    if (spCube.nzcount() <= 0)
    {
        spCube.release();
        cube.release();

        return vec;
    }


    //calculate mean gradient vector:
    cv::Vec3f vec3f = IntensityGradientComputation::computeMeanIntensityGradientDifferentiation(cube, IntensityGradientComputation::Central_Difference);

    //clean up
    cube.release();

    vec[0] = vec3f[0];
    vec[1] = vec3f[1];
    vec[2] = vec3f[2];

    return vec;
}

bool OcvGradientComputer::isInBuffer(const Box3D &box) const
{
    //TODO
    return true;
}

int OcvGradientComputer::getWidth() const
{
    return _vmt->getWidth();
}

int OcvGradientComputer::getHeight() const
{
    return _vmt->getHeight();
}

int OcvGradientComputer::getDepth() const
{
    return _vmt->getDepth();
}

cv::Mat OcvGradientComputer::cropVmt(const Box3D &box) const
{
    //check if we reached the end of the vmt (in any direction)
    if (box.x + box.width  > _vmtMat.size[0] ||
            box.y + box.height > _vmtMat.size[1] ||
            box.z + box.depth  > _vmtMat.size[2])
    {
        int sizes[] = {(int)box.width, (int)box.height, (int)box.depth};
        cv::Mat mat(3, sizes, CV_8UC1);
        mat = cv::Scalar(0);
        return mat;
    }
    cv::Range ranges[3];

    ranges[0] = cv::Range(box.x, box.x + box.width);
    ranges[1] = cv::Range(box.y, box.y + box.height);
    ranges[2] = cv::Range(box.z, box.z + box.depth);

    return _vmtMat(ranges).clone();
}

