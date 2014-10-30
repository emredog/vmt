#include "vmtcalculator.h"

#include "vmt_calculation/VmtFunctions.h"
#include "vmt_calculation/PointCloudFunctions.h"

VmtCalculator::VmtCalculator(bool isGsuData)
{
    this->vmtCore = new VmtFunctions(isGsuData); //default size: 640*480
    this->vmtCore->setSavedObjects(false, false, false);
    this->vmtCore->setDownsampleRate(1); //no downsampling
    this->isGsuData = isGsuData;
}

VmtCalculator::~VmtCalculator()
{
    if (this->vmtCore)
        delete this->vmtCore;
}

Vmt VmtCalculator::calculateVmt(std::string imgDir, std::string trackFile)
{
    //calculate VMT
    cv::SparseMat vmt = this->vmtCore->constructSparseVMT(QString::fromStdString(imgDir), QString::fromStdString(trackFile));

    //apply statistical outlier filter from PCL
    cv::SparseMat filtered = PointCloudFunctions::statisticalOutlierRemoval(vmt);
    vmt.release();

    //spatially normalize: Z is scaled between 0 and 2000
    cv::SparseMat normalized = vmtCore->spatiallyNormalizeSparseMat(filtered);
    filtered.release();

    //trim the sparsemat
    cv::SparseMat trimmed = vmtCore->trimSparseMat(normalized);
    normalized.release();



    return Vmt(trimmed);
}

QList<float> VmtCalculator::calculateRotation(string imgDir, string trackFile)
{
    return this->vmtCore->calculateRotationAngles(QString::fromStdString(imgDir), QString::fromStdString(trackFile));
}
bool VmtCalculator::getIsGsuData() const
{
    return isGsuData;
}

void VmtCalculator::setIsGsuData(bool value)
{
    isGsuData = value;
}

