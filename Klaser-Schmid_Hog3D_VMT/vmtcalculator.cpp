#include "vmtcalculator.h"

#include "vmt_calculation/VmtFunctions.h"
#include "vmt_calculation/PointCloudFunctions.h"

VmtCalculator::VmtCalculator()
{
    this->vmtCore = new VmtFunctions(); //default size: 640*480
    this->vmtCore->setSavedObjects(false, false, false);
    this->vmtCore->setDownsampleRate(1); //no downsampling
}

VmtCalculator::~VmtCalculator()
{
    if (this->vmtCore)
        delete this->vmtCore;
}

Vmt VmtCalculator::calculateVmt(std::string imgDir, std::string trackFile)
{
    cv::SparseMat vmtSparseMat = this->vmtCore->constructSparseVMT(QString::fromStdString(imgDir), QString::fromStdString(trackFile));
    //TODO
    return Vmt();
}
