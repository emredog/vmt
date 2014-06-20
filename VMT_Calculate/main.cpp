#include "VmtFunctions.h"



int main()
{


    QString videoPath = "/home/emredog/LIRIS-data/training-validation/vid0025";
    QString trackFile = "/home/emredog/LIRIS-data/training-validation_annotations-with-NO-ACTION-SLIDING_WINDOWS/vid0025_1_unlock-enter_(110-150).track";
    int downsamplingRate = 1; // --> no downsampling


    VmtFunctions* vmtCore = new VmtFunctions();
    cv::SparseMat vmt = vmtCore->GenerateSparseVMT(videoPath, trackFile, downsamplingRate);

}
