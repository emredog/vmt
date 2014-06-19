#include "VmtFunctions.h"



int main()
{


    QString videoPath = "/home/emredog/LIRIS-data/training-validation/vid0002";
    QString trackFile = "/home/emredog/LIRIS-data/training-validation_annotations-with-NO-ACTION-SLIDING_WINDOWS/vid0002_1_discussion_(1-41).track";
    int downsamplingRate = 1; // --> no downsampling


    VmtFunctions* vmtCore = new VmtFunctions();
    cv::SparseMat vmt = vmtCore->GenerateSparseVMT(videoPath, trackFile, downsamplingRate);

}
