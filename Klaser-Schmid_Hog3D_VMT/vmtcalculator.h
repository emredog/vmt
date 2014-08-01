#ifndef VMTCALCULATOR_H
#define VMTCALCULATOR_H

#include <map>

#include "opencv/vmt.h"
#include "geometry/Box.h"

class VmtFunctions;

class VmtCalculator
{
public:
    VmtCalculator();
    ~VmtCalculator();

    Vmt calculateVmt(std::string imgDir, std::string trackFile);

protected:
    VmtFunctions* vmtCore;
};

#endif // VMTCALCULATOR_H
