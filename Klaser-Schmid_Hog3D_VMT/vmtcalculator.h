#ifndef VMTCALCULATOR_H
#define VMTCALCULATOR_H

#include <map>

#include "opencv/vmt.h"
#include "geometry/Box.h"

class VmtCalculator
{
public:
    VmtCalculator();

    Vmt calculateVmt(std::string imgDir, std::multimap<int, Box<double> > track);
};

#endif // VMTCALCULATOR_H
