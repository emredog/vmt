#ifndef VMTCALCULATOR_H
#define VMTCALCULATOR_H

#include <map>
#include <QList>

#include "opencv/vmt.h"
#include "geometry/Box.h"

class VmtFunctions;

class VmtCalculator
{
public:
    VmtCalculator(bool isGsuData = false);
    ~VmtCalculator();

    Vmt calculateVmt(std::string imgDir, std::string trackFile);
    QList<float> calculateRotation(std::string imgDir, std::string trackFile);

    bool getIsGsuData() const;
    void setIsGsuData(bool value);

protected:
    VmtFunctions* vmtCore;
    bool isGsuData;
};

#endif // VMTCALCULATOR_H
