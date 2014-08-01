#ifndef DEPTHTOTOLERANCE_H
#define DEPTHTOTOLERANCE_H

#include <QHash>

class DepthToTolerance
{
public:
    DepthToTolerance();
    ~DepthToTolerance();
    int GetTolerance(int depthInMm);
    QList<int> allDepthValues;

protected:
    QHash<int, int> depthToTolerance;
};

#endif // DEPTHTOTOLERANCE_H
