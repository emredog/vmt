#include <QCoreApplication>
#include <QFile>
#include <QDir>
#include <QTextStream>
#include <iostream>
using namespace std;

#include "interpolatethread.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    //-----------------------------------------------------------------------------------------------------------------
    // SET MAJOR VARIABLES HERE
    //-----------------------------------------------------------------------------------------------------------------
    QDir dataDir("/home/emredog/LIRIS-data/test_VMTs_20140916-ROTATED");
    int threadCount = 3;
    int maxSegmentLength = 20;
    //-----------------------------------------------------------------------------------------------------------------

    QDir::setCurrent("/home/emredog/qt_builds/build-Interpolate_VMT-Desktop-Release");

    //print info
    cout << "Data directory: " << dataDir.absolutePath().toStdString() << endl;
    QStringList nameFilters;
    nameFilters << "*.pcd";

    //get all vmts:
    QStringList vmtList = dataDir.entryList(nameFilters, QDir::Files | QDir::NoDotAndDotDot);
    if (vmtList.isEmpty())
    {
        cout << "No PCD files found in:\n" << dataDir.absolutePath().toStdString() << endl;
        return -1;
    }
    cout << "VMTs fetched: " << vmtList.count() << endl;

    //divide the workload into threadcount:
    int length = vmtList.length()/threadCount;


    for (int i=0; i<threadCount; i++)
    {
        int startPos = i * length;

        if (i == threadCount - 1) //if it's the last thread
            length = -1; //just take all of the remaining files

        InterpolateThread* proc = new InterpolateThread(dataDir, vmtList.mid(startPos, length), maxSegmentLength, i+1);
        proc->start();
    }

    return a.exec();
}
