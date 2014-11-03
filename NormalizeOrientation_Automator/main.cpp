#include <QCoreApplication>
#include <QFile>
#include <QDir>
#include <QTextStream>
#include <iostream>
using namespace std;

#include "normalizeorientationthread.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    //-----------------------------------------------------------------------------------------------------------------
    // SET MAJOR VARIABLES HERE
    //-----------------------------------------------------------------------------------------------------------------
    QFile rotationAnglesFile("/home/emredog/LIRIS-data/RotationAngles-test_StationaryUnions.txt");
    QDir dataDir("/home/emredog/LIRIS-data/test_VMTs_20140916");
    bool rotateOnlyAroundYAxis = true;
    int threadCount = 3;
    //-----------------------------------------------------------------------------------------------------------------

    QDir::setCurrent("/home/emredog/qt_builds/build-NormalizeOrientation_VMT-Desktop-Release");

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

    //check the file:
    if (!rotationAnglesFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        cerr << "Cannot open rotation angle file!!!\n";
        return -1;
    }

    //parse all text
    QTextStream txtStream(&rotationAnglesFile);
    QList<float> angles;
    bool ok;
    float angle;
    QList<PcdAnglesPair> vmtsToAngles;

    while(!txtStream.atEnd())
    {
        QString line = txtStream.readLine();
        QStringList parts = line.split(";");
        QString trackFile = parts[0].split("/").last();
        QString searchStr = trackFile.left(trackFile.indexOf("Union"));
        QStringList filtered = vmtList.filter(searchStr);

        if (filtered.isEmpty())
        {
            //not found
            continue;
        }
        else if (filtered.length() > 1)
        {
            //multiple hits??
            cerr << "Multiple hits found for: " << searchStr.toStdString() << endl;
            continue;
        }

        PcdAnglesPair vmtToAngle;
        vmtToAngle.first = dataDir.absoluteFilePath(filtered.first());

        //get angle for X axis:
        if (rotateOnlyAroundYAxis)
        {
            angles << 0.0;
        }
        else
        {
            angle = parts[1].toFloat(&ok);
            if (ok)
            {
                angles << angle;
            }
            else
            {
                cerr << "Float parse error for " << parts[0].toStdString();
                angles.clear();
                continue;
            }

        }
        //get angle for Y Axis
        angle = parts[2].toFloat(&ok);
        if (ok)
        {
            angles << angle;
        }
        else
        {
            cerr << "Float parse error for " << parts[0].toStdString();
            angles.clear();
            continue;
        }
        //get angle for Z axis:
        if (rotateOnlyAroundYAxis)
        {
            angles << 0.0;
        }
        else
        {
            angle = parts[3].toFloat(&ok);
            if (ok)
            {
                angles << angle;
            }
            else
            {
                cerr << "Float parse error for " << parts[0].toStdString();
                angles.clear();
                continue;
            }

        }

        vmtToAngle.second = angles;
        vmtsToAngles.append(vmtToAngle);

        angles.clear();

    }

    //divide the workload into threadcount:
    int length = vmtsToAngles.length()/threadCount;


    for (int i=0; i<threadCount; i++)
    {
        int startPos = i * length;

        if (i == threadCount - 1) //if it's the last thread
            length = -1; //just take all of the remaining files

        NormalizeOrientationThread* proc = new NormalizeOrientationThread(vmtsToAngles.mid(startPos, length), i+1);
        proc->start();
    }

    a.exec();

}
