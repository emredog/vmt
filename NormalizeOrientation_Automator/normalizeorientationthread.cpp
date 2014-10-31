#include "normalizeorientationthread.h"
#include <QProcess>

#include <iostream>
using namespace std;

NormalizeOrientationThread::NormalizeOrientationThread(const QList<PcdAnglesPair> &vmtsToAngles, int threadId, QObject *parent) :
    QThread(parent)
{
    this->threadId = threadId;
    this->vmtsToAngles = vmtsToAngles;

    this->program = "./NormalizeOrientation_VMT";
}

void NormalizeOrientationThread::run()
{
    int counter = 1;
    int total = this->vmtsToAngles.length();
    int errorCounter = 0;


    QProcess* process;
    foreach(PcdAnglesPair vmtAngle, this->vmtsToAngles)
    {
        //create a new process
        process = new QProcess();

        QStringList arguments;
        //add file name:
        arguments << vmtAngle.first;
        //add angles
        arguments << QString::number(vmtAngle.second[0], 'g', 6); //rotation amount around X axis
        arguments << QString::number(vmtAngle.second[1], 'g', 6); //rotation amount around Y axis
        arguments << QString::number(vmtAngle.second[2], 'g', 6); //rotation amount around Z axis

        //run the program
        process->start(program, arguments);

        //check if it started normally
        if (!process->waitForStarted(3000))
        {
            cerr << "Could not start process with following parameters:" << endl
                 << "\t" << arguments.join(" ").toStdString() << endl << endl;


            errorCounter++;
            continue;
        }

        //print info
        cout << QString("[%1] Process %2 started...").arg(this->threadId).arg(counter*this->threadId).toStdString() << endl;

        //wait 10 minutes for it to finish
        if (!process->waitForFinished(600000)) //wait for 10 minutes
        {
            cerr << "Could not complete process within 10 minutes, with following parameters:" << endl
                 << "\t" << arguments.join(" ").toStdString() << endl << endl;

            errorCounter++;
            continue;
        }

        cout << QString("[%1] Process %2 completed. Remaining: %3").arg(this->threadId)
                                                                   .arg(counter)
                                                                   .arg(total-counter).toStdString() <<  endl << endl;
        counter++;

        //clean-up
        delete process;
    }

    cout << "Thread " << this->threadId << " is completed.\n"
         << "Processed:\t" << counter << endl
         << "Error:\t"     << errorCounter << endl;


}

