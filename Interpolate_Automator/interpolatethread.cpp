#include "interpolatethread.h"

#include <QProcess>
#include <iostream>
using namespace std;

InterpolateThread::InterpolateThread(const QDir &dataDir, const QStringList &vmtFiles, int maxSegmentLength, int threadId, QObject *parent) :
    QThread(parent)
{
    this->threadId = threadId;
    this->dataDir = dataDir;
    this->vmtFiles = vmtFiles;
    this->maxSegmentLength = maxSegmentLength;

    this->program = "./Interpolate_VMT";
}

void InterpolateThread::run()
{
    int counter = 1;
    int total = this->vmtFiles.length();
    int errorCounter = 0;

    QProcess* process;
    foreach(QString vmtFileStr, this->vmtFiles)
    {
        //create a new process
        process = new QProcess();

        QStringList arguments;
        //add file name:
        arguments << vmtFileStr;
        //add angles
        arguments << QString::number(maxSegmentLength); //maximum length of the segment to be interpolated

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
                .arg(counter*this->threadId)
                .arg(total-counter).toStdString() <<  endl << endl;
        counter++;

        //clean-up
        delete process;
    }

    cout << "\nThread " << this->threadId << " is completed.\n"
         << "Processed:\t" << counter << endl
         << "Error:\t"     << errorCounter << endl;
}
