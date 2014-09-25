#include "klaserschmidthreadvmtonly.h"
#include <QProcess>
#include <QFileInfo>

#include <iostream>
using namespace std;

KlaserSchmidThreadFromVMT::KlaserSchmidThreadFromVMT(QStringList vmtFiles, QDir vmtDir, QStringList algoArgs, QDir targetDir, int threadId, QObject *parent) :
    QThread(parent)
{
    this->program = "./Klaser-Schmid_Hog3D_VMT";

    this->vmtDir = vmtDir;
    this->targetDir = targetDir;
    this->vmtFiles = vmtFiles;
    this->algoArgs = algoArgs;
    this->threadId = threadId;
}

void KlaserSchmidThreadFromVMT::run()
{
    QStringList inputArgs;
    QFileInfo* trackFileInfo;
    int counter = 1;

    int total = this->vmtFiles.length();

    QProcess* process;

    foreach (QString vmtFile, vmtFiles)
    {
        //set input arguments with vmt file name
        inputArgs << "--vmt-file" << vmtDir.absoluteFilePath(vmtFile);

        //create a new process
        process = new QProcess();
        //set output file name
        trackFileInfo = new QFileInfo(vmtFile);
        process->setStandardOutputFile(targetDir.absoluteFilePath(trackFileInfo->baseName() + ".out"), QIODevice::Truncate);

        //run the program
        process->start(program, inputArgs + algoArgs);
        cout << "[" <<  this->thread()->currentThreadId() <<  "] All parameters:" << endl << "\t"
             << inputArgs.join(" ").toStdString() << " " << algoArgs.join(" ").toStdString() << endl << endl;

        //check if it started normally
        if (!process->waitForStarted(3000))
        {
            cerr << "Could not start process with following parameters:" << endl
                 << "input: " << inputArgs.join(" ").toStdString() << endl
                 << "parameters: " << algoArgs.join(" ").toStdString() << endl
                 << "output: " << targetDir.absoluteFilePath(trackFileInfo->baseName() + ".out").toStdString() << endl;

            continue;
        }

        QString txt = QString("[%1] Process %2 started...\n\tExpected output file: %3").arg(this->threadId)
                                                                             .arg(counter)
                                                                             .arg(targetDir.absoluteFilePath(trackFileInfo->baseName() + ".out"));
        cout << txt.toStdString() << endl << endl;

        //wait 10 minutes for it to finish
        if (!process->waitForFinished(600000)) //wait for 10 minutes
        {
            cerr << "Could not complete process within 10 minutes, with following parameters:" << endl
                 << "input: " << inputArgs.join(" ").toStdString() << endl
                 << "parameters: " << algoArgs.join(" ").toStdString() << endl
                 << "output: " << targetDir.absoluteFilePath(trackFileInfo->baseName() + ".out").toStdString() << endl;

            continue;
        }

        txt = QString("[%1] Process %2 completed. Remaining: %3").arg(this->threadId).arg(counter).arg(total-counter);
        cout << txt.toStdString() <<  endl << endl;
        counter++;

        //clean-up
        delete process;
        delete trackFileInfo;
        inputArgs.clear();
    }
}
