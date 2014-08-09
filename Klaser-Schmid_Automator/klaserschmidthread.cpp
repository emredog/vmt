#include "klaserschmidthread.h"
#include <QProcess>
#include <QFileInfo>

#include <iostream>
using namespace std;

KlaserSchmidThread::KlaserSchmidThread(QList<VideoTrackPair> pairs, QStringList algoArgs,
                                       QDir dataDir, QDir trackFileDir, QDir targetDir, int threadId, QObject *parent) :
    QThread(parent)
{
    this->program = "./Klaser-Schmid_Hog3D_VMT";
    this->dataDir = dataDir;
    this->trackFileDir = trackFileDir;
    this->targetDir = targetDir;
    this->pairs = pairs;
    this->algoArgs = algoArgs;
    this->threadID = threadId;
}

void KlaserSchmidThread::run()
{
    QStringList inputArgs;
    QFileInfo* trackFileInfo;
    int counter = 1;

    int total = this->pairs.length();


    QProcess* process;

    foreach (VideoTrackPair pair, pairs)
    {
        //set input arguments with video name and track file name
        inputArgs << "--video-file" << dataDir.absoluteFilePath(pair.first)
                 << "-t" << trackFileDir.absoluteFilePath(pair.second);

        //create a new process
        process = new QProcess();
        //set output file name
        trackFileInfo = new QFileInfo(pair.second);
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

        QString txt = QString("[%1] Process %2 started...\n\tExpected output file: %3").arg(this->threadID)
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

        txt = QString("[%1] Process %2 completed. Remaining: %3").arg(this->threadID).arg(counter).arg(total-counter);
        cout << txt.toStdString() <<  endl << endl;
        counter++;

        //clean-up
        delete process;
        delete trackFileInfo;
        inputArgs.clear();
    }
}


