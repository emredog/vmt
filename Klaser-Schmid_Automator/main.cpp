#include <QCoreApplication>
#include <QProcess>
#include <QDir>
#include <QHash>
#include <iostream>
#include <unistd.h>
#include "klaserschmidthread.h"
#include "klaserschmidthreadvmtonly.h"

//#include <stdio.h>

using std::cout;
using std::cerr;
using std::endl;

enum KlaserFunction
{
    Generate_Vmt,
    Feature_From_Vmt,
    Rotation_Angle_Only
};



int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    //-----------------------------------------------------------------------------------------------------------------
    // SET MAJOR VARIABLES HERE
    //-----------------------------------------------------------------------------------------------------------------
    KlaserFunction operation = Rotation_Angle_Only;
    bool isGsuData = false;

    QDir dataDir("/home/emredog/LIRIS-data/test");
    QDir trackFileDir("/home/emredog/LIRIS-data/test_tracklets_20140424/track/union/stationary_cam");
    QDir targetDir("...");
    QDir vmtDir("...");
    //------------------------------------------------------------------------------------------------------------------

    if (!targetDir.exists())
        QDir().mkdir(targetDir.absolutePath());
    QDir initialDir = QDir::current();
    QDir::setCurrent("/home/emredog/qt_builds/build-Klaser-Schmid_Hog3D_VMT-Desktop-Release");
    const int threadCount = 2;

    QStringList algoArgs;

    switch(operation)
    {
    case Generate_Vmt:
    {
        // SET KLASER&SCHMID VARIABLES HERE
        algoArgs << "--vmt-only";
        if (isGsuData)
            algoArgs << "--gsu-data";

        cout << "Data directory: " << dataDir.absolutePath().toStdString() << endl;
        QStringList nameFilters;

        //get all image sequences:
        QStringList videos = dataDir.entryList(nameFilters, QDir::AllDirs | QDir::NoDotAndDotDot);
        cout << "Videos fetched: " << videos.count() << endl;

        //get all track files
        nameFilters << "*.track";
        QStringList trackFiles = trackFileDir.entryList(nameFilters, QDir::Files | QDir::NoDotAndDotDot);
        cout << "Track files fetched: " << trackFiles.count() << endl;

        //match track files and videos:
        cout << "Matching track files and videos";

        QHash<QString, QString> videosToTrackFiles;
        foreach (QString video, videos)
        {
            QStringList searchResults = trackFiles.filter(video);
            foreach (QString result, searchResults)
                videosToTrackFiles.insertMulti(video, result);

            cout << ".";
        }

        cout << endl << "Matching completed." << endl;


        QHash<QString, QString>::const_iterator it = videosToTrackFiles.constBegin();


        QList<VideoTrackPair> pairs;

        while (it != videosToTrackFiles.constEnd())
        {
            VideoTrackPair pair;
            pair.first = it.key();
            pair.second = it.value();
            pairs.append(pair);
            ++it;
        }

        int length = pairs.length()/threadCount;

        struct timespec ts = { 5000 / 1000, (5000 % 1000) * 1000 * 1000 };
        nanosleep(&ts, NULL);

        for (int i=0; i<threadCount; i++)
        {
            int startPos = i * length;

            if (i == threadCount - 1) //if it's the last thread
                length = -1; //just take all of the remaining files

            KlaserSchmidThread* proc = new KlaserSchmidThread(pairs.mid(startPos, length), algoArgs, dataDir, trackFileDir, targetDir, i+1);
            proc->start();
        }
        break;

    }
    case Feature_From_Vmt: //we have VMT files already
    {
        // SET KLASER&SCHMID VARIABLES HERE
        algoArgs << "-P" << "icosahedron" //"dodecahedron" icosahedron
                    //<< "--loose-track"
                 << "--xy-stride" <<  "16"  //"16" "32"
                 << "--t-stride" << "16"    //"16" "32"
                 << "--xy-ncells" << "2"
                 << "--t-ncells" << "2"
                 << "--npix" << "2";

        if (isGsuData)
            algoArgs << "--gsu-data";

        cout << "Data directory: " << vmtDir.absolutePath().toStdString() << endl;
        QStringList nameFilters;
        //get all VMT files
        nameFilters << "*.pcd";
        QStringList vmtFiles = vmtDir.entryList(nameFilters, QDir::Files | QDir::NoDotAndDotDot);
        cout << "VMT files fetched: " << vmtFiles.count() << endl;

        int length = vmtFiles.length()/threadCount;

        struct timespec ts = { 5000 / 1000, (5000 % 1000) * 1000 * 1000 };
        nanosleep(&ts, NULL);

        for (int i=0; i<threadCount; i++)
        {
            int startPos = i * length;

            if (i == threadCount - 1) //if it's the last thread
                length = -1; //just take all of the remaining files

            KlaserSchmidThreadFromVMT* proc = new KlaserSchmidThreadFromVMT(vmtFiles.mid(startPos, length), vmtDir, algoArgs, targetDir, i+1);
            proc->start();
        }
        break;
    }
    case Rotation_Angle_Only:
    {
        // SET KLASER&SCHMID VARIABLES HERE
        algoArgs << "--calculate-rotation";
        if (isGsuData)
            algoArgs << "--gsu-data";

        QString program = "./Klaser-Schmid_Hog3D_VMT";

        cout << "Data directory: " << dataDir.absolutePath().toStdString() << endl;
        QStringList nameFilters;

        //get all image sequences:
        QStringList videos = dataDir.entryList(nameFilters, QDir::AllDirs | QDir::NoDotAndDotDot);
        cout << "Videos fetched: " << videos.count() << endl;

        //get all track files
        nameFilters << "*.track";
        QStringList trackFiles = trackFileDir.entryList(nameFilters, QDir::Files | QDir::NoDotAndDotDot);
        cout << "Track files fetched: " << trackFiles.count() << endl;

        //match track files and videos:
        cout << "Matching track files and videos";

        QHash<QString, QString> videosToTrackFiles;
        foreach (QString video, videos)
        {
            QStringList searchResults = trackFiles.filter(video);
            foreach (QString result, searchResults)
                videosToTrackFiles.insertMulti(video, result);

            cout << ".";
        }

        cout << endl << "Matching completed." << endl;


        QHash<QString, QString>::const_iterator it = videosToTrackFiles.constBegin();

        QStringList inputArgs;
        QFileInfo* trackFileInfo;
        int counter = 1;

        QList<VideoTrackPair> pairs;

        while (it != videosToTrackFiles.constEnd())
        {
            VideoTrackPair pair;
            pair.first = it.key();
            pair.second = it.value();
            pairs.append(pair);
            ++it;
        }

        //we don't need multithread functionality for this operation

        QProcess* process;
        QString outputFilePath = initialDir.absoluteFilePath("rotationAngles.txt");

        foreach (VideoTrackPair pair, pairs)
        {
            inputArgs << "--video-file" << dataDir.absoluteFilePath(pair.first)
                      << "-t" << trackFileDir.absoluteFilePath(pair.second);

            //create a new process
            process = new QProcess();
            //set output file name
            trackFileInfo = new QFileInfo(pair.second);
            process->setStandardOutputFile(outputFilePath, QIODevice::Append);

            //run the program
            process->start(program, inputArgs + algoArgs);
            cout << "All parameters:" << endl << "\t"
                 << inputArgs.join(" ").toStdString() << " " << algoArgs.join(" ").toStdString() << endl << endl;

            //check if it started normally
            if (!process->waitForStarted(3000))
            {
                cerr << "Could not start process with following parameters:" << endl
                     << "input: " << inputArgs.join(" ").toStdString() << endl
                     << "parameters: " << algoArgs.join(" ").toStdString() << endl;

                continue;
            }

            //wait 10 minutes for it to finish
            if (!process->waitForFinished(600000)) //wait for 10 minutes
            {
                cerr << "Could not complete process within 10 minutes, with following parameters:" << endl
                     << "input: " << inputArgs.join(" ").toStdString() << endl
                     << "parameters: " << algoArgs.join(" ").toStdString() << endl;

                continue;
            }

            counter++;

            //clean-up
            delete process;
            delete trackFileInfo;
            inputArgs.clear();
        }

        break;
    }
    default:
    {
        std::cerr << "BAD OPERATION TYPE\n\n\n";
        break;
    }
    }
}
