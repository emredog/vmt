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



int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    //-----------------------------------------------------------------------------------------------------------------
    // SET MAJOR VARIABLES HERE
    //-----------------------------------------------------------------------------------------------------------------
    QDir dataDir("/home/emredog/gsu-data/test/camera2");
    QDir trackFileDir("/home/emredog/gsu-data/test_tracks/camera2");
    QDir targetDir("/home/emredog/gsu-data/test_features_args16/camera2");
    QDir vmtDir("/home/emredog/gsu-data/test_vmts/camera2");

    bool fromVMT = true;
    //------------------------------------------------------------------------------------------------------------------

    if (!targetDir.exists())
        QDir().mkdir(targetDir.absolutePath());
    QDir::setCurrent("/home/emredog/qt_builds/build-Klaser-Schmid_Hog3D_VMT-Desktop-Release");
    const int threadCount = 2;


    //-----------------------------------------------------------------------------------------------------------------
    // SET KLASER&SCHMID VARIABLES HERE
    //-----------------------------------------------------------------------------------------------------------------
    QStringList algoArgs;
    if (fromVMT)
    {
        algoArgs << "-P" << "icosahedron" //"dodecahedron" icosahedron
                    //<< "--loose-track"
                 << "--xy-stride" <<  "16"  //"16" "32"
                 << "--t-stride" << "16"    //"16" "32"
                 << "--xy-ncells" << "2"
                 << "--t-ncells" << "2"
                 << "--npix" << "2";
    }
    else
    {
        // SET IF VMT ONLY
        algoArgs << "--vmt-only";
    }

    //-----------------------------------------------------------------------------------------------------------------

    if (!fromVMT)
    {
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

    }
    else //we have VMT files already
    {
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
    }

    return a.exec();
}
