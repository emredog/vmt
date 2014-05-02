#include <QCoreApplication>
#include <QProcess>
#include <QDir>
#include <QHash>
#include <iostream>
#include <unistd.h>
#include "klaserschmidthread.h"

//#include <stdio.h>

using std::cout;
using std::cerr;
using std::endl;



int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    //-----------------------------------------------------------------------------------------------------------------
    QDir dataDir("/home/emredog/LIRIS-data/test/");
    QDir trackFileDir("/home/emredog/LIRIS-data/test_tracklets_20140424/");
    QDir targetDir("/home/emredog/LIRIS-data/test_features/test_withSlidingWindows_params03");
    QDir::setCurrent("/home/emredog/qt_builds/build-Klaser-Schmid_Hog3D_qt-Desktop-Release/");    

    const int threadCount = 2;

    QStringList algoArgs;
    algoArgs << "-P" <<  "dodecahedron"  //"icosahedron"
             << "--loose-track"
             << "--xy-stride" <<  "16"
             << "--t-stride" << "16"
             << "--xy-ncells" << "2"
             << "--t-ncells" << "2"
             << "--xy-scale" << "1"
             << "--t-scale" << "1"
             << "--scale-overlap" << "1"
             << "--npix" << "2";
    //-----------------------------------------------------------------------------------------------------------------

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

    for (int i=0; i<threadCount; i++)
    {
        int startPos = i * length;

        if (i == threadCount - 1) //if it's the last thread
            length = -1; //just take all of the remaining files

        KlaserSchmidThread* proc = new KlaserSchmidThread(pairs.mid(startPos, length), algoArgs, dataDir, trackFileDir, targetDir);
        proc->start();
    }

    return a.exec();
}
