#include <QCoreApplication>
#include <QProcess>
#include <QDir>
#include <QHash>
#include <iostream>
//#include <stdio.h>

using std::cout;
using std::cerr;
using std::endl;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    //-----------------------------------------------------------------------------------------------------------------
    QDir dataDir("/home/emredog/LIRIS-data/training-validation/");
    QDir targetDir("/home/emredog/LIRIS-data/training-validation_features/");
    QDir::setCurrent("/home/emredog/qt_builds/build-Klaser-Schmid_Hog3D_qt-Desktop-Release/");
    QString program = "./Klaser-Schmid_Hog3D_qt";

    QStringList algoArgs;
    algoArgs << "-P" <<  "icosahedron"
             << "--loose-track"
             << "--xy-stride" <<  "16"
             << "--t-stride" << "16"
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
    QStringList trackFiles = dataDir.entryList(nameFilters, QDir::Files | QDir::NoDotAndDotDot);
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
    int total = videosToTrackFiles.count();

    QProcess* process;

    while (it != videosToTrackFiles.constEnd())
    {
        //set input arguments with video name and track file name
        inputArgs << "--video-file" << dataDir.absoluteFilePath(it.key())
                 << "-t" << dataDir.absoluteFilePath(it.value());

        //create a new process
        process = new QProcess();
        //set output file name
        trackFileInfo = new QFileInfo(it.value());
        process->setStandardOutputFile(targetDir.absoluteFilePath(trackFileInfo->baseName() + ".out"), QIODevice::Truncate);

        //run the program
        process->start(program, inputArgs + algoArgs);

        //check if it started normally
        if (!process->waitForStarted(-1))
        {
            cerr << "Could not start process with following parameters:" << endl
                 << "input: " << inputArgs.join(" ").toStdString() << endl
                 << "parameters: " << algoArgs.join(" ").toStdString() << endl
                 << "output: " << targetDir.absoluteFilePath(trackFileInfo->baseName() + ".out").toStdString() << endl;

            continue;
        }

        cout << "Process " << counter << " started..." << endl;
        cout << "\tExpected output file: " << targetDir.absoluteFilePath(trackFileInfo->baseName() + ".out").toStdString() << endl << endl;

        //wait 10 minutes for it to finish
        if (!process->waitForFinished(600000)) //wait for 10 minutes
        {
            cerr << "Could not complete process within 10 minutes, with following parameters:" << endl
                 << "input: " << inputArgs.join(" ").toStdString() << endl
                 << "parameters: " << algoArgs.join(" ").toStdString() << endl
                 << "output: " << targetDir.absoluteFilePath(trackFileInfo->baseName() + ".out").toStdString() << endl;

            continue;
        }

        cout << "Process " << counter << " completed. Remaining: " << total-counter << endl << endl;
        counter++;

        it++;

        //clean-up
        delete process;
        delete trackFileInfo;
        inputArgs.clear();
    }




    //TODO: set arguments


    //TODO: manage process starts / ends etc.
    //QProcess *myProcess = new QProcess();
    //myProcess->start(program, arguments);

    return a.exec();
}
