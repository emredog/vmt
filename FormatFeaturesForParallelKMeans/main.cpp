#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QStringList>
#include <QProcess>
#include <QSet>

#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
    //-------------------------------------------------------------------------------
    // SET VARIABLES HERE
    const int nrOfUnwantedFeats = 8;
    const int randomFeatSize = 100000; //100000 500000
    //-------------------------------------------------------------------------------

    QCoreApplication a(argc, argv);

    QFile file("/home/emredog/LIRIS-data/training-validation_features/20141102-rot-int_args16/balanced/AllFeaturesInSingleFile.features");
    QFileInfo fileInfo(file);
    QFile outputFile(QString("%1/%2-Formatted-%3k.features").arg(fileInfo.absolutePath()).arg(fileInfo.baseName()).arg(randomFeatSize/1000));

    if (!file.open(QIODevice::ReadOnly))
    {
        std::cerr << "CANT OPEN FILE: " << file.fileName().toStdString();
        return -1;
    }

    bool ok = true;

    //get number of features:
    QProcess p;
    p.start("wc", QStringList() << "-l" << file.fileName());
    p.waitForReadyRead(10000);

    QString stdout = p.readAllStandardOutput();
    stdout = stdout.split(" ").first();

    int totNumberOfFeats = stdout.toInt(&ok);
    if (!ok)
    {
        cerr << "COULD NOT READ THE LINE COUNT OF THE FILE: " << file.fileName().toStdString() << endl << endl;
        return EXIT_FAILURE;
    }

    //generate random indices to run clustering on
    cout << "Generating random indices...\n";
    QSet<int> indices;
    qsrand(qrand());
    while (indices.size() < randomFeatSize)
    {
        int newIndice = qrand() % totNumberOfFeats;
        if (!indices.contains(newIndice))
        {
            indices.insert(newIndice);
        }
    }
    cout << "Number generation completed.\n";

    QTextStream in(&file);

    int lineCounter = 0;
    int lineCounterInNewFile = 1;

    QString line;
    QStringList fields;

    if (!outputFile.open(QIODevice::Truncate | QIODevice::WriteOnly))
    {
        std::cerr << "CANT OPEN FILE: " << outputFile.fileName().toStdString() << endl;
        return -1;
    }

    QTextStream out(&outputFile);


    cout << "Starting to generate output file...\n";
    while (!in.atEnd())
    {
        line = in.readLine();
        if (indices.contains(lineCounter))
        {
            fields = line.split(" ");
            //remove the unwanted dims
            for (int i=0; i<nrOfUnwantedFeats; i++) fields.removeFirst();
            out << QString("%1 %2").arg(lineCounterInNewFile).arg(fields.join(" ")) << endl;
            lineCounterInNewFile++;
        }

        lineCounter++;

    }
    cout << "Completed.\n";

    return EXIT_SUCCESS;
}
