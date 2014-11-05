#include <QCoreApplication>
#include <QDir>
#include <iostream>

#include "featureset.h"
#include "calculateprocess.h"

using namespace std;

float calculateSquaredEuclidDistance(const QList<float> &featVect1, const QList<float> &featVect2);
int findNearestCodeword(const FeatureSet &codebook, const QList<float> &featVect);
QVector<int> getBagOfWords(const FeatureSet &codeBook, const FeatureSet& activity, int vocabularySize);
void writeBoWtoFile(QVector<int> &codeWords, QString fileName);

int main(int argc, char *argv[])
{
    bool isCodeBookContainsIndex = true;
    QCoreApplication app(argc, argv);

    const QString codebookStr = "AllFeaturesInSingleFile-Formatted-500k.features-K1000.cluster_centres";
    const QDir activityFeaturesDir("/home/emredog/LIRIS-data/test_features/20141102-rot-int_args16");
    const QString codeBookFilePath = QString("/home/emredog/LIRIS-data/Codebooks/20141103/%1").arg(codebookStr);
    const QString targetDir = QString("/home/emredog/LIRIS-data/test_BagOfWords/20141103-fromParallelKMeans-K1000");

    //create missing target directory
    {
        QDir targetDirectory(targetDir);

        if (!targetDirectory.exists())
            targetDirectory.mkpath(targetDir);
    }

    const int ignoreFeatsOnActs = 8; //number of ignored features in the beginning of each line
    const int vocabularySize = 1000; //K of K-means    4000 or 1000
    const int featureDim = 80;// 48<--- dodecahedron //80 <--- icosahedron
    const int threadCount = 4;

    //read Code book:
    int ignoreFeatsOnCodebook = isCodeBookContainsIndex ? 1 : 0;
    FeatureSet codeBook(codeBookFilePath, featureDim, ignoreFeatsOnCodebook);


    QStringList filters;
    filters << "*.out";
    QStringList activityFiles = activityFeaturesDir.entryList(filters, QDir::Files | QDir::NoDotAndDotDot);

    cout << "Obtained " << activityFiles.length() << " activity files from " << activityFeaturesDir.absolutePath().toStdString() << endl;

    cout << "Starting to assign features to nearest codewords with " << threadCount << "threads..." << endl << endl;

    int length = activityFiles.length()/threadCount;
    QList<CalculateProcess*> processes;
    for (int i=0; i<threadCount; i++)
    {
        int startPos = i * length;

        if (i == threadCount - 1) //if it's the last thread
            length = -1; //just take all of the remaining files

        CalculateProcess* proc = new CalculateProcess(activityFiles.mid(startPos, length), activityFeaturesDir, codeBook, vocabularySize, featureDim, ignoreFeatsOnActs);
        processes.append(proc);
        proc->setTargetDir(targetDir);
        proc->start();
    }

    return app.exec();


}
