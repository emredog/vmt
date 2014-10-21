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
    QCoreApplication app(argc, argv);

    const QString codebookStr = "args16_K-Means_s100K_k1000_C100_e0.1.out";
    const QDir activityFeaturesDir("/home/emredog/gsu-data/test_features_args16/camera2");
    const QString codeBookFilePath = QString("/home/emredog/gsu-data/Codebook/%1").arg(codebookStr);
    const QString targetDir = QString("/home/emredog/gsu-data/test_BagOfWords/camera2");

    //create missing target directory
    {
        QDir targetDirectory(targetDir);

        if (!targetDirectory.exists())
            targetDirectory.mkpath(targetDir);
    }

    const int ignoreFeatsOnActs = 8; //number of ignored features in the beginning of each line
    const int vocabularySize = 1000; //K of K-means    4000 or 1000
    const int featureDim = 80;// 48<--- dodecahedron //80 <--- icosahedron
    const int threadCount = 1;

    //read Code book:
    FeatureSet codeBook(codeBookFilePath, featureDim);


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
