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

    const QDir activityFeaturesDir("/home/emredog/LIRIS-data/training-validation_features/training-validation_features_params03");
    const QString codeBookFilePath("/home/emredog/LIRIS-data/CodeBooks/CodeBook_with_featExtractionParams_03/K-Means_s500K_k4000_C100_e0.1.out");
    const int ignoreFeatsOnActs = 8; //number of ignored features in the beginning of each line
    const int vocabularySize = 4000;
    const int featureDim = 48;
    const int threadCount = 3;

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
        proc->start();
    }


    return app.exec();


}
