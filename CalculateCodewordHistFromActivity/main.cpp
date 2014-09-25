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

    const QString codebookStr = "args32_K-Means_s199K_k4000_C100_e0.1.out";
    const QDir activityFeaturesDir("/home/emredog/LIRIS-data/test_features/20140921_args32x32x32-2x2x2_icosa");
    const QString codeBookFilePath = QString("/home/emredog/LIRIS-data/Codebooks/%1").arg(codebookStr);
    const QString targetDir = QString("/home/emredog/LIRIS-data/test_BagOfWords/args32_S199k_K4000/");

    //create missing target directory
    {
        QDir targetDirectory(targetDir);

        if (!targetDirectory.exists())
            targetDirectory.mkpath(targetDir);
    }

    const int ignoreFeatsOnActs = 8; //number of ignored features in the beginning of each line
    const int vocabularySize = 4000; //K of K-means    4000 or 1000
    const int featureDim = 80;// 48<--- dodecahedron //80 <--- icosahedron
    const int threadCount = 4;

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
