#include <QCoreApplication>
#include <QDir>
#include <iostream>
#include <qmath.h>
#include <QVector>
#include <QTextStream>

#include "featureset.h"

using namespace std;

float calculateSquaredEuclidDistance(const QList<float> &featVect1, const QList<float> &featVect2);
int findNearestCodeword(const FeatureSet &codebook, const QList<float> &featVect);
QVector<int> getBagOfWords(const FeatureSet &codeBook, const FeatureSet& activity, int vocabularySize);
void writeBoWtoFile(QVector<int> &codeWords, QString fileName);

int main(/*int argc, char *argv[]*/)
{
    const QDir activityFeaturesDir("/home/emredog/LIRIS-data/training-validation_features/");
    const QString codeBookFilePath("/home/emredog/LIRIS-data/CodeBooks/K-Means_s500K_k4000_C100_e0.1.out");
    const int ignoreFeatsOnActs = 8; //number of ignored features in the beginning of each line
    const int vocabularySize = 4000;

    //read Code book:
    FeatureSet codeBook(codeBookFilePath);

    QStringList filters;
    filters << "*.out";
    QStringList activityFiles = activityFeaturesDir.entryList(filters, QDir::Files | QDir::NoDotAndDotDot);

    cout << "Obtained " << activityFiles.length() << " activity files from " << activityFeaturesDir.absolutePath().toStdString() << endl;

    cout << "Starting to assign features to nearest codewords..." << endl << endl;

    int stopAfter = -1;
    int counter = 0;

    foreach(QString actFile, activityFiles)
    {
        counter++;
        if (stopAfter > 0 && counter > stopAfter)
            break;


        FeatureSet activity(activityFeaturesDir.absoluteFilePath(actFile), ignoreFeatsOnActs);
        if (activity.getNrOfFeatures() <= 0)
            std::cout << "[ERROR] No feature is read from " << actFile.toStdString() << std::endl;

        QVector<int> codeWords = getBagOfWords(codeBook, activity, vocabularySize);
        writeBoWtoFile(codeWords, actFile.append(".BoW"));
        std::cout << "Processed: " << counter << std::endl;
    }


}

float calculateSquaredEuclidDistance(const QList<float> &featVect1, const QList<float> &featVect2)
{
    if (featVect1.length() != featVect2.length())
        return -1.0;

    float sum = 0.0;

    for (int i=0; i<featVect1.length(); i++) sum += qPow(featVect1[i] - featVect2[i], 2);

    return sum;
}

int findNearestCodeword(const FeatureSet &codebook, const QList<float> &featVect)
{
    float minDistance = 100000000.0; //a ridiculously big float
    float curDistance = 0.0;
    int indexOfNearest = -1;

    const QList<QList<float> > vocabulary = codebook.getFeatureVectors();

    for (int i=0; i<vocabulary.length(); i++)
    {
        const QList<float> codeWord = vocabulary[i];
        curDistance = calculateSquaredEuclidDistance(codeWord, featVect);
        if (curDistance < minDistance) //found a more near word
        {
            minDistance = curDistance;
            indexOfNearest = i;
        }
    }

    return indexOfNearest;
}

QVector<int> getBagOfWords(const FeatureSet& codeBook, const FeatureSet& activity, int vocabularySize)
{
    QVector<int> codeWords(vocabularySize, 0); //initialize a vector of vocabularySize, with value 0
    const QList<QList<float> > featureVectors = activity.getFeatureVectors();
    for(int i=0; i<featureVectors.length(); i++)
    {
        codeWords[findNearestCodeword(codeBook, featureVectors[i])]++; //increment the codeword count for the given feature vector
    }

    return codeWords;
}

void writeBoWtoFile(QVector<int> &codeWords, QString fileName)
{
    if (codeWords.isEmpty())
    {
        std::cout << "[ERROR] No words for:" << fileName.toStdString() << std::endl;
        return;
    }

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        std::cout << "[ERROR] Can't open file for writing:" << fileName.toStdString() << std::endl;
        return;
    }

    QTextStream out(&file);
    for (int i=0; i<codeWords.size(); i++) out << codeWords[i] << "\n";

}
