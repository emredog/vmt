#include "calculateprocess.h"

#include <iostream>
#include <QVector>
#include <qmath.h>
#include <QTextStream>

CalculateProcess::CalculateProcess(QStringList activityFiles, QDir activityFeaturesDir, FeatureSet codebook,
                                   int vocabularySize, int featDim, int ignoreFeatsOnActs, QObject *parent) :
    QThread(parent)
{
    this->activityFiles = activityFiles;
    this->activityFeaturesDir = activityFeaturesDir;
    this->vocabularySize = vocabularySize;
    this->featDim = featDim;
    this->ignoreFeatsOnActs = ignoreFeatsOnActs;
    this->codeBook = codebook;
    this->targetDir = QDir(".");
}

CalculateProcess::~CalculateProcess()
{

}

void CalculateProcess::run()
{
    int counter = 0;

    foreach(QString actFile, this->activityFiles)
    {
        counter++;

        FeatureSet activity(this->activityFeaturesDir.absoluteFilePath(actFile), this->featDim, this->ignoreFeatsOnActs);
        if (activity.getNrOfFeatures() <= 0)
            std::cout << "[" << QThread::currentThreadId() << "][ERROR] No feature is read from " << actFile.toStdString() << std::endl;

        QVector<int> codeWords = getBagOfWords(codeBook, activity, this->vocabularySize);
        writeBoWtoFile(codeWords, actFile.append(".BoW"));
        std::cout << "Processed: " << counter << std::endl;
    }
    std::cout << "\n\nFinished for this thread.\n";
}

float CalculateProcess::calculateSquaredEuclidDistance(const QList<float> &featVect1, const QList<float> &featVect2)
{
    if (featVect1.length() != featVect2.length())
        return -1.0;

    float sum = 0.0;

    for (int i=0; i<featVect1.length(); i++) sum += qPow(featVect1[i] - featVect2[i], 2);

    return sum;
}

int CalculateProcess::findNearestCodeword(const FeatureSet &codebook, const QList<float> &featVect)
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

QVector<int> CalculateProcess::getBagOfWords(const FeatureSet& codeBook, const FeatureSet& activity, int vocabularySize)
{
    QVector<int> codeWords(vocabularySize, 0); //initialize a vector of vocabularySize, with value 0
    const QList<QList<float> > featureVectors = activity.getFeatureVectors();
    for(int i=0; i<featureVectors.length(); i++)
    {
        codeWords[findNearestCodeword(codeBook, featureVectors[i])]++; //increment the codeword count for the given feature vector
    }

    return codeWords;
}

void CalculateProcess::writeBoWtoFile(QVector<int> &codeWords, QString fileName)
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

void CalculateProcess::setTargetDir(QString pathToFolder)
{
    if (!this->targetDir.setCurrent(pathToFolder))
        this->targetDir.setCurrent(".");
}
