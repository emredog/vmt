#ifndef CALCULATEPROCESS_H
#define CALCULATEPROCESS_H

#include <QThread>
#include <QStringList>
#include <QDir>

#include "featureset.h"

class CalculateProcess : public QThread
{
    Q_OBJECT
public:
    explicit CalculateProcess(QStringList activityFiles, QDir activityFeaturesDir, FeatureSet codebook, int vocabularySize = 4000,
                              int featDim = 88, int ignoreFeatsOnActs = 8, QObject *parent = 0);
    ~CalculateProcess();

signals:

public slots:

protected:
    void run();

    float calculateSquaredEuclidDistance(const QList<float> &featVect1, const QList<float> &featVect2);
    int findNearestCodeword(const FeatureSet &codebook, const QList<float> &featVect);
    QVector<int> getBagOfWords(const FeatureSet& codeBook, const FeatureSet& activity, int vocabularySize);
    void writeBoWtoFile(QVector<int> &codeWords, QString fileName);

    FeatureSet codeBook;
    QStringList activityFiles;
    QDir activityFeaturesDir;
    int featDim;
    int ignoreFeatsOnActs;
    int vocabularySize;
};

#endif // CALCULATEPROCESS_H
