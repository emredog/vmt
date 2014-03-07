#ifndef FEATURESET_H
#define FEATURESET_H

#include <QString>
#include <QFile>
#include <QHash>
#include <QList>

class FeatureSet
{
public:
    FeatureSet(QString fileName, int preFeatureCount = 0);

    int getNrOfFeatures() const;

    static const int featDim = 80;
    const QList<QList<float> >& getFeatureVectors() const;

protected:

    int ignoreFirstFeatures;
    QList<QList<float> > featureVectors;
};

#endif // ACTIVITY_H
