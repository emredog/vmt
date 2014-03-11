#ifndef FEATURESET_H
#define FEATURESET_H

#include <QString>
#include <QFile>
#include <QHash>
#include <QList>

class FeatureSet
{
public:
    FeatureSet();
    FeatureSet(QString fileName, int featDim = 80, int preFeatureCount = 0);

    int getNrOfFeatures() const;
    //dimensionality of feature vector
    int featDim;
    const QList<QList<float> >& getFeatureVectors() const;

protected:

    int ignoreFirstFeatures;
    QList<QList<float> > featureVectors;
};

#endif // ACTIVITY_H
